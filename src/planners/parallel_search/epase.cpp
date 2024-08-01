/*
 * Copyright (C) 2024, Hanlan Yang
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Carnegie Mellon University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/*!
 * \file   epase.cpp
 * \author Hanlan Yang (yanghanlan666@gmail.com)
 * \date   July 19 2024
 */

#include <search/planners/parallel_search/epase.hpp>

namespace ims {

/***Protected***/

bool Epase::independenceCheck(int state_id, const boost::any& popped_vec) {
    auto state = getSearchState(state_id);
    // Check against all the states being expanded.
    for (auto w : *(work_in_progress_)) {
        auto wip = std::dynamic_pointer_cast<SearchEdge>(w);
        if (!wip) {
            continue;
        }
        auto h_diff = computeHeuristic(state_id, wip->state_id);
        if (state->g > wip->g + params_.epsilon_ * h_diff) {
            return false;
        }
    }
    // Check against all the states ahead in the open list.
    for (auto& pop : boost::any_cast<const std::vector<std::shared_ptr<SearchEdge>>&>(popped_vec)) {
        if (pop->state_id != state_id) {
            auto h_diff = computeHeuristic(state_id, pop->state_id);
            if (state->g > pop->g + params_.epsilon_ * h_diff) {
                return false;
            }
        }
    }
    return true;
}

void Epase::workerLoop(int thread_id) {
    try {
        while (!terminate_) {
            std::unique_lock<LockType> locker(lock_vec_[thread_id]);
            cv_vec_[thread_id].wait(locker, [this, thread_id] { return work_status_[thread_id] == 1; });
            locker.unlock();

            if (terminate_) {
                break;
            }

            expand(std::dynamic_pointer_cast<SearchEdge>(work_in_progress_->at(thread_id)), thread_id);

            locker.lock();
            work_in_progress_->at(thread_id) = nullptr;
            work_status_[thread_id] = 0;
            locker.unlock();
        }
    } catch (const std::exception& e) {
        std::cerr << "Worker thread " << thread_id << " terminated with exception: " << e.what() << std::endl;
    }
}

void Epase::expandProxy(std::shared_ptr<SearchEdge> curr_edge_ptr, int thread_id) {
    /// Status change, Stats & Debug
    stampTimer();
    lock_.lock();
    stats_.lock_time += getTimeFromStamp();
    stats_.num_expanded++;

    if (params_.verbose) curr_edge_ptr->print("Thread " + std::to_string(thread_id) + " Proxy Expanding ");

    // Get the real edges
    std::vector<int> real_edges;
    if (!action_space_ptr_->createRobotEdgesFromProxy(curr_edge_ptr->edge_id, real_edges)) {
        throw std::runtime_error("Action Space - Failed to create real edges from proxy edge");
    }

    // For epase, the real-edge's expansion priority is based on the proxy edge's f-value.
    double priority = curr_edge_ptr->edge_priority;

    // Create the real edges and add them to the open list.
    for (auto i : real_edges) {
        auto real_edge_ptr = getOrCreateSearchEdge(i);
        // Should be running into these case where the real edge is already in closed/opened.
        // Since the proxy edge will take care of the update in the open list.
        setEdgeVals(real_edge_ptr->edge_id, curr_edge_ptr->state_id, priority);
        edge_open_->push(real_edge_ptr.get());
        real_edge_ptr->setOpen();
        notifyMainThread();
    }
    lock_.unlock();
}

void Epase::expand(std::shared_ptr<SearchEdge> curr_edge_ptr, int thread_id) {
    /// Status change, Stats & Debug
    stampTimer();
    lock_.lock();
    stats_.lock_time += getTimeFromStamp();

    // If the edge is a proxy edge, expand it and add real-edges into open list.
    if (curr_edge_ptr->isState()) {
        lock_.unlock();
        expandProxy(curr_edge_ptr, thread_id);
        return;
    }

    stats_.num_jobs_per_thread[thread_id]++;

    if (params_.verbose) curr_edge_ptr->print("Thread " + std::to_string(thread_id) + " Expanding ");

    // Get the successors
    // TODO: Currently there is no check for Preconditions on whether the action is valid for a state or not
    // (or should it be taken care by the action space?)
    std::vector<int> successors;
    std::vector<double> costs;
    lock_.unlock();
    stampTimer();
    action_space_ptr_->getSuccessors(curr_state_ptr->state_id, successors, costs);
    stats_.evaluation_time += getTimeFromStamp();
    lock_.lock();

    for (size_t i{0}; i < successors.size(); ++i) {
        int successor_id = successors[i];
        double cost = costs[i];
        auto successor_ptr = getOrCreateSearchState(successor_id);

        // If the successor is already closed, skip it.
        // Another variant of this algorithm would be to create a new search state for the successor and add it to the open list.

        if (successor_ptr->in_closed) {
            continue;
        }
        if (successor_ptr->in_open) {
            if (successor_ptr->g > curr_state_ptr->g + cost) {
                successor_ptr->parent_id = curr_state_ptr->state_id;
                successor_ptr->g = curr_state_ptr->g + cost;
                successor_ptr->f = successor_ptr->g + params_.epsilon_ * successor_ptr->h;
                open_->update(successor_ptr.get());
            }
        } else {
            setStateVals(successor_ptr->state_id, curr_state_ptr->state_id, cost);
            open_->push(successor_ptr.get());
            successor_ptr->setOpen();
        }
        notifyMainThread();
    }
    lock_.unlock();
}

/***Public***/

Epase::Epase(const ParallelSearchParams& params) : ParallelSearch(params) {
}

Epase::~Epase() = default;

bool Epase::plan(std::vector<StateType>& path) {
    initializeCheck();

    std::vector<std::shared_ptr<SearchEdge>> popped_edges;
    lock_.lock();

    startTimer();
    // Time out control loop
    while (!terminate_ && !isTimeOut()) {
        std::shared_ptr<SearchEdge> curr_edge_ptr = nullptr;

        // Select work loop
        while (!terminate_ && !curr_edge_ptr) {
            // Check if there is no more state to work on.
            if (edge_open_->empty() && noWorkInProgress()) {
                // Meaning that the search can't continue.
                terminate_ = true;
                getTimeFromStart(stats_.time);
                stats_.cost = curr_edge_ptr->g;
                stats_.num_generated = (int)action_space_ptr_->states_.size();
                stats_.suboptimality = params_.epsilon_;
                if (params_.verbose) {
                    std::cout << "No solution found - exhausted open list" << std::endl;
                }
                lock_.unlock();
                joinThread();
                return false;
            }

            // While loop to select an edge to expand.
            while (!curr_edge_ptr && !edge_open_->empty()) {
                curr_edge_ptr = edges_[edge_open_->min()->edge_id];
                edge_open_->pop();
                popped_edges.push_back(curr_edge_ptr);

                // Independence check
                if (independenceCheck(curr_edge_ptr->state_id, popped_edges)) {
                    break;
                } else {
                    curr_edge_ptr = nullptr;
                }
            }

            // Re-add the popped edges to the open list.
            for (auto& s : popped_edges) {
                if (!curr_edge_ptr) {
                    edge_open_->push(s.get());
                } else {
                    if (curr_edge_ptr->edge_id != s->edge_id) {
                        edge_open_->push(s.get());
                    }
                }
            }
            popped_edges.clear();

            // If curr_state_ptr is NULL, meaning that none of the edges in the open list can be expanded.
            // Wait for the other threads to finish.
            if (!curr_edge_ptr) {
                lock_.unlock();
                // Wait for recheck_flag_ to be set true
                std::unique_lock<std::mutex> locker(lock_);
                cv_.wait(locker, [this]() { return (recheck_flag_ == 1); });
                recheck_flag_ = false;
                locker.unlock();
                lock_.lock();
                continue;
            }

            // If the current state is the goal state, then reconstruct the path.
            if (isGoalState(curr_edge_ptr->edge_id)) {
                terminate_ = true;
                curr_edge_ptr->setClosed();
                goal_ = curr_edge_ptr->state_id;
                getTimeFromStart(stats_.time);
                reconstructPath(path, stats_.transition_costs);
                stats_.cost = curr_edge_ptr->g;
                stats_.path_length = (int)path.size();
                stats_.num_generated = (int)action_space_ptr_->states_.size();
                stats_.suboptimality = params_.epsilon_;
                if (params_.verbose) {
                    std::cout << "Solution found" << std::endl;
                }
                lock_.unlock();
                joinThread();
                return true;
            }
        }

        // Expand the current state.
        curr_edge_ptr->setClosed();
        lock_.unlock();

        int thread_id = 0;
        bool work_assinged = false;

        if (params_.num_threads_ == 1) {
            expand(curr_edge_ptr, 0);
        } else {
            while (!work_assinged) {
                // Check if thread with [id] is available.
                std::unique_lock<LockType> locker(lock_vec_[thread_id]);
                int working = work_status_[thread_id];
                locker.unlock();

                // If available
                if (!working) {
                    int num_threads_current = work_futures_.size();
                    if (thread_id >= num_threads_current) {
                        if (params_.verbose) std::cout << "Spawning worker thread " << thread_id << std::endl;
                        work_futures_.emplace_back(
                            std::async(std::launch::async, &Epase::workerLoop, this, thread_id));
                    }
                    locker.lock();
                    work_in_progress_->at(thread_id) = curr_edge_ptr;
                    work_status_[thread_id] = 1;
                    work_assinged = true;
                    locker.unlock();
                    cv_vec_[thread_id].notify_one();
                } else {
                    // Note that the main thread will have the id thread_num - 1.
                    thread_id = thread_id == params_.num_threads_ - 2 ? 0 : thread_id + 1;
                }
            }
        }

        lock_.lock();
    }

    // If the search is terminated due to time out.
    terminate_ = true;
    getTimeFromStart(stats_.time);
    stats_.num_generated = (int)action_space_ptr_->states_.size();
    stats_.suboptimality = params_.epsilon_;
    if (params_.verbose) {
        std::cout << "No solution found - timeout" << std::endl;
    }
    lock_.unlock();
    joinThread();
    return false;
}

void Epase::initializePlanner(const std::shared_ptr<EdgeActionSpace>& action_space_ptr,
                              const std::vector<StateType>& starts,
                              const std::vector<StateType>& goals) {
    // Action space pointer.
    action_space_ptr_ = action_space_ptr;
    // Clear both.
    action_space_ptr_->resetPlanningData();
    this->resetPlanningData();

    if (goals.empty() || starts.empty()) {
        throw std::runtime_error("Starts or goals are empty");
    }

    if (goals.size() > 1) {
        throw std::runtime_error("Currently, only one goal is supported");
    }

    // Creating the goal state
    int goal_state_ind_ = action_space_ptr_->getOrCreateRobotState(goals[0]);
    int goal_edge_ind_ = action_space_ptr_->createProxyEdgeFromState(goal_state_ind_);
    auto goal_state_ = getOrCreateSearchState(goal_state_ind_);
    auto goal_edge_ = getOrCreateProxyEdge(goal_edge_ind_, goal_state_ind_);

    // Evaluate the goal state
    goal_state_->parent_id = PARENT_TYPE(GOAL);
    setProxyVals(goal_edge_ind_);
    heuristic_->setGoal(const_cast<StateType&>(goals[0]));
    goals_.push_back(goal_edge_ind_);
    this->goal_ = goal_edge_ind_;

    for (auto& start : starts) {
        // Creating the start state
        int start_state_ind_ = action_space_ptr_->getOrCreateRobotState(start);
        int start_edge_ind_ = action_space_ptr_->createProxyEdgeFromState(start_state_ind_);
        auto start_state_ = getOrCreateSearchState(start_state_ind_);
        auto start_edge_ = getOrCreateProxyEdge(start_edge_ind_, start_state_ind_);
        // Evaluate the start state
        heuristic_->setStart(const_cast<StateType&>(start));
        start_state_->parent_id = PARENT_TYPE(START);
        start_state_->g = 0;
        start_state_->h = computeHeuristic(start_state_ind_);
        start_state_->f = computeHeuristic(start_state_ind_);
        setProxyVals(start_edge_ind_);
        // TODO: Temperarily using .get() to get the raw pointer.
        edge_open_->push(start_edge_.get());
        start_edge_->setOpen();
    }
}

void Epase::initializePlanner(const std::shared_ptr<EdgeActionSpace>& action_space_ptr,
                              const StateType& start, const StateType& goal) {
    // Action space pointer.
    action_space_ptr_ = action_space_ptr;
    // Clear both.
    action_space_ptr_->resetPlanningData();
    this->resetPlanningData();

    // Creating the goal state
    int goal_state_ind_ = action_space_ptr_->getOrCreateRobotState(goal);
    int goal_edge_ind_ = action_space_ptr_->createProxyEdgeFromState(goal_state_ind_);
    auto goal_state_ = getOrCreateSearchState(goal_state_ind_);
    auto goal_edge_ = getOrCreateProxyEdge(goal_edge_ind_, goal_state_ind_);

    // Evaluate the goal state
    goal_state_->parent_id = PARENT_TYPE(GOAL);
    setProxyVals(goal_edge_ind_);
    heuristic_->setGoal(const_cast<StateType&>(goal));
    goals_.push_back(goal_edge_ind_);
    this->goal_ = goal_edge_ind_;

    // Creating the start state
    int start_state_ind_ = action_space_ptr_->getOrCreateRobotState(start);
    int start_edge_ind_ = action_space_ptr_->createProxyEdgeFromState(start_state_ind_);
    auto start_state_ = getOrCreateSearchState(start_state_ind_);
    auto start_edge_ = getOrCreateProxyEdge(start_edge_ind_, start_state_ind_);

    // Evaluate the start state
    heuristic_->setStart(const_cast<StateType&>(start));
    start_state_->parent_id = PARENT_TYPE(START);
    start_state_->g = 0;
    start_state_->h = computeHeuristic(start_state_ind_);
    start_state_->f = computeHeuristic(start_state_ind_);
    setProxyVals(start_edge_ind_);
    // TODO: Temperarily using .get() to get the raw pointer.
    edge_open_->push(start_edge_.get());
    start_edge_->setOpen();
}

}  // namespace ims