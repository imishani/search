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
 * \file   pase.hpp
 * \author Hanlan Yang (yanghanlan666@gmail.com)
 * \date   July 15 2024
 */

#include <search/planners/parallel_search/pase.hpp>

namespace ims {

/***Protected***/

bool Pase::independenceCheck(int state_id, const boost::any& popped_vec) {
    auto state = getSearchState(state_id);
    // Check against all the states being expanded.
    for (auto wip : *(work_in_progress_)) {
        if (!wip) {
            continue;
        }
        auto h_diff = computeHeuristic(state_id, wip->state_id);
        if (state->g > wip->g + params_.epsilon_ * h_diff) {
            return false;
        }
    }
    // Check against all the states ahead in the open list.
    for (auto& pop : boost::any_cast<const std::vector<std::shared_ptr<SearchState>>&>(popped_vec)) {
        if (pop->state_id != state_id) {
            auto h_diff = computeHeuristic(state_id, pop->state_id);
            if (state->g > pop->g + params_.epsilon_ * h_diff) {
                return false;
            }
        }
    }
    return true;
}

void Pase::workerLoop(int thread_id) {
    try {
        while (!terminate_) {
            std::unique_lock<LockType> locker(lock_vec_[thread_id]);
            cv_vec_[thread_id].wait(locker, [this, thread_id] { return work_status_[thread_id] == 1; });
            locker.unlock();

            if (terminate_) {
                break;
            }

            expand(work_in_progress_->at(thread_id), thread_id);

            locker.lock();
            work_in_progress_->at(thread_id) = nullptr;
            work_status_[thread_id] = 0;
            locker.unlock();
        }
    } catch (const std::exception& e) {
        std::cerr << "Worker thread " << thread_id << " terminated with exception: " << e.what() << std::endl;
    }
}

void Pase::expand(std::shared_ptr<SearchState> curr_state_ptr, int thread_id) {
    /// Status change, Stats & Debug
    stampTimer(thread_id);
    lock_.lock();
    stats_.lock_time += getTimeFromStamp(thread_id);
    stats_.num_jobs_per_thread[thread_id]++;
    stats_.num_expanded++;

    if (params_.verbose) curr_state_ptr->print("Thread " + std::to_string(thread_id) + " Expanding ");

    // Get the successors
    // TODO: Currently there is no check for Preconditions on whether the action is valid for a state or not
    // (or should it be taken care by the action space?)
    std::vector<std::vector<int>> successor_seqs_state_ids;
    std::vector<std::vector<double>> successor_seqs_transition_costs;
    lock_.unlock();
    stampTimer(thread_id);
    action_space_ptr_->getSuccessors(curr_state_ptr->state_id, successor_seqs_state_ids, successor_seqs_transition_costs);
    stats_.evaluation_time += getTimeFromStamp(thread_id);
    lock_.lock();

    for (size_t i{0}; i < successor_seqs_state_ids.size(); ++i) {
        int successor_id = successor_seqs_state_ids[i].back();
        double cost = std::accumulate(successor_seqs_transition_costs[i].begin(), successor_seqs_transition_costs[i].end(), 0.0);
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

Pase::Pase(const ParallelSearchParams& params) : ParallelSearch(params) {
    // Instantiation
    open_ = std::make_unique<SimpleQueue<SearchState, SearchStateCompare>>();
    work_in_progress_ = std::make_unique<std::vector<std::shared_ptr<SearchState>>>(params.num_threads_ - 1, nullptr);
}

Pase::~Pase() = default;

bool Pase::plan(std::vector<StateType>& path) {
    initializeCheck();

    std::vector<std::shared_ptr<SearchState>> popped_states;
    lock_.lock();

    startTimer();
    // Time out control loop
    while (!terminate_ && !isTimeOut()) {
        std::shared_ptr<SearchState> curr_state_ptr = NULL;

        // Select work loop
        while (!terminate_ && !curr_state_ptr) {
            // Check if there is no more state to work on.
            if (open_->empty() && noWorkInProgress()) {
                // Meaning that the search can't continue.
                terminate_ = true;
                getTimeFromStart(stats_.time);
                stats_.cost = curr_state_ptr->g;
                stats_.num_generated = (int)action_space_ptr_->states_.size();
                stats_.suboptimality = params_.epsilon_;
                if (params_.verbose) {
                    std::cout << "No solution found - exhausted open list" << std::endl;
                }
                lock_.unlock();
                joinThread();
                return false;
            }

            // While loop to select a state to expand.
            while (!curr_state_ptr && !open_->empty()) {
                // curr_state_ptr = std::shared_ptr<SearchState>(open_->min());
                curr_state_ptr = states_[open_->min()->state_id];
                open_->pop();
                popped_states.push_back(curr_state_ptr);

                // Independence check
                if (independenceCheck(curr_state_ptr->state_id, popped_states)) {
                    break;
                } else {
                    curr_state_ptr = NULL;
                }
            }

            // Re-add the popped states to the open list.
            for (auto& s : popped_states) {
                if (!curr_state_ptr) {
                    open_->push(s.get());
                } else {
                    if (curr_state_ptr->state_id != s->state_id) {
                        open_->push(s.get());
                    }
                }
            }
            popped_states.clear();

            // If curr_state_ptr is NULL, meaning that none of the states in the open list can be expanded.
            // Wait for the other threads to finish.
            if (!curr_state_ptr) {
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
            if (isGoalState(curr_state_ptr->state_id)) {
                terminate_ = true;
                curr_state_ptr->setClosed();
                goal_ = curr_state_ptr->state_id;
                getTimeFromStart(stats_.time);
                reconstructPath(path, stats_.transition_costs);
                stats_.cost = curr_state_ptr->g;
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
        curr_state_ptr->setClosed();
        lock_.unlock();

        int thread_id = 0;
        bool work_assinged = false;

        if (params_.num_threads_ == 1) {
            expand(curr_state_ptr, 0);
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
                            std::async(std::launch::async, &Pase::workerLoop, this, thread_id));
                    }
                    locker.lock();
                    work_in_progress_->at(thread_id) = curr_state_ptr;
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

void Pase::initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
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

    int goal_ind_ = action_space_ptr_->getOrCreateRobotState(goals[0]);
    auto goal_ = getOrCreateSearchState(goal_ind_);
    goals_.push_back(goal_ind_);
    this->goal_ = goal_ind_;

    // Evaluate the goal state
    goal_->parent_id = PARENT_TYPE(GOAL);
    heuristic_->setGoal(const_cast<StateType&>(goals[0]));

    for (auto& start : starts) {
        // Evaluate the start state
        int start_ind_ = action_space_ptr_->getOrCreateRobotState(start);
        auto start_ = getOrCreateSearchState(start_ind_);
        start_->parent_id = PARENT_TYPE(START);
        heuristic_->setStart(const_cast<StateType&>(start));
        start_->g = 0;
        start_->h = computeHeuristic(start_ind_);
        start_->f = computeHeuristic(start_ind_);
        // TODO: Temperarily using .get() to get the raw pointer.
        open_->push(start_.get());
        start_->setOpen();
    }
}

void Pase::initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
                             const StateType& start, const StateType& goal) {
    // Action space pointer.
    action_space_ptr_ = action_space_ptr;
    // Clear both.
    action_space_ptr_->resetPlanningData();
    resetPlanningData();

    // Reset the search algorithm and the action space.
    action_space_ptr_->resetPlanningData();
    this->resetPlanningData();

    // Check if start is valid and add it to the action space.
    int start_ind_ = action_space_ptr_->getOrCreateRobotState(start);
    auto start_ = getOrCreateSearchState(start_ind_);

    int goal_ind_ = action_space_ptr_->getOrCreateRobotState(goal);
    auto goal_ = getOrCreateSearchState(goal_ind_);
    goals_.push_back(goal_ind_);
    this->goal_ = goal_ind_;

    // Evaluate the start state
    start_->parent_id = PARENT_TYPE(START);
    heuristic_->setStart(const_cast<StateType&>(start));
    // Evaluate the goal state
    goal_->parent_id = PARENT_TYPE(GOAL);
    heuristic_->setGoal(const_cast<StateType&>(goal));

    // Evaluate the start state.
    start_->g = 0;
    start_->h = computeHeuristic(start_ind_);
    start_->f = computeHeuristic(start_ind_);
    // TODO: Temperarily using .get() to get the raw pointer.
    open_->push(start_.get());
    start_->setOpen();
}

}  // namespace ims