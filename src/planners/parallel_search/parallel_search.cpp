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
 * \file   parallel_search.cpp
 * \author Hanlan Yang (yanghanlan666@gmail.com)
 * \date   July 12 2024
 */

#include <search/planners/parallel_search/parallel_search.hpp>

/***Protected***/

auto ims::ParallelSearch::getSearchState(int state_id) -> std::shared_ptr<ims::ParallelSearch::SearchState> {
    assert(state_id < states_.size() && state_id >= 0);
    return states_[state_id];
}

auto ims::ParallelSearch::getOrCreateSearchState(int state_id) -> std::shared_ptr<ims::ParallelSearch::SearchState> {
    if (state_id >= states_.size()) {
        states_.resize(state_id + 1, nullptr);
    }
    if (states_[state_id] == nullptr) {
        assert(state_id < states_.size() && state_id >= 0);
        states_[state_id] = std::make_shared<SearchState>();
        states_[state_id]->state_id = state_id;
    }
    return states_[state_id];
}

double ims::ParallelSearch::computeHeuristic(int state_id) {
    double dist;
    auto s = action_space_ptr_->getRobotState(state_id);
    if (!heuristic_->getHeuristic(s->state, dist))
        throw std::runtime_error("Heuristic function failed");
    else
        return dist;
}

double ims::ParallelSearch::computeHeuristic(int s1_id, int s2_id) {
    double dist;
    auto s1 = action_space_ptr_->getRobotState(s1_id);
    auto s2 = action_space_ptr_->getRobotState(s2_id);
    if (!heuristic_->getHeuristic(s1->state, s2->state, dist))
        throw std::runtime_error("Heuristic function failed");
    else
        return dist;
}

void ims::ParallelSearch::reconstructPath(std::vector<StateType>& path) {
    path.clear();

    std::shared_ptr<SearchState> state = getSearchState(goal_);
    while (state->parent_id != -1) {
        path.push_back(action_space_ptr_->getRobotState(state->state_id)->state);
        state = getSearchState(state->parent_id);
    }
    path.push_back(action_space_ptr_->getRobotState(state->state_id)->state);
    std::reverse(path.begin(), path.end());
}

void ims::ParallelSearch::reconstructPath(std::vector<StateType>& path, std::vector<double>& costs) {
    path.clear();
    costs.clear();

    costs.push_back(0);  // The goal state gets a transition cost of 0.
    std::shared_ptr<SearchState> state_ = getSearchState(goal_);
    while (state_->parent_id != -1) {
        path.push_back(action_space_ptr_->getRobotState(state_->state_id)->state);

        // Get the transition cost. This is the difference between the g values of the current state and its parent.
        double transition_cost = state_->g - getSearchState(state_->parent_id)->g;
        costs.push_back(transition_cost);

        state_ = getSearchState(state_->parent_id);
    }
    path.push_back(action_space_ptr_->getRobotState(state_->state_id)->state);

    std::reverse(path.begin(), path.end());
    std::reverse(costs.begin(), costs.end());
}

bool ims::ParallelSearch::isGoalState(int s_id) {
    if (std::any_of(goals_.begin(), goals_.end(), [&s_id](int goal_ind) { return s_id == goal_ind; })) {
        return true;
    }

    // Also ask the action space if this state id is a goal state. Sometimes, states need to be determined as goal in real time.
    // This is EXTREMELY bug prone. Commenting this out for now until we have a proper GoalCondition object.
    // return action_space_ptr_->isGoalState(s_id, goals_);
    return false;
}

void ims::ParallelSearch::initializeCheck() const {
    if (action_space_ptr_ == nullptr) {
        throw std::runtime_error("Action space is not initialized");
    }
    if (heuristic_ == nullptr) {
        throw std::runtime_error("Heuristic is not initialized");
    }
    if (goals_.empty() || goal_ == -1) {
        throw std::runtime_error("Goals are not initialized");
    }
    if (open_->empty()) {
        throw std::runtime_error("Open list is empty, make sure to initialize the planner before run");
    }
}

void ims::ParallelSearch::cleanUp() {
    // Notify all worker threads to stop.
    for (int thread_id{0}; thread_id < params_.num_threads_; ++thread_id) {
        std::unique_lock<LockType> locker(lock_vec_[thread_id]);
        work_status_[thread_id] = true;
        locker.unlock();
        cv_vec_[thread_id].notify_one();
    }

    stats_.num_threads_spawned = work_futures_.size() + 1;

    bool all_threads_exit = false;
    while (!all_threads_exit) {
        all_threads_exit = true;
        for (auto& fut : work_futures_) {
            if (!isFutureReady(fut)) {
                all_threads_exit = false;
                break;
            }
        }
    }
    work_futures_.clear();
}

/***Public***/

ims::ParallelSearch::ParallelSearch(const ParallelSearchParams& params) : Planner(params), params_(params) {
    heuristic_ = params.heuristic_;
}

ims::ParallelSearch::~ParallelSearch() {
}

void ims::ParallelSearch::resetPlanningData() {
    // First dealing with raw pointers data structure
    open_->clear();

    states_.clear();
    states_ = std::vector<std::shared_ptr<SearchState>>();

    goals_.clear();
    goal_ = -1;
    stats_ = ParallelSearchPlannerStats();
    stats_.num_jobs_per_thread.resize(params_.num_threads_, 0);

    work_in_progress_->resize(params_.num_threads_ - 1, NULL);
    std::vector<std::condition_variable> cv_vec(params_.num_threads_ - 1);
    cv_vec_.swap(cv_vec);
    std::vector<LockType> lock_vec(params_.num_threads_ - 1);
    lock_vec_.swap(lock_vec);
    work_futures_.clear();
    work_status_.resize(params_.num_threads_ - 1, false);
    terminate_ = false;
    plan_found_ = false;
    recheck_flag_ = true;
}
