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

Pase::Pase(const ParallelSearchParams& params) : ParallelSearch(params) {
}

Pase::~Pase() = default;

bool Pase::plan(std::vector<StateType>& path) {
    initializeCheck();

    std::vector<SearchState*> popped_states;
    lock_.lock();

    startTimer();
    while (!terminate_ && !isTimeOut()) {
        if (open_.empty() && noWorkInProgress()) {
            // Meaning that thee search can't continue.
            terminate_ = true;
            getTimeFromStart(stats_.time);
            if (params_.verbose) {
                std::cout << "No solution found" << std::endl;
            }
            lock_.unlock();
            cleanUp();
            return false;
        }
    }
    return false;
}

void Pase::initializeCheck() const {
    if (action_space_ptr_ == nullptr) {
        throw std::runtime_error("Action space is not initialized");
    }
    if (heuristic_ == nullptr) {
        throw std::runtime_error("Heuristic is not initialized");
    }
    if (goals_.empty() || goal_ == -1) {
        throw std::runtime_error("Goals are not initialized");
    }
    if (open_.empty()) {
        throw std::runtime_error("Open list is empty, make sure to initialize the planner before run");
    }
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
        start_->f = computeHeuristic(start_ind_);
        open_.push(start_);
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

    // Evaluate the start state
    start_->parent_id = PARENT_TYPE(START);
    heuristic_->setStart(const_cast<StateType&>(start));
    // Evaluate the goal state
    goal_->parent_id = PARENT_TYPE(GOAL);
    heuristic_->setGoal(const_cast<StateType&>(goal));

    // Evaluate the start state.
    start_->g = 0;
    start_->f = computeHeuristic(start_ind_);
    open_.push(start_);
    start_->setOpen();
}

void Pase::cleanUp() {
    for (int thread_id {0}; thread_id < params_.num_threads_; ++thread_id) {
        std::unique_lock<LockType> locker(lock_vec_[thread_id]);
    }
}

void Pase::resetPlanningData() {
    open_.clear();
    work_in_progress_.resize(params_.num_threads_ - 1, NULL);
    ParallelSearch::resetPlanningData();
}

}  // namespace ims