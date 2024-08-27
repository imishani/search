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
 * \file   qpase.cpp
 * \author Hanlan Yang (yanghanlan666@gmail.com)
 * \date   August 19 2024
 */

#include <search/planners/parallel_search/qpase.hpp>

namespace ims {

/***Protected***/

void Qpase::expandProxy(std::shared_ptr<SearchEdge> curr_edge_ptr, int thread_id) {
    /// Status change, Stats & Debug
    stats_.num_expanded++;

    if (params_.verbose) curr_edge_ptr->print("Thread " + std::to_string(thread_id) + " Proxy Expanding ");

    // Get the real edges
    std::vector<int> real_edges;
    if (!action_space_ptr_->createRobotEdgesFromProxy(curr_edge_ptr->edge_id, real_edges)) {
        throw std::runtime_error("Action Space - Failed to create real edges from proxy edge");
    }

    stampTimer(thread_id);
    lock_.lock();
    stats_.lock_time += getTimeFromStamp(thread_id);

    // Create the real edges and add them to the open list.
    for (auto i : real_edges) {
        auto real_edge_ptr = getOrCreateSearchEdge(i);
        // For qpase, the real-edge's expansion priority is g + epsilon * q-value
        double cost{0}, next_h{0};
        lock_.unlock();
        stampTimer(thread_id);
        this->action_space_ptr_->getQValue(i, cost, next_h);
        stats_.q_time += getTimeFromStamp(thread_id);
        lock_.lock();
        double priority = curr_edge_ptr->edge_priority;
        if (cost > 0) {
            priority = curr_edge_ptr->g + cost + params_.epsilon_ * next_h;
            // priority = curr_edge_ptr->g + params_.epsilon_ * (cost + next_h);
            // priority = curr_edge_ptr->g + params_.epsilon_ * next_h;
        }
        // Shouldn't be running into these case where the real edge is already in closed/opened.
        // Since the proxy edge will take care of the update in the open list.
        setEdgeVals(real_edge_ptr->edge_id, curr_edge_ptr->edge_id, priority);
        edge_open_->push(real_edge_ptr.get());
        real_edge_ptr->setOpen();
        notifyMainThread();
    }
    lock_.unlock();
}

/***Public***/

Qpase::Qpase(const ParallelSearchParams& params) : Epase(params) {
}

Qpase::~Qpase() = default;

void Qpase::initializePlanner(const std::shared_ptr<EdgeActionSpace>& action_space_ptr,
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
    this->action_space_ptr_->setGoal(goals[0]);
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

void Qpase::initializePlanner(const std::shared_ptr<EdgeActionSpace>& action_space_ptr,
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
    this->action_space_ptr_->setGoal(goal);
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