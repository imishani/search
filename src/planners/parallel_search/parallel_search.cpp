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

ims::ParallelSearch::ParallelSearch(const ParallelSearchParams &params) : Planner(params), params_(params) {
    heuristic_ = params.heuristic_;
    std::vector<LockType> lock_vec(params.num_threads_-1);
    locks_.swap(lock_vec);
}

ims::ParallelSearch::~ParallelSearch() {
    for (auto &state : states_) {
        delete state;
    }
}

auto ims::ParallelSearch::getSearchState(int state_id) -> ims::ParallelSearch::SearchState*{
    assert(state_id < states_.size() && state_id >= 0);
    return states_[state_id];
}

auto ims::ParallelSearch::getOrCreateSearchState(int state_id) -> ims::ParallelSearch::SearchState * {
    if (state_id >= states_.size()){
        states_.resize(state_id + 1, nullptr);
    }
    if (states_[state_id] == nullptr){
        assert(state_id < states_.size() && state_id >= 0);
        states_[state_id] = new SearchState;
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

    SearchState* state = getSearchState(goal_);
    while (state->parent_id != -1){
        path.push_back(action_space_ptr_->getRobotState(state->state_id)->state);
        state = getSearchState(state->parent_id);
    }
    path.push_back(action_space_ptr_->getRobotState(state->state_id)->state);
    std::reverse(path.begin(), path.end());
}

void ims::ParallelSearch::reconstructPath(std::vector<StateType>& path, std::vector<double>& costs) {
    path.clear();
    costs.clear();

    costs.push_back(0); // The goal state gets a transition cost of 0.
    SearchState* state_ = getSearchState(goal_);
    while (state_->parent_id != -1){
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
    if (std::any_of(goals_.begin(), goals_.end(), [&s_id](int goal_ind) {return s_id == goal_ind;})){
        return true;
    }

    // Also ask the action space if this state id is a goal state. Sometimes, states need to be determined as goal in real time.
    // This is EXTREMELY bug prone. Commenting this out for now until we have a proper GoalCondition object.
    // return action_space_ptr_->isGoalState(s_id, goals_);
    return false;
}


void ims::ParallelSearch::resetPlanningData() {
    for (auto state : states_){
        delete state;
    }
    states_ = std::vector<SearchState*>();
    // open_.clear();

    goals_.clear();
    goal_ = -1;
    stats_ = ParallelSearchPlannerStats();
}
