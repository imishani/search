/*
 * Copyright (C) 2024, Yorai Shaoul
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
 * \file   sipp.cpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   2024-01-16
*/

#include <search/planners/sipp.hpp>

ims::SIPP::SIPP(const ims::SIPPParams &params) : params_(params), BestFirstSearch(params) {}

ims::SIPP::~SIPP() {
    for (auto &state : states_) {
        delete state;
    }
}

auto ims::SIPP::getSearchState(int state_id) -> ims::SIPP::SearchState * {
    assert(state_id < states_.size() && state_id >= 0);
    return states_[state_id];
}

auto ims::SIPP::getOrCreateSearchState(int state_id) -> ims::SIPP::SearchState * {
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

auto ims::SIPP::getOrCreateSearchStateFromCfgIdAndSafeInterval(int cfg_state_id, const SafeIntervalType& safe_interval) -> ims::SIPP::SearchState* {
    // Check if the state already exists.
    if (cfg_id_to_state_ids_.find(cfg_state_id) != cfg_id_to_state_ids_.end()){
        // Check if the state with the same safe interval already exists.
        for (auto existing_state_id : cfg_id_to_state_ids_[cfg_state_id]){
            if (states_[existing_state_id]->safe_interval == safe_interval){ // TODO(yoraish): check this.
                return states_[existing_state_id];
            }
        }
    }
    // If not, create a new state.
    int new_state_id = (int)states_.size();
    auto new_state = getOrCreateSearchState(new_state_id);
    new_state->safe_interval = safe_interval;
    new_state->cfg_state_id = cfg_state_id;
    cfg_id_to_state_ids_[cfg_state_id].push_back(new_state_id);
    return new_state;
}

void ims::SIPP::initializePlanner(const std::shared_ptr<ConstrainedActionSpace>& action_space_ptr,
                                   const StateType& start, const StateType& goal) {
    // Space pointer.
    action_space_ptr_ = action_space_ptr;

    // Clear both.
    action_space_ptr_->resetPlanningData();
    resetPlanningData();

    // Get start and goal without time.
    auto start_time = (TimeType)start.back();
    auto goal_time = (TimeType)goal.back();
    StateType start_wo_time{start.begin(), start.end() - 1};
    StateType goal_wo_time{goal.begin(), goal.end() - 1};

    // Check if start is valid.
    if (!action_space_ptr_->isStateValid(start_wo_time)){
        throw std::runtime_error("Start state is not valid.");
    }
    // check if goal is valid
    if (!action_space_ptr_->isStateValid(goal_wo_time)){
        throw std::runtime_error("Goal state is not valid.");
    }

    // Create the objects for the start state.
    int start_cfg_id = action_space_ptr_->getOrCreateRobotState(start_wo_time);
    std::vector<SafeIntervalType> start_safe_intervals;
    action_space_ptr_->getSafeIntervals(start_cfg_id, start_safe_intervals);
    assert(!start_safe_intervals.empty());
    // Check that the first safe interval contains the start time.
    if (start_time < start_safe_intervals[0].first || start_time > start_safe_intervals[0].second){
        throw std::runtime_error("Start time is not in the first safe interval.");
    }
    // Get the search state for the start.
    ims::SIPP::SearchState* start_state = getOrCreateSearchStateFromCfgIdAndSafeInterval(start_cfg_id, start_safe_intervals[0]);
    int start_id = start_state->state_id;
    start_state->parent_id = PARENT_TYPE(START);
    // Evaluate the start state
    start_state->g = start_time;
    start_state->f = start_state->g + params_.epsilon*start_state->h;
    start_state->setOpen();

    // Create the objects for the goal state.
    int goal_cfg_id = action_space_ptr_->getOrCreateRobotState(goal_wo_time);
    // Get the safe intervals for the goal.
    std::vector<SafeIntervalType> goal_safe_intervals;
    action_space_ptr_->getSafeIntervals(goal_cfg_id, goal_safe_intervals);
    assert(!goal_safe_intervals.empty());
    // The last safe interval should contain the goal time.
    if ((goal_time < goal_safe_intervals.back().first || goal_time > goal_safe_intervals.back().second) && goal_time != -1){
        throw std::runtime_error("Goal time is not in the last safe interval.");
    }
    // Get the search state for the goal. We use the last safe interval to ensure that the agent can wait at the goal forever.
    ims::SIPP::SearchState* goal_state = getOrCreateSearchStateFromCfgIdAndSafeInterval(goal_cfg_id, goal_safe_intervals.back());
    int goal_id = goal_state->state_id;
    goals_.push_back(goal_id);

    heuristic_->setStart(const_cast<StateType &>(start_wo_time));
    // Evaluate the goal state
    goal_state->parent_id = PARENT_TYPE(GOAL);
    heuristic_->setGoal(const_cast<StateType &>(goal_wo_time));
    goal_state->h = 0;

    // Set the in open.
    start_state->h = computeHeuristic(start_cfg_id);
    open_.push(start_state);
    // Update the sub-optimality bound in the statistics object.
    stats_.suboptimality = params_.epsilon;
}

bool ims::SIPP::plan(std::vector<StateType>& path) {
    startTimer();
    int iter {0};
    while (!open_.empty() && !isTimeOut()){
        // report progress every 1000 iterations
        if (iter % 10 == 0 && params_.verbose){
            std::cout << "Iter: " << iter << " open size: " << open_.size() << std::endl;
        }
        auto state  = open_.min();
        open_.pop();
        state->setClosed();
        if (isGoalState(state->state_id)){
            goal_ = state->state_id;
            getTimeFromStart(stats_.time);
            reconstructPath(path, stats_.transition_costs);
            stats_.cost = state->g;
            stats_.path_length = (int)path.size();
            stats_.num_generated = (int)action_space_ptr_->states_.size();
            return true;
        }
        expand(state->state_id);
        ++iter;
    }
    getTimeFromStart(stats_.time);
    return false;
}

void ims::SIPP::expand(int state_id){

    auto state = getSearchState(state_id);

    StateType state_wo_time = action_space_ptr_->getRobotState(state->cfg_state_id)->state;
    std::vector<int> successors;
    std::vector<double> costs;
    // The successors are ids of configurations without time. The configurations are stored in the action space.
    action_space_ptr_->getSuccessors(state->cfg_state_id, successors, costs);
    for (size_t i {0} ; i < successors.size() ; ++i){
        int successor_cfg_state_id = successors[i];
        StateType succ_state_wo_time = action_space_ptr_->getRobotState(successor_cfg_state_id)->state;
        double cost = costs[i];
        int transition_time = (int)cost;
        assert(transition_time != 0);
        // The earliest time of reaching a successor.
        TimeType arrival_start_t = (TimeType)state->g + transition_time;
        // The latest time of reaching a successor.
        TimeType arrival_end_t = state->safe_interval.second + transition_time;
        // Iterate over the safe intervals of the successor configuration.
        std::vector<SafeIntervalType> succ_safe_intervals;
        action_space_ptr_->getSafeIntervals(successor_cfg_state_id, succ_safe_intervals);
        for (const SafeIntervalType& succ_safe_interval : succ_safe_intervals){
            // If the safe interval starts after the latest time of reaching the successor, or ends before the earliest time of reaching the successor, then it is not relevant.
            if (succ_safe_interval.first > arrival_end_t || succ_safe_interval.second < arrival_start_t){
                continue;
            }
            // Compute the earliest valid arrival time to the successor. This necessitates a validity check w.r.t. constraints. The static obstacles are already checked in the getSuccessors function.
            TimeType arrival_t = -1;
            for (TimeType t = arrival_start_t ; t <= std::min(arrival_end_t, succ_safe_interval.second) ; ++t){
                // Construct the configurations of the state and the successor.
                StateType robot_cfg_state{state_wo_time};
                StateType succ_robot_cfg_state{succ_state_wo_time};
                robot_cfg_state.push_back(t - transition_time);
                succ_robot_cfg_state.push_back(t);

                if (action_space_ptr_->isSatisfyingAllConstraints(robot_cfg_state,succ_robot_cfg_state)
                                                                  && t >= succ_safe_interval.first
                                                                  && t <= succ_safe_interval.second){
                    if (params_.verbose){
                        std::cout << GREEN << " * Found valid arrival time: " << t << " from " << action_space_ptr_->getRobotState(state->cfg_state_id)->state << " to " << action_space_ptr_->getRobotState(successor_cfg_state_id)->state << RESET << std::endl;
                    }
                    
                    arrival_t = t;
                    break;
                }
                else{
                    if (params_.verbose){
                        std::cout << RED << " * Invalid arrival time: " << t << " from " << action_space_ptr_->getRobotState(state->cfg_state_id)->state << " to " << action_space_ptr_->getRobotState(successor_cfg_state_id)->state << RESET << std::endl;
                        std::cout << RED << " * The reason is: ";
                        if (!action_space_ptr_->isSatisfyingAllConstraints(robot_cfg_state,succ_robot_cfg_state)){
                            std::cout << "Constraints are not satisfied." << std::endl;
                        }
                        if (t < succ_safe_interval.first || t > succ_safe_interval.second){
                            std::cout << "Arrival time is not in the safe interval." << std::endl;
                        }
                        std::cout << RESET;
                    }
                }
            }
            if (arrival_t == -1){
                continue;
            }

            // Found a valid arrival time. Create a search state for this successor.
            SearchState* successor = getOrCreateSearchStateFromCfgIdAndSafeInterval(successor_cfg_state_id, succ_safe_interval);
            if (successor->in_closed){
                continue;
            }
            // Check if the successor is already in the open list.
            if (successor->in_open){
                // Check if the new arrival time is better than the old one.
                if (successor->g > arrival_t){
                    successor->parent_id = state->state_id;
                    successor->g = arrival_t;
                    successor->f = successor->g + params_.epsilon*successor->h;
                    open_.update(successor);
                }
            } else {
                successor->parent_id = state->state_id;
                successor->g = arrival_t;
                successor->h = computeHeuristic(successor_cfg_state_id);
                successor->f = successor->g + params_.epsilon*successor->h;
                open_.push(successor);
                successor->setOpen();
            }
        }
    }
    stats_.num_expanded++;
}

double ims::SIPP::computeHeuristic(int state_id) {
    double dist;
    auto s = action_space_ptr_->getRobotState(state_id);
    if (!heuristic_->getHeuristic(s->state, dist)) {
        throw std::runtime_error("Heuristic function failed");
    }
    else {
        return dist;
    }
}

double ims::SIPP::computeHeuristic(int s1_id, int s2_id) {
    double dist;
    auto s1 = action_space_ptr_->getRobotState(s1_id);
    auto s2 = action_space_ptr_->getRobotState(s2_id);
    if (!heuristic_->getHeuristic(s1->state, s2->state, dist)) {
        throw std::runtime_error("Heuristic function failed");
    }
    else {
        return dist;
    }
}

void ims::SIPP::reconstructPath(std::vector<StateType>& path, std::vector<double>& costs) {
    path.clear();
    costs.clear();

    SearchState* search_state = getSearchState(goal_);
    auto goal_time = (TimeType)search_state->g;
    // Add the goal state.
    StateType goal_state = action_space_ptr_->getRobotState(search_state->cfg_state_id)->state;
    goal_state.push_back(goal_time);
    path.push_back(goal_state);
    costs.push_back(0); // The goal state gets a transition cost of 0.
    // Start from the goal and go back to the start.
    search_state = getSearchState(search_state->parent_id);

    while (search_state->parent_id != START){
        // Get the arrival time of the state.
        auto state_time = (TimeType)search_state->g;
        // Get the previous arrival time.
        auto child_state_time = (TimeType)path.back().back();
        // Get the state configuration.
        StateType state = action_space_ptr_->getRobotState(search_state->cfg_state_id)->state;

        // If the state time is not exactly 1 unit away from the parent time, then we'll add more than one states waiting at this configuration.
        int num_states_to_add = child_state_time - state_time;
        double transition_cost = 1; // search_state->g - getSearchState(search_state->parent_id)->g;
        double cost = transition_cost;

        // Add the states.
        for (int i = 0 ; i < num_states_to_add ; ++i){
            StateType state_to_add = state;
            state_to_add.push_back(state_time + num_states_to_add - 1 - i);
            path.push_back(state_to_add);
            costs.push_back(cost);
        }

        search_state = getSearchState(search_state->parent_id);
    }
    // Add the start state.
    // Get the arrival time of the state.
    auto state_time = (TimeType)search_state->g;
    // Get the previous arrival time.
    auto child_state_time = (TimeType)path.back().back();
    // Get the state configuration.
    StateType state = action_space_ptr_->getRobotState(search_state->cfg_state_id)->state;

    // If the state time is not exactly 1 unit away from the parent time, then we'll add more than one states waiting at this configuration.
    int num_states_to_add = child_state_time - state_time;
    double transition_cost = 1 ;  // search_state->g;
    double cost = transition_cost;

    // Add the states.
    for (int i = 0 ; i < num_states_to_add ; ++i){
        StateType state_to_add = state;
        state_to_add.push_back(state_time + num_states_to_add - 1 - i);
        path.push_back(state_to_add);
        costs.push_back(cost);
    }

    std::reverse(path.begin(), path.end());
    std::reverse(costs.begin(), costs.end());

    // Print the path.
    if (params_.verbose){
        std::cout << "Path: " << std::endl;
        for (size_t i {0} ; i < path.size() ; ++i){
            std::cout << path[i] << " " << costs[i] << std::endl;
        }
    }
}

void ims::SIPP::reconstructPath(std::vector<StateType>& path) {
    std::vector<double> costs;
    reconstructPath(path, costs);
}

void ims::SIPP::resetPlanningData(){
    for (auto state_ : states_){
        delete state_;
    }
    states_.clear();
    open_.clear();
    goals_.clear();
    goal_ = -1;
    stats_ = PlannerStats();
    cfg_id_to_state_ids_.clear();
}

auto ims::SIPP::getAllSearchStates() -> std::vector<ims::SIPP::SearchState*> {
    return states_;
}