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
    if (!action_space_ptr_->isStateValid(start)){
        throw std::runtime_error("Start state is not valid.");
    }
    // check if goal is valid
    if (!action_space_ptr_->isStateValid(goal)){
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
    std::vector<std::vector<int>> successor_seqs_state_ids;
    std::vector<std::vector<double>> successor_seqs_transition_costs;
    // The successors are ids of configurations without time. The configurations are stored in the action space.
    action_space_ptr_->getSuccessorSequences(state->cfg_state_id, successor_seqs_state_ids, successor_seqs_transition_costs);
    for (size_t i {0} ; i < successor_seqs_state_ids.size() ; ++i){
        const std::vector<int> & successor_edge_state_ids = successor_seqs_state_ids[i];
        int successor_cfg_state_id = successor_edge_state_ids.back();
        StateType succ_state_wo_time = action_space_ptr_->getRobotState(successor_cfg_state_id)->state;
        double successor_edge_total_cost = vectorSum(successor_seqs_transition_costs[i]);
        int transition_time = (TimeType)successor_edge_total_cost;
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
                // Construct the configurations of the state, the intermediate states, and the successor.
                std::vector<StateType> robot_cfg_edge_states;
                for (int j {0} ; j < (int)successor_edge_state_ids.size() ; ++j) {
                    int edge_cfg_state_id = successor_edge_state_ids[j];
                    StateType edge_cfg_state = action_space_ptr_->getRobotState(edge_cfg_state_id)->state;
                    edge_cfg_state.push_back(t - transition_time + j);
                    robot_cfg_edge_states.push_back(edge_cfg_state);
                    assert(transition_time == vectorSum(successor_seqs_transition_costs[i]));
                }

                if (isTimedCfgPathSatisfyingAllConstraints(robot_cfg_edge_states)
                                                                  && t >= succ_safe_interval.first
                                                                  && t <= succ_safe_interval.second){
                    if (params_.verbose){
                        std::cout << GREEN
                                  << " * Found valid arrival time: "
                                  << t
                                  << " from state id " << state->cfg_state_id << " cfg "
                                  << action_space_ptr_->getRobotState(state->cfg_state_id)->state
                                  << " to state id " << successor_cfg_state_id << " cfg "
                                  << action_space_ptr_->getRobotState(successor_cfg_state_id)->state
                                  << RESET
                                  << std::endl;
                    }
                    
                    arrival_t = t;
                    break;
                }
                else{
                    if (params_.verbose){
                        std::cout << RED << " * Invalid arrival time: " << t << " from " << action_space_ptr_->getRobotState(state->cfg_state_id)->state << " to " << action_space_ptr_->getRobotState(successor_cfg_state_id)->state << RESET << std::endl;
                        std::cout << RED << " * The reason is: ";
                        if (isTimedCfgPathSatisfyingAllConstraints(robot_cfg_edge_states)){
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
                // In WSIPP, closed nodes must be evaluated again and re-queued if a better path is found.
                if (params_.epsilon == 1){
                    continue;
                }
                else{
                    if (successor->g > arrival_t) {
                        successor->parent_id = state->state_id;
                        successor->g = arrival_t;
                        successor->f = successor->g + params_.epsilon * successor->h;
                        successor->edge_from_parent_state_ids = successor_edge_state_ids;
                        successor->edge_from_parent_transition_costs = successor_seqs_transition_costs[i];
                        open_.push(successor);
                        successor->setOpen();
                        if (params_.verbose) {
                            std::cout << "State id " << successor->state_id << " gets parent id " << state->state_id
                                      << " with g " << successor->g << " and f " << successor->f << " [reopened]"
                                      << std::endl;
                        }
                    }
                    continue;
                }
            }
            // Check if the successor is already in the open list.
            if (successor->in_open){
                // Check if the new arrival time is better than the old one.
                if (successor->g > arrival_t){
                    successor->parent_id = state->state_id;
                    successor->g = arrival_t;
                    successor->f = successor->g + params_.epsilon*successor->h;
                    successor->edge_from_parent_state_ids = successor_edge_state_ids;
                    successor->edge_from_parent_transition_costs = successor_seqs_transition_costs[i];
                    open_.update(successor);
                    if (params_.verbose) {
                        std::cout << "State id " << successor->state_id << " gets parent id " << state->state_id
                                  << " with g " << successor->g << " and f " << successor->f << " [updated]"
                                  << std::endl;
                    }
                }
            } else {
                successor->parent_id = state->state_id;
                successor->g = arrival_t;
                successor->h = computeHeuristic(successor_cfg_state_id);
                successor->f = successor->g + params_.epsilon*successor->h;
                successor->edge_from_parent_state_ids = successor_edge_state_ids;
                successor->edge_from_parent_transition_costs = successor_seqs_transition_costs[i];
                successor->setOpen();
                open_.push(successor);
                if (params_.verbose) {
                    std::cout << "State id " << successor->state_id << " gets parent id " << state->state_id
                              << " with g " << successor->g << " and f " << successor->f << " [pushed new]"
                              << std::endl;
                }
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
    // If the goal is at time 0, then return.
    if (goal_time == 0){
        return;
    }
    while (search_state->parent_id != START){
        auto parent_search_state = getSearchState(search_state->parent_id);
        // Two options to deal with adding a state to the reverse path.
        // First, if there are intermediate edge states from the added state to the existing state.
        int edge_from_parent_num_states = (int)search_state->edge_from_parent_state_ids.size();
        // Assertion for convention: cost between any two states should be exactly 1.
        assert(edge_from_parent_num_states == (int)vectorSum(search_state->edge_from_parent_transition_costs) + 1);
        // The edge from the parent includes the parent and the child state. If there are additional ones, insert them.
        if (edge_from_parent_num_states > 2){
            // The child state is already in the path. Start adding states from the edge.
            for (int i{edge_from_parent_num_states - 2}; i >= 0; --i) {
                // Get the state id.
                int edge_cfg_state_id = search_state->edge_from_parent_state_ids[i];
                // Get the transition cost from this state to its child.
                double transition_cost = search_state->edge_from_parent_transition_costs[i];
                // Add the timed configuration of this state and the cost to the path.
                StateType state_to_add = action_space_ptr_->getRobotState(edge_cfg_state_id)->state;
                state_to_add.push_back(path.back().back() - transition_cost);
                path.push_back(state_to_add);
                costs.push_back(transition_cost);
            }
            // Now, we must check if the edge total cost is equal to the cost difference between the state and the child.
            // Get the arrival time at the existing state and its parent that we just added.
            auto child_state_time = (TimeType)path.back().back();
            auto parent_state_time = (TimeType)parent_search_state->g;
            // Get the number of timesteps between the parent and child. This is the number of states that should exist
            // from the parent state (inclusive) to the child (exclusive).
            int num_states_needed_parent_until_child = child_state_time - parent_state_time;
            // Get the number of states that should be added waiting at the parent. Starting from its g-value (inclusive).
            int num_states_to_add = child_state_time - parent_state_time;
            // Add the necessary number of waiting states.
            for (int i{0}; i < num_states_to_add; ++i) {
                StateType state_to_add = action_space_ptr_->getRobotState(parent_search_state->cfg_state_id)->state;
                state_to_add.push_back(parent_state_time + num_states_to_add - 1 - i);
                path.push_back(state_to_add);
                costs.push_back(1); // The cost of waiting is 1.
            }
            // Set the search state to the parent.
            search_state = parent_search_state;
        }
        // Second, if there are no intermediate edge states from the added state to the existing state, just add the parent with the associated cost.
        else{
            // Get the transition cost from the parent to the child.
            double transition_cost = 1; // By convention, if only one state in edge then the time transitioning must be 1.
            // Add the parent to the path.
            StateType parent_state_to_add = action_space_ptr_->getRobotState(parent_search_state->cfg_state_id)->state;
            parent_state_to_add.push_back(search_state->g - 1);
            path.push_back(parent_state_to_add);
            costs.push_back(transition_cost);
            // Now that we have added the edge back from the child to the parent (including the parent), we check if there
            // is a need to wait at the parent configuration.
            auto child_state_time = (TimeType)path.back().back();
            auto parent_state_time = (TimeType)parent_search_state->g;
            // Get the number of timesteps between the parent and child. This is the number of states that should exist
            // from the parent state (inclusive) to the child (exclusive).
            int num_states_needed_parent_until_child = child_state_time - parent_state_time;
            // Get the number of states that should be added waiting at the parent. Starting from its g-value (inclusive).
            int num_states_to_add = num_states_needed_parent_until_child;
            // Add the necessary number of waiting states.
            for (int i{0}; i < num_states_to_add; ++i) {
                StateType state_to_add = action_space_ptr_->getRobotState(parent_search_state->cfg_state_id)->state;
                state_to_add.push_back(parent_state_time + num_states_to_add - 1 - i);
                path.push_back(state_to_add);
                costs.push_back(1); // The cost of waiting is 1.
            }
            // Set the search state to the parent.
            search_state = parent_search_state;
        }
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

bool ims::SIPP::isTimedCfgPathSatisfyingAllConstraints(const std::vector<StateType>& edge_robot_cfg_states){
    for (size_t i {0} ; i < edge_robot_cfg_states.size() - 1 ; ++i){
        if (!action_space_ptr_->isSatisfyingAllConstraints(edge_robot_cfg_states[i], edge_robot_cfg_states[i+1])){
            return false;
        }
    }
    return true;
}
