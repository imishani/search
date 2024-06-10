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
 * \file   focal_wsipp.cpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   May 29 2024
*/

#include <search/planners/focal_search/focal_wsipp.hpp>
ims::FocalwSIPP::FocalwSIPP(const ims::FocalwSIPPParams &params) : params_(params), FocalSearch(params) {
}

ims::FocalwSIPP::~FocalwSIPP(){
    for (auto &state : states_){
        delete state;
    }
}

//void ims::FocalwSIPP::initializePlanner(const std::shared_ptr<SubcostActionSpace> &action_space_ptr,
//                                    const std::vector<StateType> &starts,
//                                    const std::vector<StateType> &goals) {
//    // space pointer
//    action_space_ptr_ = action_space_ptr;
//    // Clear both.
//    action_space_ptr_->resetPlanningData();
//    resetPlanningData();
//
//    if (goals.empty() || starts.empty()) {
//        throw std::runtime_error("Starts or goals are empty");
//    }
//
//    if (goals.size() > 1) {
//        throw std::runtime_error("Currently, only one goal is supported");
//    }
//
//    // Check if the goal is valid.
//    if (!action_space_ptr_->isStateValid(goals[0])){
//        throw std::runtime_error("Goal state is not valid");
//    }
//    int goal_ind_ = action_space_ptr_->getOrCreateRobotState(goals[0]);
//    auto goal_ = getOrCreateSearchState(goal_ind_);
//    goals_.push_back(goal_ind_);
//
//    // Evaluate the goal state with the heuristic.
//    goal_->parent_id = PARENT_TYPE(GOAL);
//    heuristic_->setGoal(const_cast<StateType &>(goals[0]));
//    goal_->h = 0;
//
//    for (auto &start : starts) {
//        // check if start is valid
//        if (!action_space_ptr_->isStateValid(start)){
//            throw std::runtime_error("Start state is not valid");
//        }
//        // Evaluate the start state
//        int start_ind_ = action_space_ptr_->getOrCreateRobotState(start);
//        auto start_ = getOrCreateSearchState(start_ind_);
//        start_->parent_id = PARENT_TYPE(START);
//        heuristic_->setStart(const_cast<StateType &>(start));
//        start_->g = 0;
//        start_->c = 0;
//        start_->h = computeHeuristic(start_ind_);
//        start_->f = start_->g + params_.epsilon*start_->h;
//        open_.push(start_);
//        start_->setOpen();
//    }
//
//    // Update stats suboptimality.
//    this->stats_.suboptimality = params_.epsilon;
//    this->stats_.focal_suboptimality = params_.focal_suboptimality;
//}

void ims::FocalwSIPP::initializePlanner(const std::shared_ptr<SubcostConstrainedActionSpace>& action_space_ptr,
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
        throw std::runtime_error("Start state is not valid");
    }
    // check if goal is valid
    if (!action_space_ptr_->isStateValid(goal)){
        throw std::runtime_error("Goal state is not valid");
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
    ims::FocalwSIPP::SearchState* start_state = getOrCreateSearchStateFromCfgIdAndSafeInterval(start_cfg_id, start_safe_intervals[0]);
    int start_id = start_state->state_id;
    start_state->parent_id = PARENT_TYPE(START);
    // Evaluate the start state
    start_state->g = start_time;
    start_state->f = start_state->g + params_.epsilon*start_state->h;
    start_state->c = 0;
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
    ims::FocalwSIPP::SearchState* goal_state = getOrCreateSearchStateFromCfgIdAndSafeInterval(goal_cfg_id, goal_safe_intervals.back());
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
    this->stats_.focal_suboptimality = params_.focal_suboptimality;
}

bool ims::FocalwSIPP::plan(std::vector<StateType>& path) {
    startTimer();
    int iter {0};

    // Reorder the open list.
    double open_list_f_lower_bound = open_.getLowerBound();
    open_.updateWithBound(params_.focal_suboptimality * open_list_f_lower_bound);

    while (!open_.empty() && !isTimeOut()){
        // report progress every 1000 iterations
        if (iter % 100000 == 0 && params_.verbose){
            std::cout << "Iter: " << iter << " open size: " << open_.size() << std::endl;
        }

        // Get a state from the OPEN list and remove it.
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

            // TODO(yoraish): Get the non-weighted (g+h) minimal value from the OPEN list.
            stats_.lower_bound = state->g / params_.epsilon;

            return true;
        }

        // Expand the state.
        expand(state->state_id);
        ++iter;

        // Check if the OPEN list is empty. If so, break.
        if (open_.empty()){
            break;
        }

        // Update the OPEN list. Ensure that the focal bound is at least as large as the minimal f-value in the OPEN list.
        open_list_f_lower_bound = open_.getLowerBound();
        open_.updateWithBound(params_.focal_suboptimality * open_list_f_lower_bound);
    }
    getTimeFromStart(stats_.time);
    return false;
}

void ims::FocalwSIPP::expand(int state_id){

    SearchState* state = getSearchState(state_id);
    StateType state_wo_time = action_space_ptr_->getRobotState(state->cfg_state_id)->state;
    std::vector<int> successors;
    std::vector<double> costs;

    action_space_ptr_->getSuccessors(state->state_id, successors, costs);
    for (size_t i {0} ; i < successors.size() ; ++i){
        int successor_cfg_state_id = successors[i];
        double cost = costs[i];
        int transition_time = (int)cost;
        assert(transition_time != 0);
        StateType succ_state_wo_time = action_space_ptr_->getRobotState(successor_cfg_state_id)->state;
        // The earliest time of reaching a successor.
        TimeType arrival_start_t = (TimeType)state->g + transition_time;
        // The latest time of reaching a successor.
        TimeType arrival_end_t = state->safe_interval.second + transition_time;
        // Iterate over the safe intervals of the successor configuration.
        std::vector<SafeIntervalType> succ_safe_intervals;
        action_space_ptr_->getSafeIntervals(successor_cfg_state_id, succ_safe_intervals);

        //# ============
        for (const SafeIntervalType& succ_safe_interval : succ_safe_intervals){
            // If the safe interval starts after the latest time of reaching the successor, or ends before the earliest time of reaching the successor, then it is not relevant.
            if (succ_safe_interval.first > arrival_end_t || succ_safe_interval.second < arrival_start_t){
                continue;
            }
            // Compute the earliest valid arrival time to the successor. This necessitates a validity check w.r.t. constraints. The static obstacles are already checked in the getSuccessors function.
            TimeType arrival_t = -1;
            double transition_subcost;
            for (TimeType t = arrival_start_t ; t <= std::min(arrival_end_t, succ_safe_interval.second) ; ++t){
                // Construct the configurations of the state and the successor.
                StateType robot_cfg_state{state_wo_time};
                StateType succ_robot_cfg_state{succ_state_wo_time};
                robot_cfg_state.push_back(t - transition_time);
                succ_robot_cfg_state.push_back(t);

                if (action_space_ptr_->isSatisfyingAllConstraints(robot_cfg_state,succ_robot_cfg_state)
                                                                  && t >= succ_safe_interval.first
                                                                  && t <= succ_safe_interval.second){

                    arrival_t = t;
                    // Compute the subcost of the motion (from current safe interval to new safe interval identified.
//                    action_space_ptr_->getTransitionSubcost(robot_cfg_state, succ_robot_cfg_state, transition_subcost);
                    break;
                }
            }
            if (arrival_t == -1){
                continue;
            }

            // Found a valid arrival time. Create a search state for this successor.
            SearchState* successor = getOrCreateSearchStateFromCfgIdAndSafeInterval(successor_cfg_state_id, succ_safe_interval);
            if (successor->in_closed){
                // In WSIPP and also in focal search, closed nodes must be evaluated again and re-queued if a better path is found.
                if (successor->g > arrival_t) {
                    successor->parent_id = state->state_id;
                    successor->g = arrival_t;
                    successor->f = successor->g + params_.epsilon * successor->h;
                    successor->c = state->c + transition_subcost;
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
            // Check if the successor is already in the open list.
            if (successor->in_open){
                // Check if the new arrival time is better than the old one.
                if (successor->g > arrival_t){
                    successor->parent_id = state->state_id;
                    successor->g = arrival_t;
                    successor->f = successor->g + params_.epsilon*successor->h;
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

//
//        SearchState* successor = getOrCreateSearchState(successor_cfg_state_id);
//
//        // If this state does not already exists, then we add it to the open list normally.
//        if (!successor->in_closed && !successor->in_open){
//            setStateVals(successor->state_id, state->state_id, cost, subcost);
//            open_.push(successor);
//            successor->setOpen();
//        }
//
//        // If the state is not new, then we check if it passes the criterion for updating it either in the CLOSED list (update and insert to OPEN) or in the OPEN list (just update).
//        else{
//            // Compute the new tentative f, g, and c values.
//            double g_new = state->g + cost;
//            double c_new = state->c + subcost;
//            double f_new = g_new + params_.epsilon * successor->h;
//
//            // Check the update criterion.
//            if (f_new < successor->f || (f_new == successor->f && c_new < successor->c)){
//                // Update the state's parent and g value.
//                successor->parent_id = state->state_id;
//                successor->g = g_new;
//                successor->c = c_new;
//                successor->f = f_new;
//
//                // If the state is in the closed list, then we remove it from the closed list and insert it to the open list.
//                if (successor->in_closed){
//                    successor->setOpen();
//                    open_.push(successor);
//                }
//
//                // If the state is in the open list, then we update its position in the open list.
//                else if (successor->in_open){
//                    // TODO(yoraish): this may not be needed, as the OPEN list will be reordered after the expansion anyway.
//                    open_.update(successor);
//                }
//
//                // If the state is neither in the open list nor in the closed list, then we throw an error.
//                else{
//                    throw std::runtime_error("Found successor node that is neither in the open list nor in the closed list and is not new.");
//                }
//            }
//        }
//    }
//    stats_.num_expanded++;
//}


//
//
//void ims::FocalwSIPP::setStateVals(int state_id, int parent_id, double cost, double subcost)
//{
//    auto state_ = getSearchState(state_id);
//    auto parent = getSearchState(parent_id);
//    state_->parent_id = parent_id;
//    state_->g = parent->g + cost;
//    state_->c = parent->c + subcost;
//    state_->h = computeHeuristic(state_id);
//    state_->f = state_->g + params_.epsilon*state_->h;
//}
