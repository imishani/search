/*
 * Copyright (C) 2023, Yorai Shaoul
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
 * \file   focal_search.cpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   Oct 05 2023
 */

#include <search/planners/focal_search/focal_search.hpp>

ims::FocalSearch::FocalSearch(const FocalSearchParams &params) : Planner(params), params_(params) {
    heuristic_ = params.heuristic_;
}

ims::FocalSearch::~FocalSearch() {
    for (auto &state : states_) {
        delete state;
    }
}

void ims::FocalSearch::initializePlanner(const std::shared_ptr<SubcostActionSpace> &action_space_ptr,
                                         const std::vector<StateType> &starts,
                                         const ims::GoalConstraint &goal_constraint) {
    // space pointer
    action_space_ptr_ = action_space_ptr;
    // Clear both.
    action_space_ptr_->resetPlanningData();
    resetPlanningData();

    if (starts.empty()) {
        throw std::runtime_error("Starts or goals are empty");
    }
    this->goal_constraint_ = goal_constraint;
    heuristic_->setGoalConstraint(this->goal_constraint_);

    for (auto &start : starts) {
        // check if start is valid
        if (!action_space_ptr_->isStateValid(start)) {
            throw std::runtime_error("Start state is not valid");
        }
        // Evaluate the start state
        int start_ind_ = action_space_ptr_->getOrCreateRobotState(start);
        auto start_ = getOrCreateSearchState(start_ind_);
        start_->parent_id = PARENT_TYPE(START);
        heuristic_->setStart(const_cast<StateType &>(start));
        start_->g = 0;
        start_->c = 0;
        start_->f = computeHeuristic(start_ind_);
        open_.push(start_);
        start_->setOpen();
    }
}

void ims::FocalSearch::initializePlanner(const std::shared_ptr<SubcostActionSpace> &action_space_ptr,
                                             const std::vector<StateType> &starts,
                                             const std::vector<StateType> &goals) {
    // Action space pointer.
    action_space_ptr_ = action_space_ptr;
    // Clear both.
    action_space_ptr_->resetPlanningData();
    resetPlanningData();

    if (goals.empty() || starts.empty()) {
        throw std::runtime_error("Starts or goals are empty");
    }

    for (auto &goal : goals) {
        if (!action_space_ptr_->isStateValid(goal)){
            throw std::runtime_error("Goal state is not valid");
        }
        int goal_ind_ = action_space_ptr_->getOrCreateRobotState(goal);
        auto goal_ = getOrCreateSearchState(goal_ind_);
        goal_->parent_id = PARENT_TYPE(GOAL);
        goals_.push_back(goal_ind_);
    }

    // Evaluate the goal state
    goal_constraint_.type = MULTI_SEARCH_STATE_GOAL;
    goal_constraint_.check_goal = &multiGoalConstraint;
    goal_constraint_.check_goal_user = &goals_;
    goal_constraint_.action_space_ptr = action_space_ptr_;

    heuristic_->setGoalConstraint(goal_constraint_);

    for (auto &start : starts) {
        // check if start is valid
        if (!action_space_ptr_->isStateValid(start)){
            throw std::runtime_error("Start state is not valid");
        }
        // Evaluate the start state
        int start_ind_ = action_space_ptr_->getOrCreateRobotState(start);
        auto start_ = getOrCreateSearchState(start_ind_);
        start_->parent_id = PARENT_TYPE(START);
        heuristic_->setStart(const_cast<StateType &>(start));
        start_->g = 0;
        start_->c = 0;
        start_->f = computeHeuristic(start_ind_);
        open_.push(start_);
        start_->setOpen();
    }
}

void ims::FocalSearch::initializePlanner(const std::shared_ptr<SubcostActionSpace>& action_space_ptr,
                                             const StateType& start, const StateType& goal) {
    // Action space pointer.
    action_space_ptr_ = action_space_ptr;
    // Clear both.
    action_space_ptr_->resetPlanningData();
    resetPlanningData();
// check if start is valid
    if (!action_space_ptr_->isStateValid(start)){
        throw std::runtime_error("Start state is not valid");
    }
    // check if goal is valid
    if (!action_space_ptr_->isStateValid(goal)){
        throw std::runtime_error("Goal state is not valid");
    }

    // Check if start is valid and add it to the action space.
    int start_ind_ = action_space_ptr_->getOrCreateRobotState(start);
    auto start_ = getOrCreateSearchState(start_ind_);

    int goal_ind_ = action_space_ptr_->getOrCreateRobotState(goal);
    auto goal_ = getOrCreateSearchState(goal_ind_);

    // Evaluate the start state
    start_->parent_id = PARENT_TYPE(START);
    heuristic_->setStart(const_cast<StateType &>(start));
    // Evaluate the goal state
    goal_->parent_id = PARENT_TYPE(GOAL);
    goal_constraint_.type = SINGLE_SEARCH_STATE_GOAL;
    goal_constraint_.check_goal = &singleGoalConstraint;
    goal_constraint_.check_goal_user = &goal_->state_id;
    heuristic_->setGoalConstraint(goal_constraint_);

    // Evaluate the start state.
    start_->g = 0;
    start_->c = 0;
    start_->f = computeHeuristic(start_ind_);
    open_.push(start_);
    start_->setOpen();
}

auto ims::FocalSearch::getSearchState(int state_id) -> ims::FocalSearch::SearchState*{
    assert(state_id < states_.size() && state_id >= 0);
    return states_[state_id];
}

auto ims::FocalSearch::getOrCreateSearchState(int state_id) -> ims::FocalSearch::SearchState * {
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

double ims::FocalSearch::computeHeuristic(int state_id) {
    double dist;
    auto s = action_space_ptr_->getRobotState(state_id);
    if (!heuristic_->getHeuristic(s->state, dist))
        throw std::runtime_error("Heuristic function failed");
    else
        return dist;
}

double ims::FocalSearch::computeHeuristic(int s1_id, int s2_id) {
    double dist;
    auto s1 = action_space_ptr_->getRobotState(s1_id);
    auto s2 = action_space_ptr_->getRobotState(s2_id);
    if (!heuristic_->getHeuristic(s1->state, s2->state, dist))
        throw std::runtime_error("Heuristic function failed");
    else
        return dist;
}


bool ims::FocalSearch::plan(std::vector<StateType>& path) {
    startTimer();
    int iter {0};
    while (!open_.empty() && !isTimeOut()){
        // report progress every 1000 iterations
        if (iter % 100000 == 0){
            std::cout << "open size: " << open_.size() << std::endl;
        }
        
        // Get the state with the lowest f value from the open list and remove it.
        auto state  = open_.min();
        open_.pop();

        // Set it to closed.
        state->setClosed();

        // Check if this is a goal state.
        if (isGoalState(state->state_id)){
            goal_ = state->state_id;
            getTimeFromStart(stats_.time);
            reconstructPath(path, stats_.transition_costs);
            stats_.cost = state->g;
            stats_.path_length = (int)path.size();
            stats_.num_generated = (int)action_space_ptr_->states_.size();
            stats_.focal_suboptimality = params_.focal_suboptimality;
            return true;
        }

        // If it is not a goal state, then expand it.
        expand(state->state_id);
        ++iter;

        // Reorder the open list.
        double open_list_f_lower_bound = open_.getLowerBound();
        open_.updateWithBound(params_.focal_suboptimality * open_list_f_lower_bound);
    }

    getTimeFromStart(stats_.time);
    return false;
}

void ims::FocalSearch::expand(int state_id){

    auto state = getSearchState(state_id);
    
    std::vector<int> successors;
    // In this case we use the "cost" as the new f value and the "subcost" as the new c value.
    // In other algorithms, the cost is a g-cost and it is added to the previous g value, and the subcost is similarly added to the previous c value.
    std::vector<double> costs;  
    std::vector<double> subcosts;
    action_space_ptr_->getSuccessors(state->state_id, successors, costs, subcosts);
    
    for (size_t i {0} ; i < successors.size() ; ++i){
        int successor_id = successors[i];
        double cost = costs[i];
        double subcost = subcosts[i];
        auto successor = getOrCreateSearchState(successor_id);

        // If the successor is already closed, skip it.
        // Another variant of this algorithm would be to create a new search state for the successor and add it to the open list.

        if (successor->in_closed){
            continue;
        }
        if (successor->in_open){
            if (successor->f > cost){
                successor->parent_id = state->state_id;
                successor->f = cost;
                successor->c = subcost;
                open_.update(successor);
            }
        } else {
            setStateVals(successor->state_id, state->state_id, cost, subcost);
            open_.push(successor);
            successor->setOpen();
        }
    }
    stats_.num_expanded++;
}


void ims::FocalSearch::setStateVals(int state_id, int parent_id, double cost, double subcost) {
    auto state_ = getSearchState(state_id);
    state_->parent_id = parent_id;
    state_->c = subcost;
    state_->f = cost;
}


void ims::FocalSearch::reconstructPath(std::vector<StateType>& path) {
    SearchState* state = getSearchState(goal_);
    while (state->parent_id != -1){
        path.push_back(action_space_ptr_->getRobotState(state->state_id)->state);
        state = getSearchState(state->parent_id);
    }
    path.push_back(action_space_ptr_->getRobotState(state->state_id)->state);
    std::reverse(path.begin(), path.end());
}


void ims::FocalSearch::reconstructPath(std::vector<StateType>& path, std::vector<double>& costs) {
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


bool ims::FocalSearch::isGoalState(int s_id) {
    if (std::any_of(goals_.begin(), goals_.end(), [&s_id](int goal_ind) {return s_id == goal_ind;})){
        return true;
    }

    // Also ask the action space if this state id is a goal state. Sometimes, states need to be determined as goal in real time.
    // This is EXTREMELY bug prone. Commenting this out for now until we have a proper GoalCondition object.
    // return action_space_ptr_->isGoalState(s_id, goals_);
    return false;
}


void ims::FocalSearch::resetPlanningData() {
    for (auto state : states_){
        delete state;
    }
    states_ = std::vector<SearchState*>();
    open_.clear();

    goals_.clear();
    goal_ = -1;
    stats_ = FocalSearchPlannerStats();
}
