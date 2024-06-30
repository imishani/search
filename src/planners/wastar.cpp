/*
 * Copyright (C) 2023, Itamar Mishani
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
 * \file   wastar.cpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   4/4/23
*/

#include <search/planners/wastar.hpp>

ims::wAStar::wAStar(const ims::wAStarParams &params) : params_(params), BestFirstSearch(params) {}

ims::wAStar::~wAStar() {
    for (auto &state : states_) {
        delete state;
    }
}

void ims::wAStar::initializePlanner(const std::shared_ptr<ActionSpace> &action_space_ptr,
                                    const std::vector<StateType> &starts,
                                    const std::vector<StateType> &goals) {
    // space pointer
    action_space_ptr_ = action_space_ptr;
    // Clear both.
    action_space_ptr_->resetPlanningData();
    resetPlanningData();

    if (goals.empty() || starts.empty()) {
        throw std::runtime_error("Starts or goals are empty");
    }

    if (goals.size() > 1) {
        throw std::runtime_error("Currently, only one goal is supported");
    }
    // check if goal is valid
    if (!action_space_ptr_->isStateValid(goals[0])){
        throw std::runtime_error("Goal state is not valid");
    }
    int goal_ind_ = action_space_ptr_->getOrCreateRobotState(goals[0]);
    auto goal_ = getOrCreateSearchState(goal_ind_);
    goals_.push_back(goal_ind_);

    // Evaluate the goal state
    goal_->parent_id = PARENT_TYPE(GOAL);
    heuristic_->setGoal(const_cast<StateType &>(goals[0]));
    goal_->h = 0;

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
        start_->h = computeHeuristic(start_ind_);
        start_->f = start_->g + params_.epsilon*start_->h;
        open_.push(start_);
        start_->setOpen();
    }
    stats_.suboptimality = params_.epsilon;
}

void ims::wAStar::initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
                                   const StateType& start, const StateType& goal) {
    // Space pointer.
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
    int start_ind_ = action_space_ptr_->getOrCreateRobotState(start);
    auto start_ = getOrCreateSearchState(start_ind_);

    int goal_ind_ = action_space_ptr_->getOrCreateRobotState(goal);
    auto goal_ = getOrCreateSearchState(goal_ind_);
    goals_.push_back(goal_ind_);

    start_->parent_id = PARENT_TYPE(START);
    heuristic_->setStart(const_cast<StateType &>(start));
    // Evaluate the goal state
    goal_->parent_id = PARENT_TYPE(GOAL);
    heuristic_->setGoal(const_cast<StateType &>(goal));
    goal_->h = 0;
    // Evaluate the start state
    start_->g = 0;
    start_->h = computeHeuristic(start_ind_);
    start_->f = start_->g + params_.epsilon*start_->h;
    start_->setOpen();

    open_.push(start_);
    // update stats suboptimality
    stats_.suboptimality = params_.epsilon;
}

auto ims::wAStar::getSearchState(int state_id) -> ims::wAStar::SearchState * {
    assert(state_id < states_.size() && state_id >= 0);
    return states_[state_id];
}

auto ims::wAStar::getOrCreateSearchState(int state_id) -> ims::wAStar::SearchState * {
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

bool ims::wAStar::plan(std::vector<StateType>& path) {
    startTimer();
    int iter {0};
    while (!open_.empty() && !isTimeOut()){
        // report progress every 1000 iterations
        if (iter % 100000 == 0 && params_.verbose){
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

void ims::wAStar::expand(int state_id){

    auto state_ = getSearchState(state_id);
    std::vector<std::vector<int>> successor_edges_state_ids;
    std::vector<std::vector<double>> successor_edges_transition_costs;
    action_space_ptr_->getSuccessorEdges(state_->state_id, successor_edges_state_ids, successor_edges_transition_costs);
    for (size_t i {0} ; i < successor_edges_state_ids.size() ; ++i){
        const std::vector<int> & successor_edge_state_ids = successor_edges_state_ids[i];
        int successor_id = successor_edge_state_ids.back();
        double successor_edge_total_cost = std::accumulate(successor_edges_transition_costs[i].begin(), successor_edges_transition_costs[i].end(), 0.0);
        auto successor = getOrCreateSearchState(successor_id);
        if (successor->in_closed){
            continue;
        }
        if (isGoalState(successor_id) && params_.verbose ){
            std::cout << "Added Goal to open list" << std::endl;
        }
        if (successor->in_open){
            if (successor->g > state_->g + successor_edge_total_cost){
                setStateVals(successor->state_id,
                             state_->state_id,
                             successor_edge_total_cost,
                             successor_edge_state_ids,
                             successor_edges_transition_costs[i]);
                open_.update(successor);
            }
        } else {
            setStateVals(successor->state_id,
                         state_->state_id,
                         successor_edge_total_cost,
                         successor_edge_state_ids,
                         successor_edges_transition_costs[i]);
            open_.push(successor);
            successor->setOpen();
        }
    }
    stats_.num_expanded++;
}

void ims::wAStar::setStateVals(int state_id, int parent_id, double transition_cost)
{
    auto state_ = getSearchState(state_id);
    auto parent = getSearchState(parent_id);
    state_->parent_id = parent_id;
    state_->g = parent->g + transition_cost;
    state_->h = computeHeuristic(state_id);
    state_->f = state_->g + params_.epsilon*state_->h;
}

void ims::wAStar::setStateVals(int state_id,
                               int parent_id,
                               double transition_cost,
                               const std::vector<int> & edge_from_parent_state_ids,
                               const std::vector<double> & edge_from_parent_transition_costs)
{
    setStateVals(state_id, parent_id, transition_cost);
    auto state_ = getSearchState(state_id);
    state_->edge_from_parent_state_ids = edge_from_parent_state_ids;
    state_->edge_from_parent_transition_costs = edge_from_parent_transition_costs;
}

void ims::wAStar::reconstructPath(std::vector<StateType>& path, std::vector<double>& costs) {
    path.clear();
    costs.clear();

    costs.push_back(0); // The goal state gets a transition cost of 0.
    path.push_back(action_space_ptr_->getRobotState(goal_)->state);
    SearchState* state_ = getSearchState(goal_);
        while (state_->parent_id != -1){
            std::cout << "---" << std::endl;

        // If this is an edge with intermediate states, add the intermediate states and the costs to get to them via the edge.
        int edge_from_parent_num_states = (int)state_->edge_from_parent_state_ids.size();
        if (edge_from_parent_num_states > 2){
            for (int i {edge_from_parent_num_states - 2}; i >= 0; --i){
                int state_id = state_->edge_from_parent_state_ids[i];
                double transition_cost = state_->edge_from_parent_transition_costs[i]; // Removing one to get the cost from the "parent-intermediate" to the current intermediate.
                path.push_back(action_space_ptr_->getRobotState(state_id)->state);
                std::cout << "Add path state: " << action_space_ptr_->getRobotState(state_id)->state << std::endl;
                costs.push_back(transition_cost);
            }
            // Assert that this transition cost is the same as the one from the parent to the current state.
            double edge_from_parent_total_cost = vectorSum(state_->edge_from_parent_transition_costs);
            double rounded_edge_from_parent_total_cost = std::round(edge_from_parent_total_cost * 1000) / 1000;
            double rounded_g_difference = std::round((state_->g - getSearchState(state_->parent_id)->g) * 1000) / 1000;
            if (rounded_edge_from_parent_total_cost != rounded_g_difference){
                std::cout << RED << "Edge from parent total cost: " << edge_from_parent_total_cost << RESET << std::endl;
                std::cout << RED << "State g: " << state_->g << RESET << std::endl;
                std::cout << RED << "Parent g: " << getSearchState(state_->parent_id)->g << RESET << std::endl;
                std::cout << RED << "Expected: " << state_->g - getSearchState(state_->parent_id)->g << " and got " << edge_from_parent_total_cost << RESET << std::endl;
            }

            state_ = getSearchState(state_->parent_id);
        }
        // If this is an edge without intermediate states, add the parent and the cost to get to it.
        else{
            path.push_back(action_space_ptr_->getRobotState(state_->parent_id)->state);
            // Get the transition cost. This is the difference between the g values of the current state and its parent.
            double transition_cost = state_->g - getSearchState(state_->parent_id)->g;
            costs.push_back(transition_cost);
            state_ = getSearchState(state_->parent_id);
        }
    }

    std::reverse(path.begin(), path.end());
    std::reverse(costs.begin(), costs.end());

    assert(path.size() == costs.size());
}

void ims::wAStar::reconstructPath(std::vector<StateType>& path) {
    std::vector<double> costs;
    reconstructPath(path, costs);
}

void ims::wAStar::resetPlanningData(){
    for (auto state_ : states_){
        delete state_;
    }
    states_.clear();
    open_.clear();
    goals_.clear();
    goal_ = -1;
    stats_ = PlannerStats();
}

auto ims::wAStar::getAllSearchStates() -> std::vector<ims::wAStar::SearchState*> {
    return states_;
}