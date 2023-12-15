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
 * \file   arastar.cpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   7/21/23
*/

#include "search/planners/arastar.hpp"


ims::ARAStar::ARAStar(const ims::ARAStarParams &params) : wAStar(params), params_(params) {
}

ims::ARAStar::~ARAStar() {
    for (SearchState *state : states_) {
        delete state;
    }
}


void ims::ARAStar::initializePlanner(const std::shared_ptr<ActionSpace> &action_space_ptr,
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

void ims::ARAStar::initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
                                    const StateType& start, const StateType& goal) {
    // space pointer
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
    printf("start ind: %d \n", start_ind_);
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

auto ims::ARAStar::getSearchState(int state_id) -> ims::ARAStar::SearchState * {
    assert(state_id < states_.size() && state_id >= 0);
    return states_[state_id];
}

auto ims::ARAStar::getOrCreateSearchState(int state_id) -> ims::ARAStar::SearchState * {
    if (state_id >= states_.size()){
        states_.resize(state_id + 1, nullptr);
    }
    if (states_[state_id] == nullptr){
        assert(state_id < states_.size() && state_id >= 0);
        states_[state_id] = new SearchState;
        states_[state_id]->state_id = state_id;
        states_[state_id]->call_number = params_.call_number;
    }
    return states_[state_id];
}

bool ims::ARAStar::plan(std::vector<StateType> &path) {
    startTimer();
    params_.call_number = 0;
    // outer loop of ARA*
    while (params_.epsilon >= params_.final_epsilon){
        std::cout << MAGENTA << "Replanning with epsilon: " << params_.epsilon << RESET << std::endl;
        if (params_.call_number == 0){
            if (!improvePath(path)){
                return false;
            } else {
                params_.call_number++;
                updateBounds();
                if (stats_.suboptimality == params_.final_epsilon)
                    break;
            }
            continue;
        }
        // move states from incons to open
        for (SearchState *state : incons_){
            state->in_incons = false;
            open_.push(state);
            state->in_open = true;
        }
        reorderOpen();
        incons_.clear();
        // inner loop of ARA*
        bool success = improvePath(path);
        if (!success){ // could not improve the solution
            return true; // return true because we have a solution from previous iteration
        }
        params_.call_number++;
        updateBounds();
        if (stats_.suboptimality == params_.final_epsilon)
            break;

    }
    if (!path.empty()) {
        getTimeFromStart(stats_.time);
        std::cout << GREEN << "Found the optimal solution with cost: " << stats_.cost << RESET << std::endl;
        return true;
    } else
        return false;
}

bool ims::ARAStar::improvePath(std::vector<StateType> &path) {
    while (!open_.empty()){
        auto state  = open_.min();
        if (state->f > stats_.cost){
            return true;
        }
        open_.pop();
        state->setClosed(); state->v = state->g; //state->call_number = params_.call_number;
        if (isGoalState(state->state_id)){
            goal_ = state->state_id;
            getTimeFromStart(stats_.time);
            reconstructPath(path, stats_.transition_costs);
            stats_.cost = state->g;
            stats_.path_length = (int)path.size();
            stats_.num_generated = (int)action_space_ptr_->states_.size();
            return true;
        }
        if (timedOut()){
            std::cout << RED << "Timeout in improve path" << RESET << std::endl;
            return false;
        }
        expand(state->state_id);
    }
    getTimeFromStart(stats_.time);
    std::cout << RED << "Open list is empty" << RESET << std::endl;
    return false;
}

void ims::ARAStar::expand(int state_id) {
    auto state_ = getSearchState(state_id);
    std::vector<int> successors;
    std::vector<double> costs;
    action_space_ptr_->getSuccessors(state_->state_id, successors, costs);
    for (size_t i {0} ; i < successors.size() ; ++i){
        int successor_id = successors[i];
        double cost = costs[i];
        auto successor = getOrCreateSearchState(successor_id);
        reinitSearchState(successor);

        if (successor->g > state_->g + cost) {
            successor->g = state_->g + cost;
            successor->parent_id = state_->state_id;
            if (!successor->in_closed){
                successor->h = computeHeuristic(successor_id);
                successor->f = successor->g + params_.epsilon*successor->h;
                if (successor->in_open){
                    open_.update(successor);
                } else {
                    setStateVals(successor->state_id, state_->state_id, cost);
                    open_.push(successor);
                    successor->setOpen();
                }
            }
            else if (!successor->in_incons) {
                successor->in_incons = true;
                incons_.push_back(successor);
            }
        }
    }
    stats_.num_expanded++;
}


void ims::ARAStar::setStateVals(int state_id, int parent_id, double cost)
{
    auto state_ = getSearchState(state_id);
    auto parent = getSearchState(parent_id);
    state_->parent_id = parent_id;
    state_->g = parent->g + cost;
    state_->h = computeHeuristic(state_id);
    state_->f = state_->g + params_.epsilon*state_->h;
}

void ims::ARAStar::reconstructPath(std::vector<StateType>& path) {
    path.clear();
    SearchState* state_ = getSearchState(goal_);
    while (state_->parent_id != -1){
        path.push_back(action_space_ptr_->getRobotState(state_->state_id)->state);
        state_ = getSearchState(state_->parent_id);
    }
    path.push_back(action_space_ptr_->getRobotState(state_->state_id)->state);
    std::reverse(path.begin(), path.end());
}

void ims::ARAStar::reorderOpen() {
    for (auto state : open_){
        state->f = state->g + params_.epsilon * state->h;
    }
    open_.make(); // reorder intrusive heap
}


void ims::ARAStar::reconstructPath(std::vector<StateType>& path, std::vector<double>& costs) {
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

void ims::ARAStar::updateBounds() {
    // update stats to current suboptimality
    stats_.suboptimality = params_.epsilon;
    // update epsilon
    params_.epsilon -= params_.epsilon_delta;
    if (params_.epsilon < params_.final_epsilon)
        params_.epsilon = params_.final_epsilon;
}

void ims::ARAStar::reinitSearchState(ims::ARAStar::SearchState *state) const {
    if (state->call_number != params_.call_number){
        state->in_open = false;
        state->in_closed = false;
        state->in_incons = false;
        state->call_number = params_.call_number;
    }
}

bool ims::ARAStar::timedOut() {
    switch (params_.type) {
        case ARAStarParams::TIME:
            if (params_.ara_time_limit == INF_DOUBLE){
                return false;
            }
            // get the time elapsed
            double time_elapsed;
            getTimeFromStart(time_elapsed);
            return time_elapsed > params_.ara_time_limit;
        case ARAStarParams::EXPANSIONS:
            if (params_.expansions_limit == INF_DOUBLE){
                return false;
            } else
                return stats_.num_expanded >= params_.expansions_limit;
        case ARAStarParams::USER:
            return params_.timed_out_fun();
        default:
            // throw error Unknown type
            std::cout << RED << "Timed out: Unknown type of ARA* timeout" << RESET << std::endl;
            return true;
    }
}
void ims::ARAStar::resetPlanningData() {
    wAStar::resetPlanningData();
    open_.clear();
    incons_.clear();
    for (auto state : states_){
        delete state;
    }
    states_.clear();
    params_.call_number = 0;
    params_.curr_cost = INF_DOUBLE;
    params_.epsilon = params_.init_epsilon;
}

