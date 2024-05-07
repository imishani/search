/*
 * Copyright (C) 2024, Itamar Mishani
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
 * \file   lpastar.cpp
 * \author Carina Sanborn (czsanbor@andrew.cmu.edu)
 * \date   2/9/24
*/

#include <search/planners/lpastar.hpp>
#include <utility>

ims::LPAStar::LPAStar(const ims::LPAStarParams &params) : params_(params), BestFirstSearch(params) {}

ims::LPAStar::~LPAStar() {
    for (ims::LPAStar::SearchState *&state : states_) {
        delete state;
    }
}

double ims::LPAStar::getEdgeCost(StateType &u, StateType &v) {
        if (action_space_ptr_->isStateValid(v)){
            if (abs(u[0] - v[0]) > 1 || abs(u[1] - v[1]) > 1) {
                // states are not next to each other
                return 100;
            } else if (u[0] == v[0] && u[1] == v[1]) {
                // the states are the same
                return 0;
            } else if (u[0] == v[0] || u[1] == v[1]) {
                // N, E, W, S
                return 1;
            } else {
                // NE, SW, NW, SE
                return 1.414;
            }
        } else {
            // state u or v is an obstacle
            return 100;
        }
    }

std::pair<double, double> ims::LPAStar::calculateKeys(ims::LPAStar::SearchState *s) {
    std::pair<double, double> k;
    s->h = computeHeuristic(s->state_id);
    // -1 represents infinity in our code
    if (s->g == -1 && s->rhs == -1) {
        k.first = -1;
        k.second = -1;
    } else {
        if (s->g == -1) {
            k.first = s->rhs + s->h;
            k.second = s->rhs;
        } else if (s->rhs == -1) {
            k.first = s->g + s->h;
            k.second = s->g;
        } else {
            k.first = std::min(s->g, s->rhs) + s->h;
            k.second = std::min(s->g, s->rhs);
        }
    }
    s->key = k;
    return k;
}

void ims::LPAStar::updateVertex(ims::LPAStar::SearchState *s) {
    double min_rhs = -1; // infinity
    int parent_id = -1;
    if (s->g == 0)
        return;
    // loop through predecessors u and find minimum g(pred) + cost(pred, s)
    for (int pred : s->predecessors) {
        ims::LPAStar::SearchState *pred_state = getSearchState(pred);
        StateType &u = (action_space_ptr_->getRobotState(pred_state->state_id))->state;
        StateType &v = (action_space_ptr_->getRobotState(s->state_id))->state;
        // get cost from action space
        double cost = getEdgeCost(u, v);
        if (pred_state->g != -1 && (pred_state->g + cost < min_rhs || min_rhs == -1)) {
            min_rhs = pred_state->g + cost;
            parent_id = pred_state->state_id;
        }
    }
    s->rhs = min_rhs;
    s->parent_id = parent_id;
    // remove from U
    if (U_.contains(s)){
        U_.erase(s);
    }
    // Add back into U
    if (s->g != s->rhs){
        calculateKeys(s);
        U_.push(s);
    }
}

void ims::LPAStar::initializePlanner(const std::shared_ptr<ActionSpace> &action_space_ptr,
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
    ims::LPAStar::SearchState *goal_ = getOrCreateSearchState(goal_ind_);
    goals_.push_back(goal_ind_);

    // Evaluate the goal state
    goal_->parent_id = PARENT_TYPE(GOAL);
    heuristic_->setGoal(const_cast<StateType &>(goals[0]));
    goal_->h = 0;

    for (const StateType &start : starts) {
        // check if start is valid
        if (!action_space_ptr_->isStateValid(start)){
            throw std::runtime_error("Start state is not valid");
        }
        // Evaluate the start state
        int start_ind = action_space_ptr_->getOrCreateRobotState(start);
        ims::LPAStar::SearchState *start_state = getOrCreateSearchState(start_ind);
        start_state->parent_id = PARENT_TYPE(START);
        heuristic_->setStart(const_cast<StateType &>(start));
        start_state->h = computeHeuristic(start_ind);
        start_state->rhs = 0;
        calculateKeys(start_state);
        U_.push(start_state);
    }
}

void ims::LPAStar::initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
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
    int start_ind = action_space_ptr_->getOrCreateRobotState(start);
    ims::LPAStar::SearchState *start_ss = getOrCreateSearchState(start_ind);

    int goal_ind_ = action_space_ptr_->getOrCreateRobotState(goal);
    ims::LPAStar::SearchState *goal_ss = getOrCreateSearchState(goal_ind_);
    goal_ = goal_ind_;

    start_ss->parent_id = PARENT_TYPE(START);
    heuristic_->setStart(const_cast<StateType &>(start));
    // Evaluate the goal state
    goal_ss->parent_id = PARENT_TYPE(GOAL);
    heuristic_->setGoal(const_cast<StateType &>(goal));
    goal_ss->h = 0;
    // Evaluate the start state
    start_ss->rhs = 0;
    calculateKeys(start_ss);
    U_.push(start_ss);
    start_ss->h = computeHeuristic(start_ind);
}

ims::LPAStar::SearchState *ims::LPAStar::getSearchState(int state_id) {
    assert(state_id < states_.size() && state_id >= 0);
    return states_[state_id];
}

ims::LPAStar::SearchState *ims::LPAStar::getOrCreateSearchState(int state_id){
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

void ims::LPAStar::updateVertices(std::vector<std::vector<size_t>> updated_indices, 
                                    std::vector<std::vector<int>> curr_map) {
    for (const std::vector<size_t> map_loc : updated_indices) {
        double x = map_loc[0];
        double y = map_loc[1];
        // if updated index is now an obstacle
        if (curr_map[y][x] == 100) {
            // need to call updateVertex on the map loc
            StateType xy = {x, y};
            StateType &s = xy;
            bool stateExists = action_space_ptr_->checkIfContainsRobotState(s);
            if (stateExists) {
                int state_id = action_space_ptr_->getOrCreateRobotState(s);
                ims::LPAStar::SearchState *search_state = getOrCreateSearchState(state_id);
                updateVertex(search_state);
            }
        }
    }
}

bool ims::LPAStar::plan(std::vector<StateType>& path) {
    startTimer();
    int iter {0};
    ims::LPAStar::SearchState *goal = getSearchState(goal_);
    SearchStateCompare tmpComparator;
    while (!isTimeOut()) {
        // No better path to the goal will be found, so we can break loop.
        if (tmpComparator(*goal, *U_.min())) {
            break;
        }
        // report progress every 1000 iterations
        if (iter % 100000 == 0 && params_.verbose) {
            std::cout << "Iter: " << iter << " U size: " << U_.size() << std::endl;
        }
        ims::LPAStar::SearchState *s = U_.min();
        U_.pop();
        if (s->g > s->rhs || s->g == -1 && s->rhs != -1) {
            s->g = s->rhs;
            std::vector<int> successors;
            std::vector<double> costs;
            action_space_ptr_->getSuccessors(s->state_id, successors, costs);
            for (int i = 0; i < successors.size(); i++) {
                int successor_id = successors[i];
                int cost = costs[i];
                ims::LPAStar::SearchState *successor = getOrCreateSearchState(successor_id);
                // insert predecessor into predecessors set
                successor->predecessors.insert(s->state_id);
                updateVertex(successor);
            }
        } 
        ++iter;
    }
    if (!isTimeOut()) {
        getTimeFromStart(stats_.time);
        reconstructPath(path, stats_.transition_costs);
        stats_.cost = goal->g;
        stats_.path_length = (int)path.size();
        stats_.num_generated = (int)action_space_ptr_->states_.size();
        return true;
    } 
    getTimeFromStart(stats_.time);
    return false;
}

void ims::LPAStar::setStateVals(int state_id, int parent_id, double cost)
{
    ims::LPAStar::SearchState *state_ = getSearchState(state_id);
    ims::LPAStar::SearchState *parent = getSearchState(parent_id);
    state_->parent_id = parent_id;
    state_->g = parent->g + cost;
    state_->h = computeHeuristic(state_id);
    state_->f = state_->g + state_->h;
}

void ims::LPAStar::reconstructPath(std::vector<StateType>& path, std::vector<double>& costs) {
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

void ims::LPAStar::reconstructPath(std::vector<StateType>& path) {
    SearchState* state_ = getSearchState(goal_);
    while (state_->parent_id != -1){
        path.push_back(action_space_ptr_->getRobotState(state_->state_id)->state);
        state_ = getSearchState(state_->parent_id);
    }
    path.push_back(action_space_ptr_->getRobotState(state_->state_id)->state);
    std::reverse(path.begin(), path.end());
}

void ims::LPAStar::resetPlanningData(){
    for (ims::LPAStar::SearchState *state_ : states_){
        delete state_;
    }
    states_.clear();
    U_.clear();
    goals_.clear();
    goal_ = -1;
    stats_ = PlannerStats();
}

std::vector<ims::LPAStar::SearchState*> ims::LPAStar::getAllSearchStates() {
    return states_;
}