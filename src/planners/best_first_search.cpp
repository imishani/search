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
 * \file   astar.hpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   3/29/23
*/

#include <search/planners/best_first_search.hpp>


ims::BestFirstSearch::BestFirstSearch(const BestFirstSearchParams &params) : Planner(params) {
    heuristic_ = params.heuristic_;
}

ims::BestFirstSearch::~BestFirstSearch() {
    for (auto &state : states_) {
        delete state;
    }
}

void ims::BestFirstSearch::initializePlanner(const std::shared_ptr<ActionSpace> &action_space_ptr,
                                             const std::vector<StateType> &starts,
                                             const std::shared_ptr<GoalCondition>& goal_condition) {
    // space pointer
    action_space_ptr_ = action_space_ptr;
    // Clear both.
    action_space_ptr_->resetPlanningData();
    resetPlanningData();

    if (starts.empty()) {
        throw std::runtime_error("Start are empty");
    }

    // if (goals.size() > 1) {
    //     throw std::runtime_error("Currently, only one goal is supported");
    // }

    // int goal_ind_ = action_space_ptr_->getOrCreateRobotState(goals[0]);
    // auto goal_ = getOrCreateSearchState(goal_ind_);
    // goals_.push_back(goal_ind_);

    // // Evaluate the goal state
    // goal_->parent_id = PARENT_TYPE(GOAL);
    heuristic_->setGoal(const_cast<StateType &>(goals[0]));

    for (auto &start : starts) {
        // Evaluate the start state
        int start_ind_ = action_space_ptr_->getOrCreateRobotState(start);
        auto start_ = getOrCreateSearchState(start_ind_);
        start_->parent_id = PARENT_TYPE(START);
        heuristic_->setStart(const_cast<StateType &>(start));
        start_->g = 0;
        start_->f = computeHeuristic(start_ind_);
        open_.push(start_);
        start_->setOpen();
    }
}

void ims::BestFirstSearch::initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
                                             const StateType& start, const StateType& goal) {
    int goal_ind = action_space_ptr_->getOrCreateRobotState(goal);
    getOrCreateSearchState(goal_ind); // To make sure indices line up? TODO: Is this necessary? I think so
    std::shared_ptr<GoalConditionExplicitIDs> goal_condition = std::make_shared<GoalConditionExplicitIDs>(goal_ind);
    initializePlanner(action_space_ptr, std::vector<StateType>{start}, goal_condition);
}

auto ims::BestFirstSearch::getSearchState(int state_id) -> ims::BestFirstSearch::SearchState*{
    assert(state_id < states_.size() && state_id >= 0);
    return states_[state_id];
}

auto ims::BestFirstSearch::getOrCreateSearchState(int state_id) -> ims::BestFirstSearch::SearchState * {
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


double ims::BestFirstSearch::computeHeuristic(int state_id) {
    double dist;
    auto s = action_space_ptr_->getRobotState(state_id);
    if (!heuristic_->getHeuristic(s->state, dist))
        throw std::runtime_error("Heuristic function failed");
    else
        return dist;
}

double ims::BestFirstSearch::computeHeuristic(int s1_id, int s2_id) {
    double dist;
    auto s1 = action_space_ptr_->getRobotState(s1_id);
    auto s2 = action_space_ptr_->getRobotState(s2_id);
    if (!heuristic_->getHeuristic(s1->state, s2->state, dist))
        throw std::runtime_error("Heuristic function failed");
    else
        return dist;
}


bool ims::BestFirstSearch::plan(std::vector<StateType>& path) {
    startTimer();
    int iter {0};
    while (!open_.empty() && !isTimeOut()){
        // report progress every 1000 iterations
        if (iter % 100000 == 0 && params_.verbose){
            std::cout << "open size: " << open_.size() << std::endl;
        }
        auto state  = open_.min();
        open_.pop();
        state->setClosed();
        if (m_check_goal_condition->isGoalState(state->state_id)){
            getTimeFromStart(stats_.time);
            reconstructPath(state, path);
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

void ims::BestFirstSearch::expand(int state_id){
    auto state = getSearchState(state_id);
    std::vector<int> successors;
    std::vector<double> costs; // In this case we use the "cost" as the new f value
    action_space_ptr_->getSuccessors(state->state_id, successors, costs);
    for (size_t i {0} ; i < successors.size() ; ++i){
        int successor_id = successors[i];
        double cost = costs[i];
        auto successor = getOrCreateSearchState(successor_id);
        if (successor->in_closed){
            continue;
        }
        if (successor->in_open){
            if (successor->f > cost){
                successor->parent_id = state->state_id;
                successor->f = cost;
                open_.update(successor);
            }
        } else {
            setStateVals(successor->state_id, state->state_id, cost);
            open_.push(successor);
            successor->setOpen();
        }
    }
    stats_.num_expanded++;
}


void ims::BestFirstSearch::setStateVals(int state_id, int parent_id, double cost) {
    auto state_ = getSearchState(state_id);
    state_->parent_id = parent_id;
    state_->f = cost;
}


void ims::BestFirstSearch::reconstructPath(const SearchState* state, std::vector<StateType>& path) {
    while (state->parent_id != -1){
        path.push_back(action_space_ptr_->getRobotState(state->state_id)->state);
        state = getSearchState(state->parent_id);
    }
    path.push_back(action_space_ptr_->getRobotState(state->state_id)->state);
    std::reverse(path.begin(), path.end());
}


void ims::BestFirstSearch::resetPlanningData() {
    for (auto state : states_){
        delete state;
    }
    states_.clear();
    open_.clear();
    stats_ = PlannerStats();
}
