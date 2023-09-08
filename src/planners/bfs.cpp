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
 * \file   bfs.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   07/07/23
 */

#include <search/planners/bfs.hpp>

ims::BFS::BFS(const ims::BFSParams &params) : params_(params), BestFirstSearch(params) {}

ims::BFS::~BFS() {
    for (auto &state : states_) {
        delete state;
    }
}


void ims::BFS::initializePlanner(const std::shared_ptr<ActionSpace> &action_space_ptr,
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
        start_->f = 0;
        open_.push_back(start_);
        start_->setOpen();
    }
}

void ims::BFS::initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
                                   const StateType& start, const StateType& goal) {
    // Create an action space pointer.
    action_space_ptr_ = action_space_ptr;
    // Clear both.
    action_space_ptr_->resetPlanningData();
    resetPlanningData();

    // Check if the start state vector is valid.
    if (!action_space_ptr_->isStateValid(start)){
        throw std::runtime_error("Start state is not valid");
    }
    // Check if the goal state is valid
    if (!action_space_ptr_->isStateValid(goal)){
        throw std::runtime_error("Goal state is not valid");
    }

    // Create the start and goal states in the action space, generating state indices for both internally.
    int start_ind_ = action_space_ptr_->getOrCreateRobotState(start);
    auto start_ = getOrCreateSearchState(start_ind_);

    int goal_ind_ = action_space_ptr_->getOrCreateRobotState(goal);
    goals_.push_back(goal_ind_);

    // Set the start search state and add it to the open list.
    start_->parent_id = PARENT_TYPE(START);
    start_->setOpen();
    start_->g = 0;
    start_->f = 0;

    open_.push_back(start_);
}


auto ims::BFS::getSearchState(int state_id) -> ims::BFS::SearchState * {
    assert(state_id < states_.size() && state_id >= 0);
    return states_[state_id];
}

auto ims::BFS::getOrCreateSearchState(int state_id) -> ims::BFS::SearchState * {
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

bool ims::BFS::plan(std::vector<StateType>& path) {
    startTimer();
    int iter {0};
    while (!open_.empty() && !isTimeOut()){
        // report progress every 1000 iterations
        if (iter % 100000 == 0){
            std::cout << "open size: " << open_.size() << std::endl;
        }
        // Get the next node in the queue to expand.
        auto state  = open_.at(0);
        open_.pop_front();

        state->setClosed();
        if (isGoalState(state->state_id)){
            goal_ = state->state_id;
            getTimeFromStart(stats_.time);
            reconstructPath(path);
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

void ims::BFS::expand(int state_id){
    auto state = getSearchState(state_id);

    // Mark as closed.
    state->setClosed();

    // Ask for the successors from the action space.
    std::vector<int> successors;
    std::vector<double> costs;
    action_space_ptr_->getSuccessors(state->state_id, successors, costs);
    for (size_t i {0} ; i < successors.size() ; ++i){
        int successor_id = successors[i];
        
        auto successor = getOrCreateSearchState(successor_id);
        if (successor->in_closed){
            continue;
        }
        if (successor->in_open){
            continue;
        }

        successor->parent_id = state->state_id;
        successor->in_closed = false;
        successor->setOpen();
        successor->g = state->g + costs[i];
        successor->f = successor->g;
        open_.push_back(successor);
    }
    stats_.num_expanded++;
}

void ims::BFS::setStateVals(int state_id, int parent_id, double cost)
{
    auto state = getSearchState(state_id);
    state->parent_id = parent_id;
}

void ims::BFS::reconstructPath(std::vector<StateType>& path) {
    SearchState* state = getSearchState(goal_);
    while (state->parent_id != -1){
        path.push_back(action_space_ptr_->getRobotState(state->state_id)->state);
        state = getSearchState(state->parent_id);
    }
    
    path.push_back(action_space_ptr_->getRobotState(state->state_id)->state);
    std::reverse(path.begin(), path.end());
}

void ims::BFS::reconstructPath(std::vector<StateType>& path, std::vector<double>& costs) {
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