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
 * \file   focal_wastar.cpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   Oct 05 2023
*/

#include <search/planners/focal_search/focal_wastar.hpp>
ims::FocalwAStar::FocalwAStar(const ims::FocalwAStarParams &params) : params_(params), FocalSearch(params) {

}

ims::FocalwAStar::~FocalwAStar() {}

void ims::FocalwAStar::initializePlanner(const std::shared_ptr<SubcostActionSpace> &action_space_ptr,
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

    // Check if the goal is valid.
    if (!action_space_ptr_->isStateValid(goals[0])){
        throw std::runtime_error("Goal state is not valid");
    }
    int goal_ind_ = action_space_ptr_->getOrCreateRobotState(goals[0]);
    auto goal_ = getOrCreateSearchState(goal_ind_);
    goals_.push_back(goal_ind_);

    // Evaluate the goal state with the heuristic.
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
        start_->c = 0;
        start_->h = computeHeuristic(start_ind_);
        start_->f = start_->g + params_.epsilon*start_->h;
        open_.push(start_);
        start_->setOpen();
    }

    // Update stats suboptimality.
    this->stats_.suboptimality = params_.epsilon;
    this->stats_.focal_suboptimality = params_.focal_suboptimality;
}

void ims::FocalwAStar::initializePlanner(const std::shared_ptr<SubcostActionSpace>& action_space_ptr,
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
    start_->c = 0;
    start_->h = computeHeuristic(start_ind_);
    start_->f = start_->g + params_.epsilon*start_->h;
    start_->setOpen();

    open_.push(start_);

    // Update stats suboptimality.
    this->stats_.suboptimality = params_.epsilon;
    this->stats_.focal_suboptimality = params_.focal_suboptimality;
}

bool ims::FocalwAStar::plan(std::vector<StateType>& path) {
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

            return true;
        }

        // Expand the state.
        expand(state->state_id);
        ++iter;
        
        // Reorder the open list.
        double open_list_f_lower_bound = open_.getLowerBound();
        open_.updateWithBound(params_.focal_suboptimality * open_list_f_lower_bound);
    }
    getTimeFromStart(stats_.time);
    return false;
}

void ims::FocalwAStar::expand(int state_id){

    SearchState* state = getSearchState(state_id);
    std::vector<int> successors;
    std::vector<double> costs;
    std::vector<double> subcosts;

    action_space_ptr_->getSuccessors(state->state_id, successors, costs, subcosts);
    for (size_t i {0} ; i < successors.size() ; ++i){
        int successor_id = successors[i];
        double cost = costs[i];
        double subcost = subcosts[i];
        SearchState* successor = getOrCreateSearchState(successor_id);

        // If allowing to create duplicate states, then we need to check if the state already exists.
        if (!successor->in_closed && !successor->in_open){
            setStateVals(successor->state_id, state->state_id, cost, subcost);
            open_.push(successor);
            successor->setOpen();
        }

        // If the state is not new, then we need to check if the state already exists.
        else{
            // If the state is not a new state, then we have a few options options. 
            // If duplicates not allowed and the state is in the closed list, then we do nothing.
            if (successor->in_closed){
                continue;
            }

            // If duplicates not allowed and the state is in the open list, then we check if the new g,c values is lower than the current g,c values. If so, we update the state's parent and g value.
            else if (successor->in_open){
                if (successor->g > state->g + cost && successor->c > state->c + subcost){
                    successor->parent_id = state->state_id;
                    successor->g = state->g + cost;
                    successor->c = state->c + subcost;
                    successor->f = successor->g + params_.epsilon * successor->h;
                    open_.update(successor);
                }
            }

            else{
                throw std::runtime_error("Found successor node that is neither in the open list nor in the closed list and is not new.");
            }
        }
    }
    stats_.num_expanded++;
}

// TODO(yoraish): this is needed, put it back in the header file.
// void ims::FocalwAStar::reconstructPath(std::vector<StateType>& path, std::vector<double>& costs) {
//     path.clear();
//     costs.clear();

//     costs.push_back(0); // The goal state gets a transition cost of 0.
//     SearchState* state_ = getSearchState(goal_);
//     while (state_->parent_id != -1){
//         path.push_back(action_space_ptr_->getRobotState(state_->state_id)->state);
        
//         // Get the transition cost. This is the difference between the g values of the current state and its parent.
//         double transition_cost = state_->g - getSearchState(state_->parent_id)->g;
//         costs.push_back(transition_cost);

//         state_ = getSearchState(state_->parent_id);
//     }
//     path.push_back(action_space_ptr_->getRobotState(state_->state_id)->state);

//     std::reverse(path.begin(), path.end());
//     std::reverse(costs.begin(), costs.end());   
// }

// void ims::FocalwAStar::reconstructPath(std::vector<StateType>& path) {
//     SearchState* state_ = getSearchState(goal_);
//     while (state_->parent_id != -1){
//         path.push_back(action_space_ptr_->getRobotState(state_->state_id)->state);
//         state_ = getSearchState(state_->parent_id);
//     }
//     path.push_back(action_space_ptr_->getRobotState(state_->state_id)->state);
//     std::reverse(path.begin(), path.end());
// }

// void ims::FocalwAStar::resetPlanningData(){
//     for (auto state_ : states_){
//         delete state_;
//     }
//     states_.clear();
//     open_.clear();
//     goals_.clear();
//     goal_ = -1;
//     stats_ = PlannerStats();
// }

void ims::FocalwAStar::setStateVals(int state_id, int parent_id, double cost, double subcost)
{
    auto state_ = getSearchState(state_id);
    auto parent = getSearchState(parent_id);
    state_->parent_id = parent_id;
    state_->g = parent->g + cost;
    state_->c = parent->c + subcost;
    state_->h = computeHeuristic(state_id);
    state_->f = state_->g + params_.epsilon*state_->h;
}
