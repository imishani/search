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

ims::FocalwAStar::~FocalwAStar() = default;

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
        open_->push(start_);
        start_->setOpen();
    }

    // Update stats sub-optimality.
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

    open_->push(start_);

    // Update stats suboptimality.
    this->stats_.suboptimality = params_.epsilon;
    this->stats_.focal_suboptimality = params_.focal_suboptimality;
}

bool ims::FocalwAStar::plan(std::vector<PathType>& seqs_path, std::vector<std::vector<double>>& seqs_transition_costs) {
    startTimer();
    int iter {0};

    // Reorder the open list.
    double open_list_f_lower_bound = open_->getLowerBound();
    open_->updateWithBound(params_.focal_suboptimality * open_list_f_lower_bound);

    while (!open_->empty() && !isTimeOut()){
        // report progress every 1000 iterations
        if (iter % 100000 == 0 && params_.verbose){
            std::cout << "Iter: " << iter << " open size: " << open_->size() << std::endl;
        }

        // Get a state from the OPEN list and remove it.
        auto state  = open_->min();
        open_->pop();
        state->setClosed();

        if (isGoalState(state->state_id)){
            goal_ = state->state_id;
            getTimeFromStart(stats_.time);
            reconstructPath(seqs_path, seqs_transition_costs);
            stats_.cost = state->g;
            stats_.num_generated = (int)action_space_ptr_->states_.size();
            stats_.focal_suboptimality = params_.focal_suboptimality;
            stats_.lower_bound = state->g / params_.epsilon;
            return true;
        }

        // If it is not a goal state, then expand it.
        expand(state->state_id);
        ++iter;

        // Check if the OPEN list is empty. If so, break.
        if (open_->empty()){
            break;
        }

        // Reorder the open list.
        open_list_f_lower_bound = open_->getLowerBound();
        open_->updateWithBound(params_.focal_suboptimality * open_list_f_lower_bound);
    }

    getTimeFromStart(stats_.time);
    return false;
}

void ims::FocalwAStar::expand(int state_id){

    SearchState* state = getSearchState(state_id);
    std::vector<std::vector<int>> successor_seqs_state_ids;
    std::vector<std::vector<double>> successor_seqs_transition_costs;
    std::vector<std::vector<double>> successor_seqs_transition_subcosts;

    action_space_ptr_->getSuccessors(state->state_id, successor_seqs_state_ids, successor_seqs_transition_costs, successor_seqs_transition_subcosts);
    for (size_t i {0} ; i < successor_seqs_state_ids.size() ; ++i){
        const std::vector<int> & successor_edge_state_ids = successor_seqs_state_ids[i];
        int successor_id = successor_edge_state_ids.back();
        double successor_edge_total_cost = vectorSum(successor_seqs_transition_costs[i]);
        double successor_edge_total_subcost = vectorSum(successor_seqs_transition_subcosts[i]);
        SearchState* successor = getOrCreateSearchState(successor_id);

        // If this state does not already exists, then we add it to the open list normally.
        if (!successor->in_closed && !successor->in_open){
            setStateVals(successor->state_id,
                         state->state_id,
                         successor_edge_total_cost,
                         successor_edge_total_subcost,
                         successor_edge_state_ids,
                         successor_seqs_transition_costs[i]);
            open_->push(successor);
            successor->setOpen();
        }

        // If the state is not new, then we check if it passes the criterion for updating it either in the CLOSED list (update and insert to OPEN) or in the OPEN list (just update).
        else{
            // Compute the new tentative f, g, and c values.
            double g_new = state->g + successor_edge_total_cost;
            double c_new = state->c + successor_edge_total_subcost;
            double f_new = g_new + params_.epsilon * successor->h;

            // Check the update criterion.
            if (f_new < successor->f || (f_new == successor->f && c_new < successor->c)){
                // Update the state's parent and g value.
                setStateVals(successor->state_id,
                             state->state_id,
                             successor_edge_total_cost,
                             successor_edge_total_subcost,
                             successor_edge_state_ids,
                             successor_seqs_transition_costs[i]);

                // If the state is in the closed list, then we remove it from the closed list and insert it to the open list.
                if (successor->in_closed){
                    successor->setOpen();
                    open_->push(successor);
                }

                // If the state is in the open list, then we update its position in the open list.
                else if (successor->in_open){
                    // TODO(yoraish): this may not be needed, as the OPEN list will be reordered after the expansion anyway.
                    open_->update(successor);
                }

                // If the state is neither in the open list nor in the closed list, then we throw an error.
                else{
                    throw std::runtime_error("Found successor node that is neither in the open list nor in the closed list and is not new.");
                }
            }
        }
    }
    stats_.num_expanded++;
}


void ims::FocalwAStar::setStateVals(int state_id, int parent_id, double transition_cost, double transition_subcost)
{
    auto state_ = getSearchState(state_id);
    auto parent = getSearchState(parent_id);
    state_->parent_id = parent_id;
    state_->g = parent->g + transition_cost;
    state_->c = parent->c + transition_subcost;
    state_->h = computeHeuristic(state_id);
    state_->f = state_->g + params_.epsilon*state_->h;
}

void ims::FocalwAStar::setStateVals(int state_id,
                                    int parent_id,
                                    double transition_cost,
                                    double transition_subcost,
                                    const std::vector<int>& seq_from_parent_state_ids,
                                    const std::vector<double>& seq_from_parent_transition_costs)
{
    setStateVals(state_id, parent_id, transition_cost, transition_subcost);
    auto state = getSearchState(state_id);
    state->seq_from_parent_state_ids = std::make_shared<std::vector<int>>(seq_from_parent_state_ids);
    state->seq_from_parent_transition_costs = std::make_shared<std::vector<double>>(seq_from_parent_transition_costs);

}