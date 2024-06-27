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
 * \file   eawastar.cpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   Spetember 05 2023
*/

#include <search/planners/eawastar.hpp>

ims::EAwAStarUniformCost::EAwAStarUniformCost(const ims::EAwAStarUniformCostParams &params) : params_(params), wAStar(params) {}

ims::EAwAStarUniformCost::~EAwAStarUniformCost() = default;


void ims::EAwAStarUniformCost::initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
                                   const StateType& start, const StateType& goal) {
    // Initialize using the base class.
    wAStar::initializePlanner(action_space_ptr, start, goal);
    
    // TODO(yoraish): this is a hack. Change the required action space to be an experience accelerated constrained action space.
    action_space_ptr_ = std::dynamic_pointer_cast<ExperienceAcceleratedConstrainedActionSpace>(action_space_ptr);

    // The id of the start state.
    int start_id;
    for (auto search_state : open_){
        if (search_state->parent_id == PARENT_TYPE(START)){
            start_id = search_state->state_id;
            break;
        }
    }

    // Ask the action space to extend OPEN with any valid experience-(sub)path that includes this state.
    // The experience paths and their costs are returned in the vectors experience_paths and experience_costs.
    experienceAccelerateOpenList(start_id);
}


void ims::EAwAStarUniformCost::initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
                                   const std::vector<StateType>& start, const std::vector<StateType>& goal) {
    // Initialize using the base class.
    wAStar::initializePlanner(action_space_ptr, start, goal);

    // The id of the start state.
    int start_id;
    for (auto search_state : open_){
        if (search_state->parent_id == PARENT_TYPE(START)){
            start_id = search_state->state_id;
            break;
        }
    }

    // Ask the action space to extend OPEN with any valid experience-(sub)path that includes this state.
    // The experience paths and their costs are returned in the vectors experience_paths and experience_costs.
    experienceAccelerateOpenList(start_id);
}

void ims::EAwAStarUniformCost::experienceAccelerateOpenList(int state_id){
    // Ask the action space to extend OPEN with any valid experience-(sub)path that includes this state.
    // The experience paths and their costs are returned in the vectors experience_paths and experience_costs.
    // The returned states in the subexperiences do not have a time component.
    std::vector<std::vector<int>> experience_paths;
    std::vector<std::vector<double>> experience_paths_transition_costs;
    action_space_ptr_->getValidExperienceSubpathsFromState(state_id, experience_paths, experience_paths_transition_costs);

    // Add the experience paths to the open list. These paths are valid by construction.
    for (size_t i = 0; i < experience_paths.size(); ++i){
        if (experience_paths[i].empty()){
            continue;
        }
        addValidSubpathToOpenList(experience_paths[i], experience_paths_transition_costs[i]);
    }
}

void ims::EAwAStarUniformCost::addValidSubpathToOpenList(const std::vector<int> & state_ids, const std::vector<double> & costs){
    // First get the search state associated with the first state in the subpath.
    // We require that the first state in the subpath already exists in the action space and has a valid id and parent state. This is necessary for determining the g values and parent ids of the rest of the states in the subpath.
    // Notice that because we are planning in time (in this variant) and all the costs are uniform, then there is only a single g-value associated with each state. Therefore, we do not need to worry about updating g-values of states that are already in the open list.
    int first_state_id = state_ids[0];

    // Assert that the first state in the subpath exists in the action space and the search.
    SearchState* first_search_state = getSearchState(first_state_id);

    int prev_state_id = first_state_id;

    // Now, add the rest of the states to the open list.
    for (size_t i = 1; i < state_ids.size(); ++i){
        // Get the state id.
        int state_id = state_ids[i];
        // Get the search state.
        SearchState* search_state = getOrCreateSearchState(state_id);
        // Set the parent id.
        search_state->parent_id = prev_state_id;
        // Set the g value.
        search_state->g = getSearchState(search_state->parent_id)->g + costs[i-1];
        // Set the h value.
        search_state->h = computeHeuristic(state_id);
        // Set the f value.
        search_state->f = search_state->g + params_.epsilon*search_state->h;
        // Set the c value: collective conflict-inducing cost.
        // double transition_c_cost = action_space_ptr_->getTransitionConflictCost(prev_state_id, state_id);
        // search_state->c = getSearchState(search_state->parent_id)->c + transition_c_cost;

        // Add the state to the open list if it is not already there.
        // TODO(yoraish): does it makes sense to update states already in open? This may assign currently "good" state-parents to "bad" ones instead. For now, any state in open is not changed.
        if (!open_.contains(search_state)){
            open_.push(search_state);
            // Set the state to be open.
            search_state->setOpen();
        }
        // Update the previous state id.
        prev_state_id = state_id;
    }
}

void ims::EAwAStarUniformCost::expand(int state_id){

    auto state = getSearchState(state_id);
    std::vector<std::vector<int>> minipath_successors;
    std::vector<std::vector<double>> minipath_costs; // In this case we use the "cost" as the new f value
    action_space_ptr_->getSuccessors(state->state_id, minipath_successors, minipath_costs);
    // Strip down the multistep successors to single step successors.
    std::vector<int> successors;
    std::vector<double> costs;
    getSingleStepSuccessorsFromMultiStepSuccessors(minipath_successors, minipath_costs, successors, costs);
    for (size_t i {0} ; i < successors.size() ; ++i){
        int successor_id = successors[i];
        double cost = costs[i];
        auto successor = getOrCreateSearchState(successor_id);
        if (successor->in_closed){
            continue;
        }
        if (isGoalState(successor_id) && params_.verbose ){
            std::cout << "Added Goal to open list" << std::endl;
        }
        if (successor->in_open){
            if (successor->g > state->g + cost){
                successor->parent_id = state->state_id;
                successor->g = state->g + cost;
                successor->f = successor->g + params_.epsilon*successor->h;
                open_.update(successor);
            }
        } else {
            setStateVals(successor->state_id, state->state_id, cost);
            open_.push(successor);
            successor->setOpen();
        }
    }
    stats_.num_expanded++;

    // Ask the action space to extend OPEN with any valid experience-(sub)path that includes this state.
    // The experience paths and their costs are returned in the vectors experience_paths and experience_costs.
    experienceAccelerateOpenList(state_id);
}
