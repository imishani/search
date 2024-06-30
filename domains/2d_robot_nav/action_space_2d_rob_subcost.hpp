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
 * \file   action_space_2d_rob_subcost.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   Oct 05 2023
*/
#pragma once

// Project includes.
#include <search/action_space/subcost_action_space.hpp>
#include <search/common/scene_interface.hpp>

// Local includes.
# include "action_space_2d_rob.hpp"

/// @brief ActionSpace for 2D robot navigation that also allows edge-transitions to be characterized by a cost and an additional subcost. An example for an algorithm that requires this is ECBS, where the subcost is the number of conflicts that would be created with other agents when traversing an edge.
class SubcostActionSpace2dRob : public ims::SubcostActionSpace {

protected:
    std::shared_ptr<Scene2DRob> env_;
    std::shared_ptr<ActionType2dRob> action_type_;

public:
    SubcostActionSpace2dRob(const Scene2DRob& env,
                     const ActionType2dRob& actions_ptr) : ims::SubcostActionSpace(){
        this->env_ = std::make_shared<Scene2DRob>(env);
        this->action_type_ = std::make_shared<ActionType2dRob>(actions_ptr);
    }

    void getActions(int state_id,
                    std::vector<ActionSequence> & action_seqs,
                    bool check_validity) override {
        ims::RobotState* curr_state = this->getRobotState(state_id);
        std::vector<ActionSequence> prim_action_seqs = action_type_->getPrimActionSequences();
        for (int i {0} ; i < action_type_->num_actions ; i++){
            ActionSequence action_seq = prim_action_seqs[i];
            if (check_validity){
                for (const Action & action : action_seq) {
                    StateType next_state_val = StateType(curr_state->state.size());
                    std::transform(curr_state->state.begin(), curr_state->state.end(), action.begin(),
                                   next_state_val.begin(), std::plus<>());
                    if (!isStateValid(next_state_val)) {
                        continue;
                    }
                }
            }
            // Each action is a sequence of states. In the most simple case, the sequence is of length 1 - only the next state.
            // In more complex cases, the sequence is longer - for example, when the action is an experience, controller or a trajectory.
            action_seqs.push_back(action_seq);
        }
    }

    bool getSuccessorEdges(int curr_state_ind,
                                   std::vector<std::vector<int>>& edges_state_ids,
                                   std::vector<std::vector<double>> & edges_transition_costs) override{
        ims::RobotState* curr_state = this->getRobotState(curr_state_ind);
        std::vector<ActionSequence> actions;
        getActions(curr_state_ind, actions, false);

        for (int i {0} ; i < actions.size() ; i++){
            // Create edges to populate.
            std::vector<int> successor_edge_state_ids;
            std::vector<double> successor_edge_transition_costs;
            ActionSequence action_seq = actions[i];

            // Transform the given action to be rooted at the current state.
            bool is_action_valid = true;
            for (size_t j {0} ; j < action_seq.size() ; j++){
                const Action & action = action_seq[j];
                StateType next_state_val = StateType(curr_state->state.size());
                std::transform(curr_state->state.begin(), curr_state->state.end(), action.begin(),
                               next_state_val.begin(), std::plus<>());
                if (!isStateValid(next_state_val)) {
                    is_action_valid = false;
                    break;
                }
                int next_state_ind = getOrCreateRobotState(next_state_val);
                // Add to action edge.
                successor_edge_state_ids.push_back(next_state_ind);
                successor_edge_transition_costs.push_back(action_type_->action_edges_transition_costs[i][j]);
            }
            if (!is_action_valid) {
                continue;
            }
            edges_state_ids.push_back(successor_edge_state_ids);
            edges_transition_costs.push_back(successor_edge_transition_costs);
        }
        return true;
    }

    // Get successors with subcosts.
    bool getSuccessorEdges(int curr_state_ind,
                                   std::vector<std::vector<int>>& edges_state_ids,
                                   std::vector<std::vector<double>> & edges_transition_costs,
                                   std::vector<std::vector<double>> & edge_transition_subcosts) override{
        ims::RobotState* curr_state = this->getRobotState(curr_state_ind);
        std::vector<ActionSequence> actions;
        getActions(curr_state_ind, actions, false);

        for (int i {0} ; i < actions.size() ; i++){
            // Create edges to populate.
            std::vector<int> successor_edge_state_ids;
            std::vector<double> successor_edge_transition_costs;
            std::vector<double> successor_edge_transition_subcosts;
            ActionSequence action_seq = actions[i];

            // Transform the given action to be rooted at the current state.
            bool is_action_valid = true;
            for (size_t j {0} ; j < action_seq.size() ; j++){
                const Action & action = action_seq[j];
                StateType next_state_val = StateType(curr_state->state.size());
                std::transform(curr_state->state.begin(), curr_state->state.end(), action.begin(),
                               next_state_val.begin(), std::plus<>());
                if (!isStateValid(next_state_val)) {
                    is_action_valid = false;
                    break;
                }
                int next_state_ind = getOrCreateRobotState(next_state_val);
                // Add to action edge.
                successor_edge_state_ids.push_back(next_state_ind);
                successor_edge_transition_costs.push_back(action_type_->action_edges_transition_costs[i][j]);
                double random_number = ((double)rand() / RAND_MAX);
                successor_edge_transition_subcosts.push_back((double)(random_number * 10));
            }
            if (!is_action_valid) {
                continue;
            }
            edges_state_ids.push_back(successor_edge_state_ids);
            edges_transition_costs.push_back(successor_edge_transition_costs);
            edge_transition_subcosts.push_back(successor_edge_transition_subcosts);
        }
        return true;
    }

    bool isStateValid(const StateType& state_val) override{
        if (state_val[0] < 0 || state_val[0] >= (double)env_->map_size[0] || state_val[1] < 0 || state_val[1] >= (double)env_->map_size[1]){
            return false;
        }
        auto map_val = env_->map->at((size_t)state_val[0]).at((size_t)state_val[1]);
        if (map_val == 100){
            return false;
        }
        return true;
    }

    bool isPathValid(const PathType& path) override{
        return std::all_of(path.begin(), path.end(), [this](const StateType& state_val){return isStateValid(state_val);});
    }

    void getTransitionSubcost(const StateType& state_val_from, const StateType& state_val_to, double & subcost) override{
        // A random subcost with value 0 or 1.
        subcost = ((double)rand() / RAND_MAX) < 0.5 ? 0 : 1;
    }
};


