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
                    std::vector<MiniPathAction> &minipath_actions,
                    bool check_validity) override {
        ims::RobotState* curr_state = this->getRobotState(state_id);
        std::vector<MiniPathAction> actions = action_type_->getPrimActions();
        for (int i {0} ; i < action_type_->num_actions ; i++){
            MiniPathAction minipath_action = actions[i];
            if (check_validity){
                PathType minipath_successor_state_vals;
                transformStateWithMultiStepAction(curr_state->state, minipath_action, minipath_successor_state_vals);
                if (!isPathValid(minipath_successor_state_vals)){
                    continue;
                }
            }
            // Each minipath_action is a sequence of states, each a delta from the origin. In the most simple case, the sequence is of length 1 - only the next state.
            // In more complex cases, the sequence is longer - for example, when the minipath_action is an experience, controller or a trajectory.
            minipath_actions.push_back(minipath_action);
        }
    }

    bool getSuccessors(int curr_state_ind,
                       std::vector<int>& successors,
                       std::vector<double>& costs) override{
        ims::RobotState* curr_state = this->getRobotState(curr_state_ind);
        std::vector<MiniPathAction> actions;
        getActions(curr_state_ind, actions, false);
        // Multistep actions are a list of deltas from the origin. So in 2D, an example multistep action is {{1, 0}, {2, 0}, {3, 0}}
        // which means move 1 step in the x direction three times -- moving the robot from (0, 0) to (3, 0).
        for (int i {0} ; i < actions.size() ; i++){
            // Apply the action to the current state.
            std::vector<double> minipath_cost = action_type_->action_costs[i];
            PathType minipath_successor;
            transformStateWithMultiStepAction(curr_state->state, actions[i], minipath_successor);

            // Check if the successor is valid.
            if (isPathValid(minipath_successor)){
                std::vector<int> minipath_successor_state_inds = getOrCreateRobotStates(minipath_successor);
                minipath_successors.push_back(minipath_successor_state_inds);
                minipath_costs.push_back(minipath_cost);
            }
        }
        return true;
    }

    bool getSuccessors(int curr_state_ind,
                       std::vector<int>& successors,
                       std::vector<double>& costs,
                       std::vector<int> & successors_minipath) override{
        ims::RobotState* curr_state = this->getRobotState(curr_state_ind);
        std::vector<MiniPathAction> actions;
        getActions(curr_state_ind, actions, false);
        // Multistep actions are a list of deltas from the origin. So in 2D, an example multistep action is {{1, 0}, {2, 0}, {3, 0}}
        // which means move 1 step in the x direction three times -- moving the robot from (0, 0) to (3, 0).
        for (int i {0} ; i < actions.size() ; i++){
            // Apply the action to the current state.
            std::vector<double> minipath_cost = action_type_->action_costs[i];
            std::vector<double> minipath_subcost = std::vector<double>(minipath_cost.size(), 0);
            PathType minipath_successor;
            transformStateWithMultiStepAction(curr_state->state, actions[i], minipath_successor);

            // Check if the successor is valid.
            if (isPathValid(minipath_successor)){
                std::vector<int> minipath_successor_state_inds = getOrCreateRobotStates(minipath_successor);
                minipath_successors.push_back(minipath_successor_state_inds);
                minipath_costs.push_back(minipath_cost);
                // A subcost with value 0.
                minipath_subcosts.push_back(minipath_subcost);
            }
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


