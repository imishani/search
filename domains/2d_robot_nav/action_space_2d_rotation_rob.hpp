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
 * \file   action_space_rotation_2d_rob.hpp
 * \author Carina Sanborn (czsanbor@andrew.cmu.edu)
 * \date   10/25/23
*/

#pragma once 

#include "search/action_space/action_space.hpp"
#include <search/common/scene_interface.hpp>


class Scene2DRob : public ims::SceneInterface {
public:
    explicit Scene2DRob(std::vector<std::vector<int>> &map_) : ims::SceneInterface(){
        map = &map_;
        map_size = {map->size(), map[0].size()};
    }

    std::vector<std::vector<int>>* map;
    std::vector<size_t> map_size;
};

class ActionSpace2dRotationRob : public ims::ActionSpace {
    struct ActionType2dRotationRob {

        ActionType2dRotationRob() {
            name = "ActionTypeRotation2dRob";
            num_actions = 3;
            action_names = {"F", "L", "R"};
            action_costs = {{1}, {1}, {1}};
        }

        std::vector<MiniPathAction> getPrimActionsFromTheta(int curr_theta) {
            MiniPathAction forward_prim;
            switch (curr_theta) {
                case 0: // increase column
                    forward_prim = {{1, 0, 0}};
                    break;
                case 1: // decrease row
                    forward_prim = {{0, -1, 0}};
                    break;
                case 2: // decrease column
                    forward_prim = {{-1, 0, 0}};
                    break;
                case 3: // increase row
                    forward_prim = {{0, 1, 0}};
                    break;
                default:
                    std::cout << "Theta is not valid!" << std::endl;
            }
            MiniPathAction rotate_left_prim = {{0, 0, 1}};
            MiniPathAction rotate_right_prim = {{0, 0, -1}};
            std::vector<MiniPathAction> action_prims = {forward_prim, rotate_left_prim, rotate_right_prim};
            return action_prims;
        }

        std::string name;
        int num_actions;
        std::vector<std::string> action_names;
        std::vector<std::vector<double>> action_costs;
    };

protected:
    std::shared_ptr<Scene2DRob> env_;
    std::shared_ptr<ActionType2dRotationRob> action_type_;

public:
    explicit ActionSpace2dRotationRob(const Scene2DRob& env) : ims::ActionSpace(){
        this->env_ = std::make_shared<Scene2DRob>(env);
        ActionType2dRotationRob action_type;
        this->action_type_ = std::make_shared<ActionType2dRotationRob>(action_type);
    }

    void getActions(int state_id,
                    std::vector<MiniPathAction> &action_seqs,
                    bool check_validity) override {
        // Validity is checked in the getSuccessors() function, so no need to check validity in this function.
        assert(check_validity == false);
        
        ims::RobotState* curr_state = this->getRobotState(state_id);
        int curr_state_theta = (int)(curr_state->state)[2];
        std::vector<MiniPathAction> actions = action_type_->getPrimActionsFromTheta(curr_state_theta);
        for (int i {0} ; i < action_type_->num_actions ; i++){
            MiniPathAction action = actions[i];
            action_seqs.push_back(action);
        }
    }


    bool getSuccessors(int curr_state_ind,
                       std::vector<std::vector<int>>& minipath_successors,
                       std::vector<std::vector<double>>& minipath_costs) override{
        ims::RobotState* curr_state = this->getRobotState(curr_state_ind);

        std::vector<MiniPathAction> actions;
        getActions(curr_state_ind, actions, false);
        for (int i {0} ; i < actions.size() ; i++){
            // Apply the action to the current state.
            std::vector<double> minipath_cost = action_type_->action_costs[i];
            PathType minipath_successor;
            transformStateWithMultiStepAction(curr_state->state, actions[i], minipath_successor);

            // Keep theta value between 0 and 3. NOTE(yorai): why do we do it like this and not in the discretization?
            for (int j {0} ; j < minipath_successor.size() ; j++){
                minipath_successor[j][2] = (int(minipath_successor[j][2])+4) % 4;
            }

            // Check if the successor is valid.
            if (isPathValid(minipath_successor)){
                std::vector<int> minipath_successor_state_inds = getOrCreateRobotStates(minipath_successor);
                minipath_successors.push_back(minipath_successor_state_inds);
                minipath_costs.push_back(minipath_cost);
            }
        }
        return true;
    }

    bool isStateValid(const StateType& state_val) override{
        // Checking coordinate is on map.
        if (state_val[0] < 0 || state_val[0] >= (double)env_->map_size[0] || state_val[1] < 0 || state_val[1] >= (double)env_->map_size[1]){
            return false;
        }

        // Checking theta value is between 0 and 3.
        if (state_val[2] < 0 || state_val[2] > 3) {
            return false;
        }

        // Checking for obstacle.
        int map_val = env_->map->at((size_t)state_val[0]).at((size_t)state_val[1]);
        if (map_val == 100){
            return false;
        }
        return true;
    }

    bool isPathValid(const PathType& path) override{
        return std::all_of(path.begin(), path.end(), [this](const StateType& state_val){return isStateValid(state_val);});
    }
};
