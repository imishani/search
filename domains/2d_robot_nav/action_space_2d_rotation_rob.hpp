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

#ifndef SEARCH_ACTIONSCENE2DROB_HPP
#define SEARCH_ACTIONSCENE2DROB_HPP

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

struct ActionType2dRotationRob {

    ActionType2dRotationRob() {
        name = "ActionTypeRotation2dRob";
        num_actions = 3;
        action_names = {"F", "L", "R"};
        action_costs = {1, 1, 1};
    }

    std::vector<Action> getPrimActionsFromTheta(int curr_theta) {
        Action forward_prim;
        switch (curr_theta) {
            case 0: // Increase column
                forward_prim = {1, 0, 0};
                break;
            case 1: // decrease row
                forward_prim = {0, -1, 0};
                break;
            case 2: // decrease column
                forward_prim = {-1, 0, 0};
                break;
            case 3: // increase column
                forward_prim = {0, 1, 0};
                break;
            default:
                std::cout << "Theta is not valid!" << std::endl;
        }
        Action left_prim = {0, 0, -1};
        Action right_prim = {0, 0, 1};
        std::vector<Action> action_prims = {forward_prim, left_prim, right_prim};
        return action_prims;
    }

    std::string name;
    int num_actions;
    std::vector<std::string> action_names;
    std::vector<double> action_costs;
};

class ActionSpace2dRotationRob : public ims::ActionSpace {

protected:
    std::shared_ptr<Scene2DRob> env_;
    std::shared_ptr<ActionType2dRotationRob> action_type_;

public:
    ActionSpace2dRotationRob(const Scene2DRob& env,
                     const ActionType2dRotationRob& actions_ptr) : ims::ActionSpace(){
        this->env_ = std::make_shared<Scene2DRob>(env);
        this->action_type_ = std::make_shared<ActionType2dRotationRob>(actions_ptr);
    }

    void getActions(int state_id,
                    std::vector<ActionSequence> &action_seqs,
                    bool check_validity) override {
        // validity is checked in the getSuccessors() function, no need to check validity in this function
        assert(check_validity == false);
        ims::RobotState* curr_state = this->getRobotState(state_id);
        int curr_state_theta = (curr_state->state)[2];
        std::vector<Action> actions = action_type_->getPrimActionsFromTheta(curr_state_theta);
        for (int i {0} ; i < action_type_->num_actions ; i++){
            Action action = actions[i];
            // Each action is a sequence of states. In the most simple case, the sequence is of length 1 - only the next state.
            // In more complex cases, the sequence is longer - for example, when the action is an experience, controller or a trajectory.
            ActionSequence action_seq;
            action_seq.push_back(action);
            action_seqs.push_back(action_seq);
        }
    }


    bool getSuccessors(int curr_state_ind,
                       std::vector<int>& successors,
                       std::vector<double>& costs) override{
        ims::RobotState* curr_state = this->getRobotState(curr_state_ind);
        std::vector<ActionSequence> actions;
        getActions(curr_state_ind, actions, false);

        for (int i {0} ; i < actions.size() ; i++){
            StateType action = actions[i][0];
            StateType next_state_val = StateType(curr_state->state.size());
            std::transform(curr_state->state.begin(), curr_state->state.end(), action.begin(), next_state_val.begin(), std::plus<>());
            // keep theta value between 0 and 3
            next_state_val[2] = double((int(next_state_val[2])+4) % 4);
            if (isStateValid(next_state_val)){
                int next_state_ind = getOrCreateRobotState(next_state_val);
                successors.push_back(next_state_ind);
                costs.push_back(action_type_->action_costs[i]);
            }
        }
        return true;
    }

    bool isStateValid(const StateType& state_val) override{
        // checking coordinate is on map and theta value is between 0 and 3
        if (state_val[0] < 0 || state_val[0] >= (double)env_->map_size[0] || state_val[1] < 0 || state_val[1] >= (double)env_->map_size[1] || state_val[2] < 0 || state_val[2] > 3){
            return false;
        }
        // checking for obstacle
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


#endif //SEARCH_ACTIONSCENE2DROB_HPP