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

double speed = 1;
double length = 1;
double dt = 1;

class Scene2DRob : public ims::SceneInterface {
public:
    explicit Scene2DRob(std::vector<std::vector<int>> &map_) : ims::SceneInterface(){
        map = &map_;
        map_size = {map->size(), map[0].size()};
    }

    std::vector<std::vector<int>>* map;
    std::vector<size_t> map_size;
};

class ActionSpace2dAckermannRob : public ims::ActionSpace {
    struct ActionType2dAckermannRob {

        ActionType2dAckermannRob() {
            name = "ActionTypeAckermann2dRob";
            num_actions = 5;
            action_names = {"Turn-40", "Turn-20", "Turn0", "Turn+20", "Turn+40"};
            action_costs = {1, 1, 1, 1, 1};
        }

        std::vector<Action> getPrimActionsFromTheta(int curr_theta) {
            std::vector<Action> action_prims = {};
            std::vector<int> steering_angles = {-40, -20, 0, 20, 40};
            
            for(int i = 0; i < this->num_actions; i++)
            {
                int steering_angle = steering_angles[i];
                double delta_theta = (speed/length) * steering_angle * dt;
                double approx_theta = (delta_theta/2) + curr_theta;
                double delta_x = speed * cos(approx_theta) * dt;
                double delta_y = speed * sin(approx_theta) * dt;
                action_prims.push_back({delta_x, delta_y, delta_theta});
            }
            return action_prims;
        }

        std::string name;
        int num_actions;
        std::vector<std::string> action_names;
        std::vector<double> action_costs;
    };

protected:
    std::shared_ptr<Scene2DRob> env_;
    std::shared_ptr<ActionType2dAckermannRob> action_type_;

public:
    ActionSpace2dAckermannRob(const Scene2DRob& env) : ims::ActionSpace(){
        this->env_ = std::make_shared<Scene2DRob>(env);
        ActionType2dAckermannRob action_type;
        this->action_type_ = std::make_shared<ActionType2dAckermannRob>(action_type);
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
            ActionSequence action_seq = {action};
            action_seqs.push_back(action_seq);
        }
    }

    int roundThetaToMultipleOf30(double theta) {
         // Smaller multiple 
        int a = (theta / 30) * 30; 
        
        // Larger multiple 
        int b = a + 30; 
    
        // Return of closest of two 
        return (theta - a > b - theta)? b : a; 
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
            // Round next x val
            next_state_val[0] = round(next_state_val[0]);
            // Round next y val
            next_state_val[1] = round(next_state_val[1]);
            // Round next theta val
            next_state_val[2] = (roundThetaToMultipleOf30(next_state_val[2]) + 360) % 360;

            if (isStateValid(next_state_val)){
                int next_state_ind = getOrCreateRobotState(next_state_val);
                successors.push_back(next_state_ind);
                costs.push_back(action_type_->action_costs[i]);
            }
        }
        return true;
    }

    bool isStateValid(const StateType& state_val) override{
        // checking coordinate is on map
        if (state_val[0] < 0 || state_val[0] >= (double)env_->map_size[0] || state_val[1] < 0 || state_val[1] >= (double)env_->map_size[1]){
            return false;
        }

        // checking theta value is between 0 and 3
        if (state_val[2] < 0 || state_val[2] > 3) {
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