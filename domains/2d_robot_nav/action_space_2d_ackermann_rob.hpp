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
 * \date   11/20/23
*/

#pragma once

#include "search/action_space/action_space.hpp"
#include <search/common/scene_interface.hpp>

#define PI 3.14159265

class Scene2DRob : public ims::SceneInterface {
public:
    explicit Scene2DRob(std::vector<std::vector<int>> &map_) : ims::SceneInterface(){
        map = &map_;
        map_size = {map->size(), map[0].size()};
    }

    std::vector<std::vector<int>>* map;
    std::vector<size_t> map_size;
};

/// @brief An ActionSpace for a robot that follows Ackermann Steering ("Car-like") Dynamics, i.e. the robot cannot turn in-place.
class ActionSpace2dAckermannRob : public ims::ActionSpace {
    struct ActionType2dAckermannRob {
        
        /// @brief Constructor for ActionType2dAckermannRob.
        /// @param s Speed of robot (default 1).
        /// @param l Length of robot (default 1).
        /// @param t Time for robot complete the action (default 1).
        ActionType2dAckermannRob(double s, double l, double t) {
            name_ = "ActionTypeAckermann2dRob";
            num_actions_ = 5;
            action_names_ = {"Turn-40", "Turn-20", "Turn0", "Turn+20", "Turn+40"};
            action_costs_ = {1, 1, 1, 1, 1};
            speed_ = s;
            length_ = l;
            dt_ = t;
        }
        
        /// @brief Uses Ackermann steering to calculate the action primatives for a given theta.
        /// @param curr_theta Current orientation of the robot. Theta = 0 corresponds to increasing column.
        /// @return A list of possible changes in x, y, and theta based on 5 different steering angles.
        std::vector<Action> getPrimActionsFromTheta(int curr_theta) {
            std::vector<Action> action_prims = {};
            std::vector<int> steering_angles = {40, 20, 0, -20, -40};
            
            for(int i = 0; i < num_actions_; i++)
            {
                int steering_angle = steering_angles[i];
                double delta_theta = (speed_/length_) * steering_angle * dt_;
                double approx_theta = (delta_theta/2) + curr_theta;
                double delta_x = speed_ * cos(approx_theta * PI / 180.0) * dt_;
                double delta_y = speed_ * sin(approx_theta * PI / 180.0) * dt_;
                action_prims.push_back({delta_x, delta_y, delta_theta});
            }
            return action_prims;
        }

        std::string name_;
        int num_actions_;
        std::vector<std::string> action_names_;
        std::vector<double> action_costs_;
        double speed_;
        double length_;
        double dt_;
    };

protected:
    std::shared_ptr<Scene2DRob> env_;
    std::shared_ptr<ActionType2dAckermannRob> action_type_;

public:
    explicit ActionSpace2dAckermannRob(const Scene2DRob& env, StateType state_discretization, double speed = 1, double length = 1, double dt = 1) : ims::ActionSpace(){
        this->env_ = std::make_shared<Scene2DRob>(env);
        ActionType2dAckermannRob action_type = ActionType2dAckermannRob(speed, length, dt);
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
        for (int i {0} ; i < action_type_->num_actions_ ; i++){
            Action action = actions[i];
            // Each action is a sequence of states. In the most simple case, the sequence is of length 1 - only the next state.
            // In more complex cases, the sequence is longer - for example, when the action is an experience, controller or a trajectory.
            ActionSequence action_seq = {action};
            action_seqs.push_back(action_seq);
        }
    }

    int roundThetaToMultipleOf20(double theta) {
         // Smaller multiple 
        int a = (theta / 20) * 20; 
        
        // Larger multiple 
        int b = a + 20; 
    
        // Return of closest of two 
        return (theta - a > b - theta)? b : a; 
    }

    bool getSuccessors(int curr_state_ind,
                       std::vector<int>& successors,
                       std::vector<double>& costs) override{
        ims::RobotState* curr_state = this->getRobotState(curr_state_ind);
        StateType curr_state_val = curr_state->state;
        std::cout << "Curr State: " << "x: " << curr_state_val[0] << ", y: " << curr_state_val[1]<< ", theta: " << curr_state_val[2] << std::endl;
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
            next_state_val[2] = (roundThetaToMultipleOf20(next_state_val[2]) + 360) % 360;
            std::cout << "Next State Val: " << "x: " << next_state_val[0] << ", y: " << next_state_val[1]<< ", theta: " << next_state_val[2] << std::endl; 
            if (isStateValid(next_state_val)){
                int next_state_ind = getOrCreateRobotState(next_state_val);
                successors.push_back(next_state_ind);
                costs.push_back(action_type_->action_costs_[i]);
            }
        }
        return true;
    }

    /// @brief Determines if a given state is valid by checking the point is on the map, the theta is between 0 and 360, and the point is not on an obstacle.
    /// @param state_val Contains the x, y, and theta 
    /// @return True if the state is valid, false otherwise.
    bool isStateValid(const StateType& state_val) override{
        // checking coordinate is on map
        if (state_val[0] < 0 || state_val[0] >= (double)env_->map_size[0] || state_val[1] < 0 || state_val[1] >= (double)env_->map_size[1]){
            return false;
        }

        // checking theta value is between 0 and 360
        if (state_val[2] < 0 || state_val[2] >= 360) {
            return false;
        }

        // checking for obstacle
        int map_val = env_->map->at((size_t)state_val[0]).at((size_t)state_val[1]);
        if (map_val == 100){
            return false;
        }
        return true;
    }

    /// @brief Determines if all the states in a given path are valid.
    /// @param path A vector of consecutive states that make up the path.
    /// @return True if given path is valid, false otherwise.
    bool isPathValid(const PathType& path) override{
        return std::all_of(path.begin(), path.end(), [this](const StateType& state_val){return isStateValid(state_val);});
    }
};