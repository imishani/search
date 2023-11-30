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
        ActionType2dAckermannRob(StateType state_discretization, double s, double l, double t) {
            name_ = "ActionTypeAckermann2dRob";
            num_actions_ = 5;
            action_names_ = {"Turn-40", "Turn-20", "Turn0", "Turn+20", "Turn+40"};
            action_costs_ = {1, 1, 1, 1, 1};
            speed_ = s;
            length_ = l;
            dt_ = t;
            state_discretization_ = state_discretization;
        }
        
        /// @brief Uses Ackermann steering to calculate the action primatives for a given theta.
        /// @param curr_theta Current orientation of the robot. Theta = 0 corresponds to increasing column.
        /// @return A list of possible changes in x, y, and theta based on 5 different steering angles.
        std::vector<Action> getPrimActionsFromTheta(double curr_theta) {
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
        
        /// @brief Rounds the number to the nearest multiple of the discretization.
        /// @param discretization The rounding factor.
        /// @param num The number to be rounded.
        /// @return The discretizaed version of the number.
        double roundByDiscretization(double discretization, double num) {
            return round(num / discretization) * discretization; 
        }

        /// @brief Discretizes each element in the action vector according to the state_discretion vector.
        /// @param action A vector representing changes to x, y, theta for an action.
        /// @return The discretixed version of the inputed action.
        Action discretizeAction(Action action) {
            Action discretized_action = {};
            for (int i = 0; i < action.size(); i++) {
                discretized_action.push_back(roundByDiscretization(state_discretization_[i], action[i]));
            }
            return discretized_action;
        }

        /// @brief Removes duplicate actions from a vector of actions.
        /// @param actions A vector of actions (a.k.a a vector of vectors).
        /// @return The input vector with all the duplicates removed.
        std::vector<Action> removeDuplicates(std::vector<Action> actions) {
            // Step 1: Sort the vector
            std::sort(actions.begin(), actions.end());

            // Step 2: Use std::unique to rearrange the vector and get the end iterator of the unique elements
            auto last = std::unique(actions.begin(), actions.end());

            // Step 3: Erase the duplicates from the vector
            actions.erase(last, actions.end());
            return actions;
        }

        /// @brief Iterates through the map to print each key-value pair.
        /// @param map A map with a key type of double (theta) and value type of vector<Action> (action prims).
        void printActionPrimsMap(std::map<double, std::vector<Action>> map) {
            for (const auto& pair : map) {
                std::cout << "Key: " << pair.first << ", Values: ";
                
                // Print the vector of vectors associated with the current key
                for (const auto& innerVector : pair.second) {
                    std::cout << "[ ";
                    for (const auto& value : innerVector) {
                        std::cout << value << " ";
                    }
                    std::cout << "] ";
                }
                
                std::cout << std::endl;
            }
        }

        /// @brief Uses the state discretization of the action space to create a map from each possible theta value to a vector of unique action prims.
        /// @return A map from each possible theta value to a vector of unique action prims.
        std::map<double, std::vector<Action>> makeActionPrimsMap() {
            std::map<double, std::vector<Action>> apm;

            for(double theta = 0.0; theta <= 360.0; theta += state_discretization_[2]){
                std::vector<Action> action_prims = getPrimActionsFromTheta(theta);
                for (int i = 0; i < action_prims.size(); i++) {
                    action_prims[i] = discretizeAction(action_prims[i]);
                }

                action_prims = removeDuplicates(action_prims);

                apm[theta] = action_prims;
            }
            printActionPrimsMap(apm);
            return apm;
        }

        std::string name_;
        int num_actions_;
        std::vector<std::string> action_names_;
        std::vector<double> action_costs_;
        double speed_;
        double length_;
        double dt_;
        StateType state_discretization_;
    };

protected:
    std::shared_ptr<Scene2DRob> env_;
    std::shared_ptr<ActionType2dAckermannRob> action_type_;
    std::map<double, std::vector<Action>> action_prims_map_;

public:
    explicit ActionSpace2dAckermannRob(const Scene2DRob& env, StateType state_discretization, double speed = 1, double length = 1, double dt = 1) : ims::ActionSpace(){
        this->env_ = std::make_shared<Scene2DRob>(env);
        ActionType2dAckermannRob action_type = ActionType2dAckermannRob(state_discretization, speed, length, dt);
        this->action_type_ = std::make_shared<ActionType2dAckermannRob>(action_type);
        this->action_prims_map_ = action_type_->makeActionPrimsMap();
    }

    /// @brief Determines the possible actions for a given state.
    /// @param state_id The current state of the robot (x, y, theta).
    /// @param action_seqs The vector to populated with possible actions for the robot to have from the given state.
    /// @param check_validity Should be false, validity of the action is checked in the getSuccessors function.
    void getActions(int state_id,
                    std::vector<ActionSequence> &action_seqs,
                    bool check_validity) override {
        // validity is checked in the getSuccessors() function, no need to check validity in this function
        assert(check_validity == false);
        
        ims::RobotState* curr_state = this->getRobotState(state_id);
        double curr_state_theta = (curr_state->state)[2];
        std::vector<Action> actions = this->action_prims_map_[curr_state_theta];
        for (int i {0} ; i < actions.size(); i++){
            Action action = actions[i];
            // Each action is a sequence of states. In the most simple case, the sequence is of length 1 - only the next state.
            // In more complex cases, the sequence is longer - for example, when the action is an experience, controller or a trajectory.
            ActionSequence action_seq = {action};
            action_seqs.push_back(action_seq);
        }
    }

    /// @brief Determines the possible successor states for a given state.
    /// @param curr_state_ind The current state of the the robot (x, y, theta).
    /// @param successors The vector to poputate with the successor states for the current state.
    /// @param costs The vector to populate with the correspondings costs to move to each successor state.
    /// @return True if the successor states and costs and successfully calculated.
    bool getSuccessors(int curr_state_ind,
                       std::vector<int>& successors,
                       std::vector<double>& costs) override{
        ims::RobotState* curr_state = this->getRobotState(curr_state_ind);
        StateType curr_state_val = curr_state->state;
        std::vector<ActionSequence> actions;
        getActions(curr_state_ind, actions, false);
        for (int i {0} ; i < actions.size() ; i++){
            StateType action = actions[i][0];
            StateType next_state_val = StateType(curr_state->state.size());
            std::transform(curr_state->state.begin(), curr_state->state.end(), action.begin(), next_state_val.begin(), std::plus<>());
            if (isPathValid(getDiscretePointsOnLine(curr_state_val, next_state_val)) && isStateValid(next_state_val)) {
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

    /// @brief Given two points on a grid, this function calculates the discretized points on the line segment that connects the points.
    /// @param start The starting state of the line segment.
    /// @param end The ending state of the line segment.
    /// @return A list of discretized states (PathType) on the line segment.
    PathType getDiscretePointsOnLine(StateType start, StateType end) {
        double x1 = start[0];
        double x2 = end[0];
        double y1 = start[1];
        double y2 = end[1];
        double slope = (y2 - y1) / (x2 - x1);
        double intercept = y1 - (slope * x1);

        double x_discretion = this->action_type_->state_discretization_[0];
        double y_discretion = this->action_type_->state_discretization_[1];
        double delta_x = (x1 < x2) ? x_discretion : -1 * x_discretion;
        
        double theta = end[2];
        PathType points = {};
        for (double x = x1; x != x2; x += delta_x) {
            double y = this->action_type_->roundByDiscretization(y_discretion, (x * slope) + intercept);
            points.push_back({x, y, theta});
        }
        return points;
    }
};