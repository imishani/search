/*
 * Copyright (C) 2024, Itamar Mishani
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
 * \file   action_space_2d_ackerman_rob_mgs.hpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   Jan 09 2024
*/

#pragma once

// Project includes.
#include "search/action_space/action_space_mgs.hpp"
#include <search/common/scene_interface.hpp>
#include "utils.hpp"

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
class ActionSpace2dAckermannRobMGS : public ims::ActionSpaceMGS {
    struct ActionType2dAckermannRob {

        /// @brief Constructor for ActionType2dAckermannRob.
        /// @param s Speed of robot (default 1).
        /// @param l Length of robot (default 1).
        /// @param t Time for robot complete the action (default 1).
        ActionType2dAckermannRob(const StateType& state_discretization, double s, double l, double t, const std::vector<int> &steering_angles) {
            name_ = "ActionTypeAckermann2dRob";
            num_actions_ = 5;
            action_names_ = {"Turn-40", "Turn-20", "Turn0", "Turn+20", "Turn+40"};
            action_costs_ = {1, 1, 1, 1, 1};
            speed_ = s;
            length_ = l;
            dt_ = t;
            state_discretization_ = state_discretization;
            steering_angles_ = steering_angles;
            action_prims_map_ = makeActionPrimsMap();
        }

        /// @brief Uses Ackermann steering to calculate the action primatives for a given theta.
        /// @param curr_theta Current orientation of the robot. Theta = 0 corresponds to increasing column.
        /// @return A list of possible changes in x, y, and theta based on 5 different steering angles.
        std::vector<Action> getPrimActionsFromTheta(double curr_theta) {
            std::vector<Action> action_prims = {};

            for(int i = 0; i < num_actions_; i++)
            {
                int steering_angle = steering_angles_[i];
                double delta_theta = (speed_/length_) * steering_angle * dt_;
                double approx_theta = (delta_theta/2) + curr_theta;
                double delta_x = speed_ * cos(degsToRads(approx_theta)) * dt_;
                // Need to flip y (In 2D grid, the row increases as you move to the bottom of the grid).
                double delta_y = -1 * speed_ * sin(degsToRads(approx_theta)) * dt_;
                action_prims.push_back({delta_x, delta_y, delta_theta});
            }
            return action_prims;
        }

        /// @brief Iterates through the map to print each key-value pair.
        /// @param map A map with a key type of double (theta) and value type of vector<Action> (action prims).
        void printActionPrimsMap(const std::map<double, std::vector<Action>> &map) {
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

            for(double theta = 0.0; theta < 360.0; theta += state_discretization_[2]){
                std::vector<Action> action_prims = getPrimActionsFromTheta(theta);
                for (int i = 0; i < action_prims.size(); i++) {
                    action_prims[i] = discretizeState(action_prims[i], state_discretization_);
                }

                action_prims = removeDuplicateActions(action_prims);

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
        std::vector<int> steering_angles_;
        std::map<double, std::vector<Action>> action_prims_map_;
    };

protected:
    std::shared_ptr<Scene2DRob> env_;
    std::shared_ptr<ActionType2dAckermannRob> action_type_ = nullptr;
    std::map<double, std::vector<Action>> action_prims_map_;

public:
    ActionSpace2dAckermannRobMGS(const Scene2DRob& env, const StateType& state_discretization,
                                       double speed = 1, double length = 1, double dt = 1,
                                       std::vector<int> steering_angles = {-40, -20, 0, 20, 40}) :
                                       ims::ActionSpaceMGS(){
        this->env_ = std::make_shared<Scene2DRob>(env);
        ActionType2dAckermannRob action_type = ActionType2dAckermannRob(state_discretization, speed, length, dt, steering_angles);
        this->action_type_ = std::make_shared<ActionType2dAckermannRob>(action_type);
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

        ims::RobotState* curr_state = getRobotState(state_id);
        double curr_state_theta = (curr_state->state)[2];
        std::vector<Action> actions = action_type_->action_prims_map_[curr_state_theta];
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
        ims::RobotState* curr_state = getRobotState(curr_state_ind);
        StateType curr_state_val = curr_state->state;
        std::vector<ActionSequence> actions;
        getActions(curr_state_ind, actions, false);
        for (int i {0} ; i < actions.size() ; i++){
            StateType action = actions[i][0];
            StateType next_state_val = StateType(curr_state->state.size());
            std::transform(curr_state->state.begin(), curr_state->state.end(), action.begin(), next_state_val.begin(), std::plus<>());
            next_state_val[2] = double((int)(next_state_val[2] + 360) % 360);
            if (isPathValid(getDiscretePointsOnLine(curr_state_val, next_state_val, action_type_->state_discretization_)) && isStateValid(next_state_val)) {
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

    bool generateRandomState(const StateType& s1,
                             const StateType& s2,
                             double dist,
                             StateType& random_state) override {
        // maximum attempts to generate a random state
        int max_attempts = 1000;
        int attempts = 0;
        while (attempts < max_attempts){
            // get the random state
            std::random_device rd;
            std::mt19937 gen(rd());
//            std::uniform_real_distribution<> dis(0, 1);
            std::normal_distribution<> dis(0.5, 0.25);
            ////////// Ellipsoid ////////// TODO: Fix
            double major_axis = dist;
//            double minor_axis = dist * dis(gen);
//            double ratio = minor_axis / major_axis;
            StateType diff(s1.size());
            for (size_t i {0}; i < s1.size(); ++i){
                diff[i] = s2[i] - s1[i];
            }
            random_state.resize(s1.size());
            for (size_t i {0}; i < 2; ++i){
                double minor_axis = dist * dis(gen);
                double ratio = minor_axis / major_axis;
                random_state[i] = s1[i] + ratio * diff[i];
                random_state[i] = std::round(random_state[i] / action_type_->state_discretization_[i]) * action_type_->state_discretization_[i];
            }
            // generate random theta
            std::random_device rd1;
            std::mt19937 gen1(rd1());
            std::uniform_real_distribution<> dis1(0, 360);
            double theta = dis1(gen1);
            // round to discretization
            theta = std::round(theta / action_type_->state_discretization_[2]) * action_type_->state_discretization_[2];
            random_state[2] = theta;
            // check if the state is valid
            if (isStateValid(random_state)){
                return true;
            } else {
                attempts++;
            }
        }
        return false;
    }
};
