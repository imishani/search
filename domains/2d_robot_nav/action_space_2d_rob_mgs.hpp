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
 * \file   action_space_2d_rob_mgs.hpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   Jan 02 2024
*/

#pragma once

// Project includes.
#include "search/action_space/action_space_mgs.hpp"
#include "search/common/scene_interface.hpp"

// Local includes.
# include "action_space_2d_rob.hpp"


/// @brief ActionSpace2dRobMGS class for the 2D robot navigation domain, using MGS (Multi-Graph Search).
class ActionSpace2dRobMGS : public ims::ActionSpaceMGS {

protected:
    std::shared_ptr<Scene2DRob> env_;
    std::shared_ptr<ActionType2dRob> action_type_;

public:

    /// @brief Constructor
    /// @param env The environment
    /// @param action_type The type of the action space
    explicit ActionSpace2dRobMGS(const Scene2DRob& env,
                                 const ActionType2dRob& action_type) :
                                 ims::ActionSpaceMGS(std::make_shared<ActionType2dRob>(action_type)) {
        this->env_ = std::make_shared<Scene2DRob>(env);
        this->action_type_ = std::make_shared<ActionType2dRob>(action_type);
    }

    void getActions(int state_id,
                    std::vector<ActionSequence> &action_seqs,
                    bool check_validity) override {
        ims::RobotState* curr_state = this->getRobotState(state_id);
        std::vector<Action> actions = action_type_->getPrimActions();
        for (int i {0} ; i < action_type_->num_actions ; i++){
            Action action = actions[i];
            if (check_validity){
                StateType next_state_val = StateType(curr_state->state.size());
                std::transform(curr_state->state.begin(), curr_state->state.end(), action.begin(), next_state_val.begin(), std::plus<>());
                if (!isStateValid(next_state_val)){
                    continue;
                }
            }
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

            if (isStateValid(next_state_val)){
                int next_state_ind = getOrCreateRobotState(next_state_val);
                successors.push_back(next_state_ind);
                costs.push_back(action_type_->action_costs[i]);
            }
        }
        return true;
    }

    bool isStateValid(const StateType& state_val) override{
        if (state_val[0] < 0 || state_val[0] >= (double)env_->map_size[0] || state_val[1] < 0 || state_val[1] >= (double)env_->map_size[1]){
            return false;
        }
        int map_val = env_->map->at((size_t)state_val[0]).at((size_t)state_val[1]);
        if (map_val == 100){
            return false;
        }
        return true;
    }

    bool isPathValid(const PathType& path) override{
        return std::all_of(path.begin(), path.end(), [this](const StateType& state_val){return isStateValid(state_val);});
    }

    bool generateRandomState(const StateType& s1,
                             const StateType& s2,
                             double dist,
                             StateType& random_state) override {
        return ActionSpaceMGS::generateRandomState(s1, s2, dist, random_state);
        //// TODO: This is working less well then the action_space_msg implementation
        // sample random x, y in the map (env_->map_size)
        if (s1.size() != 2 || s2.size() != 2){
            std::cout << "Error: state size is not 2!" << std::endl;
            return false;
        }
        // maximum attempts to generate a random state
        int max_attempts = 100;
        int attempts = 0;
        while (attempts < max_attempts){
            // get the random state
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> dis(0, 1);
            double x = dis(gen) * (double)env_->map_size[0];
            double y = dis(gen) * (double)env_->map_size[1];
            // check if the state is valid
            if (isStateValid({x, y})){
                random_state = {x, y};
                return true;
            } else {
                attempts++;
            }
        }
        return false;
    }
};
