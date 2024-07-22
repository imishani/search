/*
 * Copyright (C) 2024, Hanlan Yang
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
 * \file   action_space_2d_rob_edge.hpp
 * \author Hanlan Yang (yanghanlan666@gmail.com)
 * \date   7/20/24
 */

#pragma once

// Project includes.
#include <search/action_space/edge_action_space.hpp>
#include <search/common/scene_interface.hpp>

// Local includes.
#include "action_space_2d_rob.hpp"

/// @brief ActionSpace for 2D robot navigation that also allows edge representation
class edgeActionSpace2dRob : public ims::EdgeActionSpace {
   protected:
    std::shared_ptr<Scene2DRob> env_;
    std::shared_ptr<ActionType2dRob> action_type_;

   public:
    edgeActionSpace2dRob(const Scene2DRob& env,
                         const ActionType2dRob& actions_ptr) : ims::EdgeActionSpace() {
        this->env_ = std::make_shared<Scene2DRob>(env);
        this->action_type_ = std::make_shared<ActionType2dRob>(actions_ptr);
    }

    void getActions(int state_id,
                    std::vector<ActionSequence>& action_seqs,
                    bool check_validity) override {
        ims::RobotState* curr_state = this->getRobotState(state_id);
        std::vector<Action> actions = action_type_->getPrimActions();
        for (int i{0}; i < action_type_->num_actions; i++) {
            Action action = actions[i];
            if (check_validity) {
                StateType next_state_val = StateType(curr_state->state.size());
                std::transform(curr_state->state.begin(), curr_state->state.end(), action.begin(), next_state_val.begin(), std::plus<>());
                if (!isStateValid(next_state_val)) {
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
                       std::vector<double>& costs) override {
        ims::RobotState* curr_state = this->getRobotState(curr_state_ind);
        std::vector<ActionSequence> actions;
        getActions(curr_state_ind, actions, false);

        for (int i{0}; i < actions.size(); i++) {
            StateType action = actions[i][0];
            StateType next_state_val = StateType(curr_state->state.size());
            std::transform(curr_state->state.begin(), curr_state->state.end(), action.begin(), next_state_val.begin(), std::plus<>());

            // if (1) {
            //     double dummy = 0;
            //     for (int i = 0; i < 5000; i++) {
            //         dummy += floor(pow(0.125, 0.5));
            //         next_state_val[0] += 1;
            //         next_state_val[0] -= 1;
            //     }
            // }
            if (isStateValid(next_state_val)) {
                int next_state_ind = getOrCreateRobotState(next_state_val);
                successors.push_back(next_state_ind);
                costs.push_back(action_type_->action_costs[i]);
            }
        }
        return true;
    }

    bool getSuccessor(int curr_edge_ind,
                      int& successor,
                      double& cost) override {
        auto curr_edge = this->getRobotEdge(curr_edge_ind);
        StateType next_state_val = StateType(curr_edge->state);
        cost = 0;
        for (auto act : curr_edge->action) {
            std::transform(next_state_val.begin(), next_state_val.end(), act.begin(), next_state_val.begin(), std::plus<>());
            if (!isStateValid(next_state_val)) {
                return false;
            }
            cost += getActionCost(act);
        }
        successor = getOrCreateRobotState(next_state_val);
        return true;
    }

    bool isStateValid(const StateType& state_val) override {
        if (state_val[0] < 0 || state_val[0] >= (double)env_->map_size[0] || state_val[1] < 0 || state_val[1] >= (double)env_->map_size[1]) {
            return false;
        }
        int map_val = env_->map->at((size_t)state_val[0]).at((size_t)state_val[1]);
        if (map_val == 100) {
            return false;
        }
        return true;
    }

    bool isPathValid(const PathType& path) override {
        return std::all_of(path.begin(), path.end(), [this](const StateType& state_val) { return isStateValid(state_val); });
    }

    double getActionCost(const Action& action) {
        std::vector<Action> actions = this->action_type_->getPrimActions();
        for (int i{0}; i < actions.size(); ++i) {
            if (action == actions[i]) {
                return this->action_type_->action_costs[i];
            }
        }

        throw std::runtime_error("Action not found in action_type_");
    }
};
