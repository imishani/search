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
        std::vector<ActionSequence> prim_action_seqs;
        std::vector<std::vector<double>> prim_action_transition_costs;
        action_type_->getPrimActions(prim_action_seqs, prim_action_transition_costs);
        for (int i{0}; i < action_type_->num_actions; i++) {
            ActionSequence action_seq = prim_action_seqs[i];
            if (check_validity) {
                for (const Action& action : action_seq) {
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

    bool getSuccessors(int curr_state_ind,
                       std::vector<std::vector<int>>& seqs_state_ids,
                       std::vector<std::vector<double>>& seqs_transition_costs) override {
        ims::RobotState* curr_state = this->getRobotState(curr_state_ind);
        std::vector<ActionSequence> actions;
        getActions(curr_state_ind, actions, false);

        for (int i{0}; i < actions.size(); i++) {
            // Create edges to populate.
            std::vector<int> successor_edge_state_ids;
            std::vector<double> successor_edge_transition_costs;
            ActionSequence action_seq = actions[i];

            // Transform the given action to be rooted at the current state.
            bool is_action_valid = true;
            for (size_t j{0}; j < action_seq.size(); j++) {
                const Action& action = action_seq[j];
                StateType next_state_val = StateType(curr_state->state.size());
                std::transform(curr_state->state.begin(), curr_state->state.end(), action.begin(),
                               next_state_val.begin(), std::plus<>());
                if (SLOW_EVAL) {
                    double dummy = 0;
                    for (int i = 0; i < 5000; i++) {
                        dummy += floor(pow(0.125, 0.5));
                        next_state_val[0] += 1;
                        next_state_val[0] -= 1;
                    }
                }
                if (!isStateValid(next_state_val)) {
                    is_action_valid = false;
                    break;
                }
                int next_state_ind = getOrCreateRobotState(next_state_val);
                // Add to action edge.
                successor_edge_state_ids.push_back(next_state_ind);
                successor_edge_transition_costs.push_back(action_type_->action_seqs_transition_costs[i][j]);
            }
            if (!is_action_valid) {
                continue;
            }
            seqs_state_ids.push_back(successor_edge_state_ids);
            seqs_transition_costs.push_back(successor_edge_transition_costs);
        }
        return true;
    }

    bool getSuccessor(int curr_edge_ind,
                      std::vector<int>& seqs_state_id,
                      std::vector<double>& seqs_transition_cost) override {
        auto curr_edge = this->getRobotEdge(curr_edge_ind);
        auto action_seq = curr_edge->action;
        auto cost = getActionCost(action_seq);
        bool is_action_valid = true;

        for (size_t j{0}; j < action_seq.size(); j++) {
            const Action& action = action_seq[j];
            StateType next_state_val = StateType(curr_edge->state.size());
            std::transform(curr_edge->state.begin(), curr_edge->state.end(), action.begin(),
                           next_state_val.begin(), std::plus<>());
            if (SLOW_EVAL) {
                double dummy = 0;
                for (int i = 0; i < 5000; i++) {
                    dummy += floor(pow(0.125, 0.5));
                    next_state_val[0] += 1;
                    next_state_val[0] -= 1;
                }
            }
            if (!isStateValid(next_state_val)) {
                is_action_valid = false;
                break;
            }
            int next_state_ind = getOrCreateRobotState(next_state_val);
            // Add to action edge.
            seqs_state_id.push_back(next_state_ind);
            seqs_transition_cost.push_back(cost[j]);
        }
        if (!is_action_valid) {
            return false;
        }
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

    std::vector<double> getActionCost(const ActionSequence& action) {
        std::vector<ActionSequence> prim_action_seqs;
        std::vector<std::vector<double>> prim_action_transition_costs;
        action_type_->getPrimActions(prim_action_seqs, prim_action_transition_costs);
        for (int i{0}; i < prim_action_seqs.size(); ++i) {
            if (action == prim_action_seqs[i]) {
                auto cost_seq = this->action_type_->action_seqs_transition_costs[i];
                return cost_seq;
            }
        }

        throw std::runtime_error("ActionSeq not found in action_type_'s prim action seqs");
    }
};
