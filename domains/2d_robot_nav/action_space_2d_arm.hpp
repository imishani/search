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
 * \file   action_space_2d_rob.hpp
 * \author Michelle Liu (mmliu@cmu.edu)
 * \date   3/6/24
*/

#ifndef SEARCH_ACTIONSCENE2DROB_HPP
#define SEARCH_ACTIONSCENE2DROB_HPP

#include "search/action_space/action_space.hpp"
#include <search/common/scene_interface.hpp>

#include "scene_2d_arm.cpp"

struct ActionType2dArm : public ims::ActionType {

    ActionType2dArm(int num_links = 3, int angle_incr = 2) : ims::ActionType() {

        name = "ActionType2dArm";
        num_actions = 2*num_links;
        action_names = {};
        action_costs = {};
        action_prims = {};
        for (double i = 0; i < num_links; i++) {
            action_names.push_back("R"+to_string(i));
            action_costs.push_back(1);
            std::vector<double> action_prim_R(num_links, 0);
            action_prim_R[i] = angle_incr;
            action_prims.push_back(action_prim_R);

            action_names.push_back("L"+to_string(i));
            action_costs.push_back(1);
            std::vector<double> action_prim_L(num_links, 0);
            action_prim_L[i] = -angle_incr;
            action_prims.push_back(action_prim_L);
        }

    }

    std::vector<Action> getPrimActions() override{
        return action_prims;
    }

    void Discretization(StateType& state_des) override{
        state_discretization_ = state_des;
    }

    std::string name;
    int num_actions;
    std::vector<std::string> action_names;
    std::vector<double> action_costs;
    std::vector<Action> action_prims;

};

class actionSpace2dArm : public ims::ActionSpace {

protected:
    std::shared_ptr<Scene2DArm> env_;
    std::shared_ptr<ActionType2dArm> action_type_;

public:
    actionSpace2dArm(const Scene2DArm& env,
                     const ActionType2dArm& actions_ptr) : ims::ActionSpace(){
        this->env_ = std::make_shared<Scene2DArm>(env);
        this->action_type_ = std::make_shared<ActionType2dArm>(actions_ptr);
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
            // Keep theta value between -180 and 180
            next_state_val[0] = (int(next_state_val[0])+180) % 360 - 180;
            if (isStateValid(next_state_val)){
                int next_state_ind = getOrCreateRobotState(next_state_val);
                successors.push_back(next_state_ind);
                costs.push_back(action_type_->action_costs[i]);
            }
        }
        return true;
    }

    bool isStateValid(const StateType& state_val) override{

        env_->setAngles(state_val);

        if (env_->isColliding()) {
            return false;
        }

        return true;
    }

    bool isPathValid(const PathType& path) override{
        return std::all_of(path.begin(), path.end(), [this](const StateType& state_val){return isStateValid(state_val);});
    }
};


#endif //SEARCH_ACTIONSCENE2DROB_HPP
