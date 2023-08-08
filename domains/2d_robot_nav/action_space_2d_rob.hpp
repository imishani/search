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
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   4/1/23
*/

#ifndef SEARCH_ACTIONSCENE2DROB_HPP
#define SEARCH_ACTIONSCENE2DROB_HPP

#include <search/common/action_space.hpp>
#include <search/common/scene_interface.hpp>


class scene2DRob : public ims::SceneInterface {
public:
    explicit scene2DRob(std::vector<std::vector<int>> &map_) : ims::SceneInterface(){
        this->map = &map_;
        this->map_size = {map->size(), map[0].size()};
    }

    std::vector<std::vector<int>>* map;
    std::vector<size_t> map_size;
};

struct actionType2dRob : public ims::ActionType {

    actionType2dRob() : ims::ActionType() {
        this->name = "actionType2dRob";
        this->num_actions = 8;
        this->action_names = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
        this->action_costs = {1, 1.414, 1, 1.414, 1, 1.414, 1, 1.414};
        this->action_prims = {{0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}};
        this->state_discretization_ = {1, 1};
    }

    std::vector<Action> getPrimActions() override{
        std::vector<Action> actions;
        return this->action_prims;
    }

    void Discretization(StateType& state_des) override{
        state_discretization_ = state_des;
    }

    std::string name;
    int num_actions;
    std::vector<std::string> action_names;
    std::vector<double> action_costs;
    std::vector<std::vector<double>> action_prims;

};

class actionSpace2dRob : public ims::ActionSpace {

protected:
    std::shared_ptr<scene2DRob> env_;
    std::shared_ptr<actionType2dRob> actions_;

public:
    actionSpace2dRob(const scene2DRob& env,
                     const actionType2dRob& actions_ptr) : ims::ActionSpace(){
        this->env_ = std::make_shared<scene2DRob>(env);
        this->actions_ = std::make_shared<actionType2dRob>(actions_ptr);
    }

    void getActions(int state_id,
                    std::vector<ActionSequence> &actions_seq,
                    bool check_validity) override {
        auto actions = actions_->getPrimActions();
        for (int i {0} ; i < actions_->num_actions ; i++){
            auto action = actions[i];
            if (check_validity){
                auto curr_state = this->getRobotState(state_id);
                auto next_state_val = StateType(curr_state->state.size());
                std::transform(curr_state->state.begin(), curr_state->state.end(), action.begin(), next_state_val.begin(), std::plus<>());
                if (!isStateValid(next_state_val)){
                    continue;
                }
            }
            ActionSequence action_seq;
            action_seq.push_back(action);
            actions_seq.push_back(action_seq);
        }
    }


    bool getSuccessors(int curr_state_ind,
                       std::vector<int>& successors,
                       std::vector<double>& costs) override{
        auto curr_state = this->getRobotState(curr_state_ind);
        std::vector<ActionSequence> actions;
        getActions(curr_state_ind, actions, false);

        for (int i {0} ; i < actions.size() ; i++){
            auto action = actions[i][0];
            auto next_state_val = StateType(curr_state->state.size());
            std::transform(curr_state->state.begin(), curr_state->state.end(), action.begin(), next_state_val.begin(), std::plus<>());
            if (isStateValid(next_state_val)){
                int next_state_ind = getOrCreateRobotState(next_state_val);
                successors.push_back(next_state_ind);
                costs.push_back(actions_->action_costs[i]);
            }
        }
        return true;
    }

    bool isStateValid(const StateType& state_val) override{
        if (state_val[0] < 0 || state_val[0] >= (double)env_->map_size[0] || state_val[1] < 0 || state_val[1] >= (double)env_->map_size[1]){
            return false;
        }
        auto map_val = env_->map->at((size_t)state_val[0]).at((size_t)state_val[1]);
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
