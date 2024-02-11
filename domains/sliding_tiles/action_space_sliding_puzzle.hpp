//  * Copyright (c) 2024, Ramkumar Natarajan
//  * All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without
//  * modification, are permitted provided that the following conditions are met:
//  *
//  *     * Redistributions of source code must retain the above copyright
//  *       notice, this list of conditions and the following disclaimer.
//  *     * Redistributions in binary form must reproduce the above copyright
//  *       notice, this list of conditions and the following disclaimer in the
//  *       documentation and/or other materials provided with the distribution.
//  *     * Neither the name of the Carnegie Mellon University nor the names of its
//  *       contributors may be used to endorse or promote products derived from
//  *       this software without specific prior written permission.
//  *
//  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
//  * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  * POSSIBILITY OF SUCH DAMAGE.
//

/*!
 * \file action_space_sliding_puzzle.hpp 
 * \author Ram Natarajan (rnataraj@cs.cmu.edu)
 * \date 2/10/24
*/

#ifndef SEARCH_ACTION_SPACE_SLIDING_PUZZLE_HPP
#define SEARCH_ACTION_SPACE_SLIDING_PUZZLE_HPP

#include "search/action_space/action_space.hpp"
#include <search/common/scene_interface.hpp>
#include <search/common/eigen_types.hpp>
#include "utils.hpp"


class SceneSlidingTiles : public ims::SceneInterface {
public:
  explicit SceneSlidingTiles(int puzzle_size) : ims::SceneInterface(){
    puzzle_size_ = puzzle_size;
  }

  int puzzle_size_;
};

struct ActionTypeSlidingTiles : public ims::ActionType {

  ActionTypeSlidingTiles() : ims::ActionType() {
    name = "ActionTypeSlidingPuzzle";
    num_actions = 4;
    action_names = {'N', 'E', 'W', 'S'};
    action_costs = {1, 1, 1, 1};
    action_prims = {{0}, {1}, {2}, {3}};
    state_discretization_ = {1, 1};
  }

  std::vector<Action> getPrimActions() override{
    return action_prims;
  }

  void Discretization(StateType& state_des) override{
    state_discretization_ = state_des;
  }

  std::string name;
  int num_actions;
  std::vector<char> action_names;
  std::vector<double> action_costs;
  std::vector<Action> action_prims;

};

class actionSpaceSlidingTiles : public ims::ActionSpace {

protected:
  std::shared_ptr<SceneSlidingTiles> env_;
  std::shared_ptr<ActionTypeSlidingTiles> action_type_;

public:
  actionSpaceSlidingTiles(const SceneSlidingTiles& env,
                   const ActionTypeSlidingTiles& actions_ptr) : ims::ActionSpace(){
    this->env_ = std::make_shared<SceneSlidingTiles>(env);
    this->action_type_ = std::make_shared<ActionTypeSlidingTiles>(actions_ptr);
  }

  void getActions(int state_id,
                  std::vector<ActionSequence> &action_seqs,
                  bool check_validity) override {
    ims::RobotState* curr_state = this->getRobotState(state_id);
    std::vector<Action> actions = action_type_->getPrimActions();
    for (int i {0} ; i < action_type_->num_actions ; i++){
      Action action = actions[i];
      if (check_validity){
        if (!isActionValid(curr_state->state, action, this->env_->puzzle_size_)){
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
    getActions(curr_state_ind, actions, true);

    for (int i {0} ; i < actions.size() ; i++){
      StateType action = actions[i][0];

      auto next_state_val = applyActionToState(curr_state->state, action, this->env_->puzzle_size_);

      if (isStateValid(next_state_val)){
        int next_state_ind = getOrCreateRobotState(next_state_val);
        successors.push_back(next_state_ind);
        costs.push_back(action_type_->action_costs[i]);
      }
    }
    return true;
  }

  bool isStateValid(const StateType& state_val) override{
    return true;
  }

  bool isPathValid(const PathType& path) override{
    return std::all_of(path.begin(), path.end(), [this](const StateType& state_val){return isStateValid(state_val);});
  }
};



#endif //SEARCH_ACTION_SPACE_SLIDING_PUZZLE_HPP
