/*
 * Copyright (C) 2023, Yorai Shaoul
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
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   15/07/23
 */

#ifndef SEARCH_ACTIONSCENE2DROB_HPP
#define SEARCH_ACTIONSCENE2DROB_HPP

#include "search/action_space/action_space.hpp"
#include <search/common/conflicts.hpp>
#include "search/action_space/constrained_action_space.hpp"
#include <search/common/scene_interface.hpp>

class scene2DRob : public ims::SceneInterface {
public:
    explicit scene2DRob(std::vector<std::vector<int>>& map_) : ims::SceneInterface() {
        std::cout << "Creating scene with map of shape " << map_.size() << ", " << map_[0].size() << std::endl;
        this->map = &map_;
        this->map_size = {(*map).size(), (*map)[0].size()};
    }

    std::vector<std::vector<int>>* map;
    std::vector<size_t> map_size;
};

struct actionType2dRob : public ims::ActionType {
    actionType2dRob() : ims::ActionType() {
        this->name = "actionType2dRob";
        this->num_actions = 5;
        this->action_names = {"N", "E", "S", "W", "Wait"};
        this->action_costs = {1, 1, 1, 1, 1};
        this->action_deltas = {{0, 1, 1}, {1, 0, 1}, {0, -1, 1}, {-1, 0, 1}, {0, 0, 1}};
        this->state_discretization_ = {1, 1, 1};
    }

    std::vector<Action> getPrimActions() override {
        std::vector<Action> actions;
        return this->action_deltas;
    }

    void Discretization(StateType& state_des) override {
        state_discretization_ = state_des;
    }

    std::string name;
    int num_actions;
    std::vector<std::string> action_names;
    std::vector<double> action_costs;
    std::vector<std::vector<double>> action_deltas;
};

class ConstrainedActionSpace2dRob : public ims::ConstrainedActionSpace {
private:
    std::shared_ptr<scene2DRob> env_;
    std::shared_ptr<actionType2dRob> action_type;

public:
    ConstrainedActionSpace2dRob(const scene2DRob& env,
                                const actionType2dRob& actions_ptr) : ims::ConstrainedActionSpace() {
        this->env_ = std::make_shared<scene2DRob>(env);
        this->action_type = std::make_shared<actionType2dRob>(actions_ptr);
    }

    void getActions(int state_id,
                    std::vector<ActionSequence> &action_seqs,
                    bool check_validity) override {
        auto actions = action_type->getPrimActions();
        for (int i {0} ; i < action_type->num_actions ; i++){
            auto action = actions[i];
            if (check_validity){
                auto curr_state = this->getRobotState(state_id);
                auto next_state_val = StateType(curr_state->state.size());
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

    bool isStateValid(const StateType& state_val) override {
        if (state_val[0] < 0 || state_val[0] >= (double)env_->map_size[0] || state_val[1] < 0 || state_val[1] >= (double)env_->map_size[1]) {
            return false;
        }

        auto map_val = env_->map->at((size_t)state_val[0]).at((size_t)state_val[1]);
        if (map_val == 100) {
            return false;
        }
        return true;
    }

    bool isSatisfyingConstraints(const StateType& state_val, const StateType& next_state_val) {
        // Check against constraints.
        // TODO(yoraish): check for time first.  If there are no constraints at this timestep, then the state is valid w.r.t constraints.

        // Otherwise, check if the state is valid w.r.t the constraints.
        // Iterate over the constraints. Those are in the pointer to the constraints collective, within a set pointer called constraints_ptr_.
        if (!constraints_collective_ptr_->getConstraints().empty()) {
            // Loop through the vector, and get a reference to each one of the elements.
            for (auto& constraint_ptr : constraints_collective_ptr_->getConstraints()) {

                // Check if the constraint is a vertex constraint or an edge constraint.
                switch (constraint_ptr->type) {
                    case ims::ConstraintType::VERTEX_CONSTRAINT: {
                        // Convert to a vertex constraint pointer to get access to its members.
                        auto* vertex_constraint_ptr = dynamic_cast<ims::VertexConstraint*>(constraint_ptr.get());
                        if (vertex_constraint_ptr != nullptr) {
                            // If the constraint is a vertex constraint, check if the state is valid w.r.t the constraint.
                            if (vertex_constraint_ptr->state[0] == next_state_val[0] && vertex_constraint_ptr->state[1] == next_state_val[1] && vertex_constraint_ptr->state[2] == next_state_val[2]) {
                                return false;
                            }
                        }
                        break;
                    }

                    case ims::ConstraintType::EDGE_CONSTRAINT: {
                        // Convert to an edge constraint pointer to get access to its members.
                        auto* edge_constraint_ptr = dynamic_cast<ims::EdgeConstraint*>(constraint_ptr.get());
                        if (edge_constraint_ptr != nullptr) {
                            // If the constraint is an edge constraint, check if the state is valid w.r.t the constraint.
                            if (edge_constraint_ptr->from_state[0] == state_val[0] && edge_constraint_ptr->from_state[1] == state_val[1] && edge_constraint_ptr->from_state[2] == state_val[2] && edge_constraint_ptr->to_state[0] == next_state_val[0] && edge_constraint_ptr->to_state[1] == next_state_val[1] && edge_constraint_ptr->to_state[2] == next_state_val[2]) {
                                return false;
                            }
                        }
                        break;
                    }
                }
            }
        }
        return true;
    }

    // The error error: jump to case label is resolved by

    bool isPathValid(const PathType& path) override {
        return std::all_of(path.begin(), path.end(), [this](const StateType& state_val) { return isStateValid(state_val); });
    }

    bool getSuccessors(int curr_state_ind,
                       std::vector<int>& successors,
                       std::vector<double>& costs) override {
        auto curr_state = this->getRobotState(curr_state_ind);
        std::vector<ActionSequence> actions;
        getActions(curr_state_ind, actions, false);
        for (int i {0} ; i < actions.size() ; i++){
            auto action = actions[i][0];
            auto next_state_val = StateType(curr_state->state.size());
            std::transform(curr_state->state.begin(), curr_state->state.end(), action.begin(), next_state_val.begin(), std::plus<>());

            // Check for constraint satisfaction.
            if (!isSatisfyingConstraints(curr_state->state, next_state_val)) {
                continue;
            }

            // EXAMPLE NOTE: it may sometimes make sense to check constraint satisfaction within the isStateValid method, for efficiency. For example, if a constraint requires comparison of a robot state against a world state and simulataneously the state of another robot, then it would be better to set all robots into their specified configurations and check for validity only once.
            if (isStateValid(next_state_val)) {
                int next_state_ind = getOrCreateRobotState(next_state_val);
                successors.push_back(next_state_ind);
                costs.push_back(action_type->action_costs[i]);
            }
        }
        return true;
    }

    int getOrCreateRobotState(const StateType& state_val) override {
        // check if the state exists
        auto* curr_state = new ims::RobotState;

        // Since we are dealing with time, we should take care to first check if this state is a goal state. That would be the case if, when setting the time for this state to be -1, the state already exists. By convention goal states have a time of -1. In this instance, at least.
        // Only do this if the time of the state is later than the last constraint.
        int state_time = state_val.back();
        int last_constraint_time = constraints_collective_ptr_->getLastConstraintTime();

        // TODO(yoraish): currently this method runs two "find"s to check if (a) the passed state is already seen and if the current state is a goal state. The first check is done by finding the state in the state_to_id_ object, and the second by setting the last element of the state to -1 (convention for states) and searching the state_to_id_ object again. This is inefficient and it would be better to check for the state only within the goal states. Unfortunately, the goals are noo known to the action space.
        // Only check if this state is a goal state if there are no more outstanding constraints later in time.
        if (state_time > last_constraint_time) {
            // If the state is a goal state, then we should check if it exists at the current time.
            StateType state_val_wo_time = {state_val.begin(), state_val.end() - 1};
            state_val_wo_time.push_back(-1);
            curr_state->state = state_val_wo_time;
            auto it = state_to_id_.find(curr_state);
            if (it != state_to_id_.end()) {
                delete curr_state;
                return it->second;
            }
        }

        // If that failed, then we should check if the state exists at the current time.
        curr_state->state = state_val;
        auto it = state_to_id_.find(curr_state);
        if (it != state_to_id_.end()) {
            delete curr_state;
            return it->second;
        }
        states_.push_back(curr_state);
        int state_id = (int)states_.size() - 1;
        state_to_id_[curr_state] = state_id;
        return state_id;
    }

    /// @brief Compute the cost of a path. This is domain-specific, and must be implemented in multi-agent settings.
    /// @param path
    /// @return The cost of a path.
    double computePathCost(const PathType& path) {
        double cost = 0;
        for (int i{0}; i < path.size() - 1; i++) {
            auto curr_state = path[i];
            auto next_state = path[i + 1];
            auto action = action_type->getPrimActions()[next_state[2]];
            cost += action_type->action_costs[next_state[2]];
        }
        return cost;
    }

    void getPathsConflicts(std::shared_ptr<ims::MultiAgentPaths> paths, std::vector<std::shared_ptr<ims::Conflict>>& conflicts_ptrs, int max_conflicts) {
        // Loop through the paths and check for conflicts.
        // Length of the longest path.
        int max_path_length = 0;
        for (auto& path : *paths) {
            if (path.second.size() > max_path_length) {
                max_path_length = path.second.size();
            }
        }

        // Loop through the paths and check for conflicts.
        for (int t{0}; t < max_path_length; t++) {

            for (int i{0}; i < paths->size(); i++) {
                for (int j{i + 1}; j < paths->size(); j++) {
                    // Get the position of the two robots at time t. If one of the robots is at its goal (aka its path is shorter than t), then use its last position.
                    int t_i = std::min(t, (int)paths->at(i).size() - 1);
                    int t_j = std::min(t, (int)paths->at(j).size() - 1);

                    // Check if the two paths are in a vertex conflict.
                    if (paths->at(i)[t_i][0] == paths->at(j)[t_j][0] && paths->at(i)[t_i][1] == paths->at(j)[t_j][1]) {
                        // If they are, then add a conflict to the vector.
                        StateType conflict_state = {paths->at(i)[t_i][0], paths->at(i)[t_i][1], (double)t};
                        std::shared_ptr<ims::VertexConflict> conflict_ptr = std::make_shared<ims::VertexConflict>(conflict_state, std::vector<int>{i, j});
                        conflicts_ptrs.push_back(conflict_ptr);

                        if (conflicts_ptrs.size() >= max_conflicts) {
                            return;
                        }
                    }

                    // Check if the two paths are in an edge conflict. The first check if for t being the at least before the last time step of the path, in which case there is no edge conflict.
                    if (t < paths->at(i).size() - 1 && t < paths->at(j).size() - 1) {
                        if (paths->at(i)[t][0] == paths->at(j)[t + 1][0] && paths->at(i)[t][1] == paths->at(j)[t + 1][1] && paths->at(i)[t + 1][0] == paths->at(j)[t][0] && paths->at(i)[t + 1][1] == paths->at(j)[t][1]) {
                            // If they are, then add a conflict to the vector.
                            std::shared_ptr<ims::EdgeConflict> conflict = std::make_shared<ims::EdgeConflict>(paths->at(i)[t], paths->at(i)[t + 1], i, j);
                            conflicts_ptrs.push_back(conflict);

                            if (conflicts_ptrs.size() >= max_conflicts) {
                                return;
                            }
                        }
                    }
                }
            }
        }
    }
};

#endif  // SEARCH_ACTIONSCENE2DROB_HPP
