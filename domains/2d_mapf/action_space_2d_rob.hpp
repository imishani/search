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
#include "search/action_space/subcost_action_space.hpp"
#include <search/planners/multi_agent/cbs.hpp>

#include "scene_interface_2d_rob.hpp"

struct ActionType2dRob : public ims::ActionType {
    ActionType2dRob() : ims::ActionType() {
        this->name = "ActionType2dRob";
        this->num_actions = 4;
        this->action_names = {"N", "E", "S", "W"};
        this->action_costs = {{1}, {1}, {1}, {1}};
        this->action_deltas = {{{0, 1}}, {{1, 0}}, {{0, -1}}, {{-1, 0}}};
        this->state_discretization_ = {1, 1};
    }

    std::vector<MiniPathAction> getPrimActions() override {
        return this->action_deltas;
    }

    void Discretization(StateType& state_des) override {
        state_discretization_ = state_des;
    }

    std::string name;
    int num_actions;
    std::vector<std::string> action_names;
    std::vector<std::vector<double>> action_costs;
    std::vector<MiniPathAction> action_deltas;
};

struct ActionType2dRobTimed : public ActionType2dRob {
    ActionType2dRobTimed() : ActionType2dRob() {
        this->name = "ActionType2dRob";
        this->num_actions = 5;
        this->action_names = {"N", "E", "S", "W", "Wait"};
        this->action_costs = {{1}, {1}, {1}, {1}, {1}};
        this->action_deltas = {{{0, 1, 1}, {0,2,2}}, {{1, 0, 1}}, {{0, -1, 1}}, {{-1, 0, 1}}, {{0, 0, 1}}};
        this->state_discretization_ = {1, 1, 1};
    }
};

class ConstrainedActionSpace2dRob : public ims::SubcostConstrainedActionSpace {
private:
    std::shared_ptr<Scene2DRob> env_;
    std::shared_ptr<ActionType2dRob> action_type_;

public:
    ConstrainedActionSpace2dRob(const std::shared_ptr<Scene2DRob>& env,
                                const ActionType2dRob& actions_ptr) : ims::SubcostConstrainedActionSpace() {
        this->env_ = env;
        this->action_type_ = std::make_shared<ActionType2dRob>(actions_ptr);
    }

    void getActions(int state_id,
                    std::vector<MiniPathAction> &minipath_actions,
                    bool check_validity) override {
        ims::RobotState* curr_state = this->getRobotState(state_id);
        std::vector<MiniPathAction> actions = action_type_->getPrimActions();
        for (int i {0} ; i < action_type_->num_actions ; i++){
            MiniPathAction minipath_action = actions[i];
            if (check_validity){
                PathType minipath_successor_state_vals;
                transformStateWithMultiStepAction(curr_state->state, minipath_action, minipath_successor_state_vals);
                if (!isPathValid(minipath_successor_state_vals)){
                    continue;
                }
            }
            // Each minipath_action is a sequence of states, each a delta from the origin. In the most simple case, the sequence is of length 1 - only the next state.
            // In more complex cases, the sequence is longer - for example, when the minipath_action is an experience, controller or a trajectory.
            minipath_actions.push_back(minipath_action);
        }
    }

    bool isStateValid(const StateType& state_val) override {
        // TODO: Change col row to row col
        double col = state_val[0];
        double row = state_val[1];
        return env_->isCellValid(row, col);
    }

    bool isSatisfyingConstraint(const StateType &state_val, const StateType &next_state_val, const std::shared_ptr<ims::Constraint> &constraint_ptr) override {
        // Check if the constraint is a vertex constraint or an edge constraint.
        switch (constraint_ptr->type) {
            case ims::ConstraintType::VERTEX: {
                // Convert to a vertex constraint pointer to get access to its members.
                auto* vertex_constraint_ptr = dynamic_cast<ims::VertexConstraint*>(constraint_ptr.get());
                if (vertex_constraint_ptr != nullptr) {
                    // If the constraint is a vertex constraint, check if the state is valid w.r.t the constraint.
                    // note(yoraish): the state includes time so the check for equality in time is the element at position 2.
                    if (vertex_constraint_ptr->state[0] == next_state_val[0] && vertex_constraint_ptr->state[1] == next_state_val[1] && vertex_constraint_ptr->state[2] == next_state_val[2]) {
                        return false;
                    }
                }
                break;
            }

            case ims::ConstraintType::EDGE: {
                // Convert to an edge constraint pointer to get access to its members.
                auto* edge_constraint_ptr = dynamic_cast<ims::EdgeConstraint*>(constraint_ptr.get());
                if (edge_constraint_ptr != nullptr) {
                    // If the constraint is an edge constraint, check if the state is valid w.r.t the constraint.
                    if (edge_constraint_ptr->state_from[0] == state_val[0] && edge_constraint_ptr->state_from[1] == state_val[1] && edge_constraint_ptr->state_from[2] == state_val[2] && edge_constraint_ptr->state_to[0] == next_state_val[0] && edge_constraint_ptr->state_to[1] == next_state_val[1] && edge_constraint_ptr->state_to[2] == next_state_val[2]) {
                        return false;
                    }
                }
                break;
            }

            case ims::ConstraintType::VERTEX_PRIORITY: {
                // Convert to a vertex constraint pointer to get access to its members.
                auto* vertex_priority_constraint_ptr = dynamic_cast<ims::VertexPriorityConstraint*>(constraint_ptr.get());
                if (vertex_priority_constraint_ptr != nullptr) {
                    // The constraint specifies avoidance of multiple agents. Loop through them and check if the state is valid w.r.t the constraint.
                    for (int other_agent_id : vertex_priority_constraint_ptr->agent_ids_to_avoid) {
                        // Check equality of the constrained time and the current time. If the constrained time is -1, then the constraint is a goal constraint and should be checked only at the goal state.
                        if (vertex_priority_constraint_ptr->time != -1 && vertex_priority_constraint_ptr->time != next_state_val[2]) {
                            continue;
                        }

                        // Get the state of the other agent at the current time step. Get this from the constraints context. If the constraint is "always imposed," there is a need to check that the current timestep is not surpassing the latest timestep of the other agent, and if it does then take the latest state of the other agent.
                        TimeType other_agent_latest_time = (TimeType)constraints_collective_ptr_->getConstraintsContext()->agent_paths.at(other_agent_id).back().back();
                        auto agent_time = (TimeType)next_state_val.back();
                        TimeType other_agent_time = std::min(other_agent_latest_time, agent_time);
                        StateType other_agent_state = constraints_collective_ptr_->getConstraintsContext()->agent_paths.at(other_agent_id).at(other_agent_time);

                        // Check if the state is valid w.r.t the constraint.
                        if (next_state_val[0] == other_agent_state[0] && next_state_val[1] == other_agent_state[1]) {
                            return false;
                        }
                    }
                }
                break;
            }

            case ims::ConstraintType::EDGE_PRIORITY: {
                // Convert to an edge constraint pointer to get access to its members.
                auto* edge_priority_constraint_ptr = dynamic_cast<ims::EdgePriorityConstraint*>(constraint_ptr.get());
                if (edge_priority_constraint_ptr != nullptr) {
                    // The constraint specifies avoidance of multiple agents. Loop through them and check if the state is valid w.r.t the constraint.
                    for (int other_agent_id : edge_priority_constraint_ptr->agent_ids_to_avoid) {
                        // Check equality of the constrained time and the current time. If the constrained time is -1, then the constraint is a goal constraint and should be checked only at the goal state.
                        if (edge_priority_constraint_ptr->t_from != -1 && edge_priority_constraint_ptr->t_to != -1 && (edge_priority_constraint_ptr->t_from != state_val[2] || edge_priority_constraint_ptr->t_to != next_state_val[2])) {
                            continue;
                        }

                        TimeType other_agent_latest_time = (TimeType)constraints_collective_ptr_->getConstraintsContext()->agent_paths.at(other_agent_id).back().back();
                        auto agent_time_from = (TimeType)state_val.back();
                        auto agent_time_to = (TimeType)next_state_val.back();

                        TimeType other_agent_time_from = std::min(other_agent_latest_time, agent_time_from);
                        TimeType other_agent_time_to = std::min(other_agent_latest_time, agent_time_to);

                        // Get the state of the other agent at the timesteps t_from and t_to. Get this from the constraints context.
                        StateType other_agent_state_from = constraints_collective_ptr_->getConstraintsContext()->agent_paths.at(other_agent_id).at(other_agent_time_from);
                        StateType other_agent_state_to = constraints_collective_ptr_->getConstraintsContext()->agent_paths.at(other_agent_id).at(other_agent_time_to);

                        // Check if the state is valid w.r.t the constraint.
                        if (state_val[0] == other_agent_state_to[0] && state_val[1] == other_agent_state_to[1] && next_state_val[0] == other_agent_state_from[0] && next_state_val[1] == other_agent_state_from[1]) {
                            return false;
                        }
                    }
                }    
            } // End of case EDGE_AVOIDANCE.
        } // End of switch.
        return true;
    }
    

    bool isPathValid(const PathType& path) override {
        return std::all_of(path.begin(), path.end(), [this](const StateType& state_val) { return isStateValid(state_val); });
    }

    bool getSuccessors(int curr_state_ind,
                       std::vector<std::vector<int>>& minipath_successors,
                       std::vector<std::vector<double>>& minipath_costs) override{
        ims::RobotState* curr_state = this->getRobotState(curr_state_ind);
        std::vector<MiniPathAction> actions;
        getActions(curr_state_ind, actions, false);
        // Multistep actions are a list of deltas from the origin. So in 2D, an example multistep action is {{1, 0}, {2, 0}, {3, 0}}
        // which means move 1 step in the x direction three times -- moving the robot from (0, 0) to (3, 0).
        for (int i {0} ; i < actions.size() ; i++){
            // Apply the action to the current state.
            std::vector<double> minipath_cost = action_type_->action_costs[i];
            PathType minipath_successor;
            transformStateWithMultiStepAction(curr_state->state, actions[i], minipath_successor);

            // Check if the successor is valid.
            if (isPathValid(minipath_successor)){
                std::vector<int> minipath_successor_state_inds = getOrCreateRobotStates(minipath_successor);
                minipath_successors.push_back(minipath_successor_state_inds);
                minipath_costs.push_back(minipath_cost);
            }
        }
        return true;
    }

    // Get successors with subcosts. The subcosts are the number of conflicts that would be created on a transition to the successor.
    bool getSuccessors(int curr_state_ind,
                       std::vector<std::vector<int>>& minipath_successors,
                       std::vector<std::vector<double>>& minipath_costs,
                       std::vector<std::vector<double>>& minipath_subcosts) override {

        auto curr_state = this->getRobotState(curr_state_ind);
        
        std::vector<MiniPathAction> actions;
        getActions(curr_state_ind, actions, false);
        for (int i {0} ; i < actions.size() ; i++){
            // Flag for whether this action is valid.
            bool is_action_valid = true;
            // Apply the action to the current state.
            std::vector<int> minipath_successor_state_inds;
            std::vector<double> minipath_cost = action_type_->action_costs[i];
            std::vector<double> minipath_subcost;
            PathType minipath_successor_state_vals;
            transformStateWithMultiStepAction(curr_state->state, actions[i], minipath_successor_state_vals);

            // Check for constraint satisfaction and collisions. This checks all transitions in the successor minipath.
            for (int step_ix{0} ; step_ix < minipath_successor_state_vals.size(); step_ix++){
                int step_ix_from = step_ix - 1; // The first step is the current state.
                int step_ix_to = step_ix;
                auto state_val_from = step_ix_from < 0 ? curr_state->state : minipath_successor_state_vals[step_ix_from];
                auto state_val_to = minipath_successor_state_vals[step_ix_to];
                if (isSatisfyingAllConstraints(state_val_from, state_val_to) &&
                    isStateValid(state_val_to)){
                    int state_to_ind = getOrCreateRobotState(state_val_to);
                    minipath_successor_state_inds.push_back(state_to_ind);
                    int num_conflicts = 0;
                    computeTransitionNumberConflicts(state_val_from, state_val_to, num_conflicts);
                    minipath_subcost.push_back((double)num_conflicts);
                }
                else {
                    // Failed to satisfy constraints -- this action is invalid.
                    is_action_valid = false;
                    break;
                }
            }
            if (is_action_valid){
                minipath_successors.push_back(minipath_successor_state_inds);
                minipath_costs.push_back(minipath_cost);
                minipath_subcosts.push_back(minipath_subcost);
            }
        }
        return true;
    }
    
    void computeTransitionNumberConflicts(const StateType& state, const StateType& next_state_val, int & num_conflicts){
        for (const auto& other_agent_id_and_path : constraints_collective_ptr_->getConstraintsContext()->agent_paths) {
            int other_agent_id = other_agent_id_and_path.first;
            PathType other_agent_path = other_agent_id_and_path.second;

            // Get the state of the other agent at the current time step. Get this from the constraints context.
            // The check here is for vertex conflicts.
            if (other_agent_path.empty()) {
                continue;
            }
            auto other_agent_last_time = (TimeType)other_agent_path.back().back();
            auto agent_time = (TimeType)next_state_val.back();
            TimeType other_agent_time = std::min(other_agent_last_time, agent_time);
            StateType other_agent_state = other_agent_path.at(other_agent_time);

            // Check if the state is valid w.r.t the constraint.
            if (next_state_val[0] == other_agent_state[0] && next_state_val[1] == other_agent_state[1]) {
                num_conflicts++;
            }
        }
    }

    int getOrCreateRobotState(const StateType& state_val) override {
        // check if the state exists
        auto* curr_state = new ims::RobotState;

        // Since we are dealing with time, we should take care to first check if this state is a goal state. That would be the case if, when setting the time for this state to be -1, the state already exists. By convention goal states have a time of -1. In this instance, at least.
        // Only do this if the time of the state is later than the last constraint.
        int state_time = (int)state_val.back();
        int last_constraint_time = constraints_collective_ptr_->getLastConstraintTime();

        // TODO(yoraish): currently this method runs two "find"s to check if (a) the passed state is already seen and if the current state is a goal state. The first check is done by finding the state in the state_to_id_ object, and the second by setting the last element of the state to -1 (convention for states) and searching the state_to_id_ object again. This is inefficient and it would be better to check for the state only within the goal states. Unfortunately, the goals are noo known to the action space.
        // Only check if this state is a goal state if there are no more outstanding constraints later in time.
        if (state_time >= last_constraint_time) {
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

    void getPathsConflicts(std::shared_ptr<ims::MultiAgentPaths> paths,
                           std::vector<std::shared_ptr<ims::Conflict>>& conflicts_ptrs,
                           const std::vector<ims::ConflictType>& conflict_types, int max_conflicts,
                           const std::vector<std::string> & names,
                           TimeType time_start, TimeType time_end) override {
        // Loop through the paths and check for conflicts.
        // If requested, get all the conflicts available.
        if (max_conflicts == -1) {
            max_conflicts = INF_INT;
        }

        // Length of the longest path.
        int max_path_length = 0;
        for (auto& path : *paths) {
            if (path.second.size() > max_path_length) {
                max_path_length = (int)path.second.size();
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
                        std::shared_ptr<ims::VertexConflict> conflict_ptr = std::make_shared<ims::VertexConflict>(std::vector<StateType>{conflict_state, conflict_state}, std::vector<int>{i, j});
                        conflicts_ptrs.push_back(conflict_ptr);

                        if (conflicts_ptrs.size() >= max_conflicts) {
                            return;
                        }
                    }

                    // Check if the two paths are in an edge conflict. The first check if for t being the at least before the last time step of the path, in which case there is no edge conflict.
                    if (t < paths->at(i).size() - 1 && t < paths->at(j).size() - 1) {
                        if (paths->at(i)[t][0] == paths->at(j)[t + 1][0] && paths->at(i)[t][1] == paths->at(j)[t + 1][1] && paths->at(i)[t + 1][0] == paths->at(j)[t][0] && paths->at(i)[t + 1][1] == paths->at(j)[t][1]) {
                            // If they are, then add a conflict to the vector.
                            int agent_id_from = i;
                            StateType conflict_state_i_from = {paths->at(i)[t][0], paths->at(i)[t][1], (double)t};
                            StateType conflict_state_i_to = {paths->at(i)[t + 1][0], paths->at(i)[t + 1][1], (double)(t + 1)};

                            int agent_id_to = j;
                            StateType conflict_state_j_from = {paths->at(j)[t][0], paths->at(j)[t][1], (double)t};
                            StateType conflict_state_j_to = {paths->at(j)[t + 1][0], paths->at(j)[t + 1][1], (double)(t + 1)};

                            std::vector<StateType> states_from = {conflict_state_i_from, conflict_state_j_from};
                            std::vector<StateType> states_to = {conflict_state_i_to, conflict_state_j_to};
                            std::vector<int> agent_ids = {agent_id_from, agent_id_to};

                            std::shared_ptr<ims::EdgeConflict> conflict = std::make_shared<ims::EdgeConflict>(states_from, states_to, agent_ids);
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

    void getTransitionSubcost(const StateType& state_val_from, const StateType& state_val_to, double & subcost) override {
        // Compute the number of conflicts that would be created on a transition to the successor.
        // Loop through the paths of all the other agents and check if the transition to the successor creates a conflict.
        int num_conflicts = 0;
        computeTransitionNumberConflicts(state_val_from, state_val_to, num_conflicts);

        // Set the subcost.
        subcost = num_conflicts;
    }

};

#endif  // SEARCH_ACTIONSCENE2DROB_HPP
