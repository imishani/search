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
 * \date   Sept 05 2023
 */

#ifndef SEARCH_ACTIONSCENE2DROBEXPERIENCE_HPP
#define SEARCH_ACTIONSCENE2DROBEXPERIENCE_HPP

#include "search/action_space/action_space.hpp"
#include <search/common/conflicts.hpp>
#include <search/common/constraints.hpp>
#include <search/action_space/subcost_action_space.hpp>
#include <search/action_space/experience_accelerated_constrained_action_space.hpp>
#include <search/common/scene_interface.hpp>
#include <search/planners/multi_agent/cbs.hpp>
#include <search/common/utils.hpp>

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

struct ActionType2dRob : public ims::ActionType {
    ActionType2dRob() : ims::ActionType() {
        this->name = "ActionType2dRob";
        this->num_actions = 4;
        this->action_names = {"N", "E", "S", "W",};
        this->action_costs = {1, 1, 1, 1};
        this->action_deltas = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
        this->state_discretization_ = {1, 1};
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

struct ActionType2dRobTimed : public ActionType2dRob {
    ActionType2dRobTimed() : ActionType2dRob() {
        name = "ActionType2dRobTimed";
        num_actions = 5;
        action_names = {"N", "E", "S", "W", "Wait"};
        action_costs = {1, 1, 1, 1, 1};
        action_deltas = {{0, 1, 1}, {1, 0, 1}, {0, -1, 1}, {-1, 0, 1}, {0, 0, 1}};
        state_discretization_ = {1, 1, 1};
    }
};

class ExperienceAcceleratedConstrainedActionSpace2dRob : public ims::ExperienceAcceleratedConstrainedActionSpace {
private:
    std::shared_ptr<scene2DRob> env_;
    std::shared_ptr<ActionType2dRob> action_type_;

public:
    ExperienceAcceleratedConstrainedActionSpace2dRob(const scene2DRob& env,
                                const ActionType2dRob& actions_ptr) : ims::ExperienceAcceleratedConstrainedActionSpace() {
        this->env_ = std::make_shared<scene2DRob>(env);
        this->action_type_ = std::make_shared<ActionType2dRob>(actions_ptr);
    }

    void getActions(int state_id,
                    std::vector<ActionSequence> &action_seqs,
                    bool check_validity) override {
        auto actions = action_type_->getPrimActions();
        for (int i {0} ; i < action_type_->num_actions ; i++){
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

    /// @brief Whether the state is satisfying a specific constraint.
    /// @param state_val The state value to check. Timed.
    /// @param next_state_val The next state value to check. Timed.
    /// @param constraint_ptr The constraint to check.
    /// @return True if the constraint is satisfied, false otherwise.
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
        }
        return true;
    }


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
            if (!isSatisfyingAllConstraints(curr_state->state, next_state_val)) {
                continue;
            }

            // EXAMPLE NOTE: it may sometimes make sense to check constraint satisfaction within the isStateValid method, for efficiency. For example, if a constraint requires comparison of a robot state against a world state and simulataneously the state of another robot, then it would be better to set all robots into their specified configurations and check for validity only once.
            if (isStateValid(next_state_val)) {
                int next_state_ind = getOrCreateRobotState(next_state_val);
                successors.push_back(next_state_ind);
                costs.push_back(action_type_->action_costs[i]);
            }
        }
        return true;
    }

    /// @brief Compute the cost of a path. This is domain-specific, and must be implemented in multi-agent settings.
    /// @param path
    /// @return The cost of a path.
    double computePathCost(const PathType& path) {
        double cost = 0;
        for (int i{0}; i < path.size() - 1; i++) {
            auto curr_state = path[i];
            auto next_state = path[i + 1];
            auto action = action_type_->getPrimActions()[next_state[2]];
            cost += action_type_->action_costs[next_state[2]];
        }
        return cost;
    }
    // Get successors with subcosts. The subcosts are the number of conflicts that would be created on a transition to the successor.
    bool getSuccessorsExperienceAccelerated(int curr_state_ind,
                       std::vector<int>& successors,
                       std::vector<double>& costs) override {

        return getSuccessors(curr_state_ind, successors, costs);
    }
    
    void getPathsConflicts(std::shared_ptr<ims::MultiAgentPaths> paths,
                           std::vector<std::shared_ptr<ims::Conflict>>& conflicts_ptrs,
                           const std::vector<ims::ConflictType>& conflict_types, int max_conflicts,
                           const std::vector<std::string> & names, TimeType time_start, TimeType time_end) override {
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
};

/// @brief A constrained action space for the 2D mapf domain, with the subcost and experience acceleration extensions.
/// @note This is very similar to the ExperienceAcceleratedConstrainedActionSpace2dRob class, with additional functionality for subcosts. We leave these two classes separate for clarity and for the sake of example.
class SubcostExperienceAcceleratedConstrainedActionSpace2dRob : public ims::SubcostExperienceAcceleratedConstrainedActionSpace {
private:
    std::shared_ptr<scene2DRob> env_;
    std::shared_ptr<ActionType2dRob> action_type_;

public:
    SubcostExperienceAcceleratedConstrainedActionSpace2dRob(const scene2DRob& env,
                                const ActionType2dRob& actions_ptr) : ims::SubcostExperienceAcceleratedConstrainedActionSpace() {
        this->env_ = std::make_shared<scene2DRob>(env);
        this->action_type_ = std::make_shared<ActionType2dRob>(actions_ptr);
    }

    void getActions(int state_id,
                    std::vector<ActionSequence> &action_seqs,
                    bool check_validity) override {
        auto actions = action_type_->getPrimActions();
        for (int i {0} ; i < action_type_->num_actions ; i++){
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

    bool isSatisfyingConstraint(const StateType &state_val, const StateType &next_state_val, const std::shared_ptr<ims::Constraint> &constraint_ptr) override {
        // Loop through the vector, and get a reference to each one of the elements.
        for (auto& constraint_ptr : constraints_collective_ptr_->getConstraints()) {

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
            }
        }
        return true;
    }


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
            if (!isSatisfyingAllConstraints(curr_state->state, next_state_val)) {
                continue;
            }

            // EXAMPLE NOTE: it may sometimes make sense to check constraint satisfaction within the isStateValid method, for efficiency. For example, if a constraint requires comparison of a robot state against a world state and simulataneously the state of another robot, then it would be better to set all robots into their specified configurations and check for validity only once.
            if (isStateValid(next_state_val)) {
                int next_state_ind = getOrCreateRobotState(next_state_val);
                successors.push_back(next_state_ind);
                costs.push_back(action_type_->action_costs[i]);
            }
        }
        return true;
    }

    // Get successors with subcosts. The subcosts are the number of conflicts that would be created on a transition to the successor.
    bool getSuccessors(int curr_state_ind,
                       std::vector<int>& successors,
                       std::vector<double>& costs,
                       std::vector<double>& subcosts) override {

        auto curr_state = this->getRobotState(curr_state_ind);
        
        std::vector<ActionSequence> actions;
        getActions(curr_state_ind, actions, false);
        for (int i {0} ; i < actions.size() ; i++){
            auto action = actions[i][0];
            auto next_state_val = StateType(curr_state->state.size());
            std::transform(curr_state->state.begin(), curr_state->state.end(), action.begin(), next_state_val.begin(), std::plus<>());

            // Check for constraint satisfaction.
            if (!isSatisfyingAllConstraints(curr_state->state, next_state_val)) {
                continue;
            }

            // If the state successor is valid, then compute the number of conflicts that would be created on a transition to the successor and add it to the successors.
            if (isStateValid(next_state_val)) {
                int next_state_ind = getOrCreateRobotState(next_state_val);
                successors.push_back(next_state_ind);
                costs.push_back(action_type_->action_costs[i]);

                // Compute the number of conflicts that would be created on a transition to the successor.
                // Loop through the paths of all the other agents and check if the transition to the successor creates a conflict.
                double num_conflicts = 0;
                computeTransitionConflictsCost(curr_state->state, next_state_val, num_conflicts);

                // Set the subcost.
                subcosts.push_back(num_conflicts);
            }
        }
        return true;
    }

    // Get successors with subcosts. The subcosts are the number of conflicts that would be created on a transition to the successor.
    bool getSuccessorsExperienceAccelerated(int curr_state_ind,
                       std::vector<int>& successors,
                       std::vector<double>& costs,
                       std::vector<double>& subcosts) override {

        return getSuccessors(curr_state_ind, successors, costs, subcosts);
    }

    // Get successors with subcosts. The subcosts are the number of conflicts that would be created on a transition to the successor.
    bool getSuccessorsExperienceAccelerated(int curr_state_ind,
                       std::vector<int>& successors,
                       std::vector<double>& costs) override {

        return getSuccessors(curr_state_ind, successors, costs);
    }


    void computeTransitionConflictsCost(const StateType& state, const StateType& next_state_val, double & num_conflicts) override {
        for (auto other_agent_id_and_path : constraints_collective_ptr_->getConstraintsContext()->agent_paths) {
            int other_agent_id = other_agent_id_and_path.first;
            PathType other_agent_path = other_agent_id_and_path.second;

            // Get the state of the other agent at the current time step. Get this from the constraints context.
            // The check here is for vertex conflicts.
            if (other_agent_path.empty()) {
                continue;
            }
            TimeType other_agent_last_time = other_agent_path.back().back();
            TimeType agent_time = next_state_val.back();
            TimeType other_agent_time = std::min(other_agent_last_time, agent_time);
            StateType other_agent_state = other_agent_path.at(other_agent_time);

            // Check if the state is valid w.r.t the constraint.
            if (next_state_val[0] == other_agent_state[0] && next_state_val[1] == other_agent_state[1]) {
                num_conflicts++;
            }
        }
    }


    /// @brief Compute the cost of a path. This is domain-specific, and must be implemented in multi-agent settings.
    /// @param path
    /// @return The cost of a path.
    double computePathCost(const PathType& path) {
        double cost = 0;
        for (int i{0}; i < path.size() - 1; i++) {
            auto curr_state = path[i];
            auto next_state = path[i + 1];
            auto action = action_type_->getPrimActions()[next_state[2]];
            cost += action_type_->action_costs[next_state[2]];
        }
        return cost;
    }

    void getPathsConflicts(std::shared_ptr<ims::MultiAgentPaths> paths,
                           std::vector<std::shared_ptr<ims::Conflict>>& conflicts_ptrs,
                           const std::vector<ims::ConflictType>& conflict_types,
                           int max_conflicts, const std::vector<std::string> & names,
                           TimeType time_start = 0,
                           TimeType time_end = -1) {
        // Loop through the paths and check for conflicts.
        // If requested, get all the conflicts available.
        if (max_conflicts == -1) {
            max_conflicts = INF_INT;
        }

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

    void getSafeIntervals(int state_id, std::vector<SafeIntervalType>& safe_intervals) override{
        std::cout << "ConstrainedActionSpace2dRob: getSafeIntervals" << std::endl;
    }

    void getTransitionSubcost(const StateType& state_val_from, const StateType& state_val_to, double & subcost) override {
        // Compute the number of conflicts that would be created on a transition to the successor.
        // Loop through the paths of all the other agents and check if the transition to the successor creates a conflict.
        double num_conflicts = 0;
        computeTransitionConflictsCost(state_val_from, state_val_to, num_conflicts);

        // Set the subcost.
        subcost = num_conflicts;
    }

};

#endif  // SEARCH_ACTIONSCENE2DROBEXPERIENCE_HPP
