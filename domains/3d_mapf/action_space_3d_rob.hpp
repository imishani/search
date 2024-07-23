/*
 * Copyright (C) 2023, Yorai Shaoul
 * All rights reserved.
 *
 */
/*!
 * \file   action_space_3d_rob.hpp
 * \author Julius Arolovitch (juliusa@cmu.edu)
 * \date   07.22.2024
 */

#ifndef SEARCH_ACTIONSCENE3DROB_HPP
#define SEARCH_ACTIONSCENE3DROB_HPP

#include "search/action_space/action_space.hpp"
#include <search/common/conflicts.hpp>
#include "search/action_space/subcost_action_space.hpp"
#include <search/planners/multi_agent/cbs.hpp>

#include "scene_interface_3d_rob.hpp"

struct ActionType3dRob : public ims::ActionType {
    ActionType3dRob() : ims::ActionType() {
        this->name = "ActionType3dRob";
        this->num_actions = 6;
        this->action_names = {"N", "E", "S", "W", "U", "D"};
        this->action_costs = {1, 1, 1, 1, 1, 1};
        this->action_deltas = {{0, 1, 0}, {1, 0, 0}, {0, -1, 0}, {-1, 0, 0}, {0, 0, 1}, {0, 0, -1}};
        this->state_discretization_ = {1, 1, 1};
    }

    std::vector<Action> getPrimActions() override {
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

struct ActionType3dRobTimed : public ActionType3dRob {
    ActionType3dRobTimed() : ActionType3dRob() {
        this->name = "ActionType3dRobTimed";
        this->num_actions = 7;
        this->action_names = {"N", "E", "S", "W", "U", "D", "Wait"};
        this->action_costs = {1, 1, 1, 1, 1, 1, 1};
        this->action_deltas = {{0, 1, 0, 1}, {1, 0, 0, 1}, {0, -1, 0, 1}, {-1, 0, 0, 1}, {0, 0, 1, 1}, {0, 0, -1, 1}, {0, 0, 0, 1}};
        this->state_discretization_ = {1, 1, 1, 1};
    }
};

class ConstrainedActionSpace3dRob : public ims::SubcostConstrainedActionSpace {
private:
    std::shared_ptr<Scene3DRob> env_;
    std::shared_ptr<ActionType3dRob> action_type_;

public:
    ConstrainedActionSpace3dRob(const std::shared_ptr<Scene3DRob>& env,
                                const ActionType3dRob& actions_ptr) : ims::SubcostConstrainedActionSpace() {
        this->env_ = env;
        this->action_type_ = std::make_shared<ActionType3dRob>(actions_ptr);
    }

    void getActions(int state_id,
                    std::vector<ActionSequence> &action_seqs,
                    bool check_validity) override {
        auto actions = action_type_->getPrimActions();
        for (int i = 0; i < action_type_->num_actions; i++) {
            auto action = actions[i];
            if (check_validity) {
                auto curr_state = this->getRobotState(state_id);
                auto next_state_val = StateType(curr_state->state.size());
                std::transform(curr_state->state.begin(), curr_state->state.end(), action.begin(), next_state_val.begin(), std::plus<>());
                if (!isStateValid(next_state_val)) {
                    continue;
                }
            }
            ActionSequence action_seq;
            action_seq.push_back(action);
            action_seqs.push_back(action_seq);
        }
    }

    bool isStateValid(const StateType& state_val) override {
        double col = state_val[0];
        double row = state_val[1];
        double z = state_val[2];
        return env_->isCellValid(row, col, z);
    }

    bool isSatisfyingConstraint(const StateType &state_val, const StateType &next_state_val, const std::shared_ptr<ims::Constraint> &constraint_ptr) override {
        switch (constraint_ptr->type) {
            case ims::ConstraintType::VERTEX: {
                auto* vertex_constraint_ptr = dynamic_cast<ims::VertexConstraint*>(constraint_ptr.get());
                if (vertex_constraint_ptr != nullptr) {
                    if (vertex_constraint_ptr->state[0] == next_state_val[0] && 
                        vertex_constraint_ptr->state[1] == next_state_val[1] && 
                        vertex_constraint_ptr->state[2] == next_state_val[2] && 
                        vertex_constraint_ptr->state[3] == next_state_val[3]) {
                        return false;
                    }
                }
                break;
            }

            case ims::ConstraintType::EDGE: {
                auto* edge_constraint_ptr = dynamic_cast<ims::EdgeConstraint*>(constraint_ptr.get());
                if (edge_constraint_ptr != nullptr) {
                    if (edge_constraint_ptr->state_from[0] == state_val[0] && 
                        edge_constraint_ptr->state_from[1] == state_val[1] && 
                        edge_constraint_ptr->state_from[2] == state_val[2] && 
                        edge_constraint_ptr->state_from[3] == state_val[3] && 
                        edge_constraint_ptr->state_to[0] == next_state_val[0] && 
                        edge_constraint_ptr->state_to[1] == next_state_val[1] && 
                        edge_constraint_ptr->state_to[2] == next_state_val[2] && 
                        edge_constraint_ptr->state_to[3] == next_state_val[3]) {
                        return false;
                    }
                }
                break;
            }

            case ims::ConstraintType::VERTEX_PRIORITY: {
                auto* vertex_priority_constraint_ptr = dynamic_cast<ims::VertexPriorityConstraint*>(constraint_ptr.get());
                if (vertex_priority_constraint_ptr != nullptr) {
                    for (int other_agent_id : vertex_priority_constraint_ptr->agent_ids_to_avoid) {
                        if (vertex_priority_constraint_ptr->time != -1 && vertex_priority_constraint_ptr->time != next_state_val[3]) {
                            continue;
                        }

                        TimeType other_agent_latest_time = (TimeType)constraints_collective_ptr_->getConstraintsContext()->agent_paths.at(other_agent_id).back().back();
                        auto agent_time = (TimeType)next_state_val.back();
                        TimeType other_agent_time = std::min(other_agent_latest_time, agent_time);
                        StateType other_agent_state = constraints_collective_ptr_->getConstraintsContext()->agent_paths.at(other_agent_id).at(other_agent_time);

                        if (next_state_val[0] == other_agent_state[0] && 
                            next_state_val[1] == other_agent_state[1] && 
                            next_state_val[2] == other_agent_state[2]) {
                            return false;
                        }
                    }
                }
                break;
            }

            case ims::ConstraintType::EDGE_PRIORITY: {
                auto* edge_priority_constraint_ptr = dynamic_cast<ims::EdgePriorityConstraint*>(constraint_ptr.get());
                if (edge_priority_constraint_ptr != nullptr) {
                    for (int other_agent_id : edge_priority_constraint_ptr->agent_ids_to_avoid) {
                        if (edge_priority_constraint_ptr->t_from != -1 && 
                            edge_priority_constraint_ptr->t_to != -1 && 
                            (edge_priority_constraint_ptr->t_from != state_val[3] || 
                             edge_priority_constraint_ptr->t_to != next_state_val[3])) {
                            continue;
                        }

                        TimeType other_agent_latest_time = (TimeType)constraints_collective_ptr_->getConstraintsContext()->agent_paths.at(other_agent_id).back().back();
                        auto agent_time_from = (TimeType)state_val.back();
                        auto agent_time_to = (TimeType)next_state_val.back();

                        TimeType other_agent_time_from = std::min(other_agent_latest_time, agent_time_from);
                        TimeType other_agent_time_to = std::min(other_agent_latest_time, agent_time_to);

                        StateType other_agent_state_from = constraints_collective_ptr_->getConstraintsContext()->agent_paths.at(other_agent_id).at(other_agent_time_from);
                        StateType other_agent_state_to = constraints_collective_ptr_->getConstraintsContext()->agent_paths.at(other_agent_id).at(other_agent_time_to);

                        if (state_val[0] == other_agent_state_to[0] && 
                            state_val[1] == other_agent_state_to[1] && 
                            state_val[2] == other_agent_state_to[2] && 
                            next_state_val[0] == other_agent_state_from[0] && 
                            next_state_val[1] == other_agent_state_from[1] && 
                            next_state_val[2] == other_agent_state_from[2]) {
                            return false;
                        }
                    }
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
        for (int i = 0; i < actions.size(); i++) {
            auto action = actions[i][0];
            auto next_state_val = StateType(curr_state->state.size());
            std::transform(curr_state->state.begin(), curr_state->state.end(), action.begin(), next_state_val.begin(), std::plus<>());

            if (!isSatisfyingAllConstraints(curr_state->state, next_state_val)) {
                continue;
            }

            if (isStateValid(next_state_val)) {
                int next_state_ind = getOrCreateRobotState(next_state_val);
                successors.push_back(next_state_ind);
                costs.push_back(action_type_->action_costs[i]);
            }
        }
        return true;
    }

    bool getSuccessors(int curr_state_ind,
                       std::vector<int>& successors,
                       std::vector<double>& costs,
                       std::vector<double>& subcosts) override {
        auto curr_state = this->getRobotState(curr_state_ind);
        
        std::vector<ActionSequence> actions;
        getActions(curr_state_ind, actions, false);
        for (int i = 0; i < actions.size(); i++) {
            auto action = actions[i][0];
            auto next_state_val = StateType(curr_state->state.size());
            std::transform(curr_state->state.begin(), curr_state->state.end(), action.begin(), next_state_val.begin(), std::plus<>());

            if (!isSatisfyingAllConstraints(curr_state->state, next_state_val)) {
                continue;
            }

            if (isStateValid(next_state_val)) {
                int next_state_ind = getOrCreateRobotState(next_state_val);
                successors.push_back(next_state_ind);
                costs.push_back(action_type_->action_costs[i]);

                int num_conflicts = 0;
                computeTransitionNumberConflicts(curr_state->state, next_state_val, num_conflicts);

                subcosts.push_back(num_conflicts);
            }
        }
        return true;
    }
    
    void computeTransitionNumberConflicts(const StateType& state, const StateType& next_state_val, int & num_conflicts) {
        for (const auto& other_agent_id_and_path : constraints_collective_ptr_->getConstraintsContext()->agent_paths) {
            int other_agent_id = other_agent_id_and_path.first;
            PathType other_agent_path = other_agent_id_and_path.second;

            if (other_agent_path.empty()) {
                continue;
            }
            auto other_agent_last_time = (TimeType)other_agent_path.back().back();
            auto agent_time = (TimeType)next_state_val.back();
            TimeType other_agent_time = std::min(other_agent_last_time, agent_time);
            StateType other_agent_state = other_agent_path.at(other_agent_time);

            if (next_state_val[0] == other_agent_state[0] && 
                next_state_val[1] == other_agent_state[1] && 
                next_state_val[2] == other_agent_state[2]) {
                num_conflicts++;
            }
        }
    }

    int getOrCreateRobotState(const StateType& state_val) override {
        auto* curr_state = new ims::RobotState;

        int state_time = (int)state_val.back();
        int last_constraint_time = constraints_collective_ptr_->getLastConstraintTime();

        if (state_time >= last_constraint_time) {
            StateType state_val_wo_time = {state_val.begin(), state_val.end() - 1};
            state_val_wo_time.push_back(-1);
            curr_state->state = state_val_wo_time;
            auto it = state_to_id_.find(curr_state);
            if (it != state_to_id_.end()) {
                delete curr_state;
                return it->second;
            }
        }

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

    double computePathCost(const PathType& path) {
        double cost = 0;
        for (int i = 0; i < path.size() - 1; i++) {
            auto curr_state = path[i];
            auto next_state = path[i + 1];
            auto action = action_type_->getPrimActions()[next_state[3]];
            cost += action_type_->action_costs[next_state[3]];
        }
        return cost;
    }

    void getPathsConflicts(std::shared_ptr<ims::MultiAgentPaths> paths,
                           std::vector<std::shared_ptr<ims::Conflict>>& conflicts_ptrs,
                           const std::vector<ims::ConflictType>& conflict_types, int max_conflicts,
                           const std::vector<std::string> & names,
                           TimeType time_start, TimeType time_end) override {
        if (max_conflicts == -1) {
            max_conflicts = INF_INT;
        }

        int max_path_length = 0;
        for (auto& path : *paths) {
            if (path.second.size() > max_path_length) {
                max_path_length = (int)path.second.size();
            }
        }

        for (int t = 0; t < max_path_length; t++) {
            for (int i = 0; i < paths->size(); i++) {
                for (int j = i + 1; j < paths->size(); j++) {
                    int t_i = std::min(t, (int)paths->at(i).size() - 1);
                    int t_j = std::min(t, (int)paths->at(j).size() - 1);

                    // Check for vertex conflicts
                    if (paths->at(i)[t_i][0] == paths->at(j)[t_j][0] && 
                        paths->at(i)[t_i][1] == paths->at(j)[t_j][1] && 
                        paths->at(i)[t_i][2] == paths->at(j)[t_j][2]) {
                        StateType conflict_state = {paths->at(i)[t_i][0], paths->at(i)[t_i][1], paths->at(i)[t_i][2], (double)t};
                        std::shared_ptr<ims::VertexConflict> conflict_ptr = std::make_shared<ims::VertexConflict>(std::vector<StateType>{conflict_state, conflict_state}, std::vector<int>{i, j});
                        conflicts_ptrs.push_back(conflict_ptr);

                        if (conflicts_ptrs.size() >= max_conflicts) {
                            return;
                        }
                    }

                    // Check for edge conflicts
                    if (t < paths->at(i).size() - 1 && t < paths->at(j).size() - 1) {
                        if (paths->at(i)[t][0] == paths->at(j)[t + 1][0] && 
                            paths->at(i)[t][1] == paths->at(j)[t + 1][1] && 
                            paths->at(i)[t][2] == paths->at(j)[t + 1][2] && 
                            paths->at(i)[t + 1][0] == paths->at(j)[t][0] && 
                            paths->at(i)[t + 1][1] == paths->at(j)[t][1] && 
                            paths->at(i)[t + 1][2] == paths->at(j)[t][2]) {
                            int agent_id_from = i;
                            StateType conflict_state_i_from = {paths->at(i)[t][0], paths->at(i)[t][1], paths->at(i)[t][2], (double)t};
                            StateType conflict_state_i_to = {paths->at(i)[t + 1][0], paths->at(i)[t + 1][1], paths->at(i)[t + 1][2], (double)(t + 1)};

                            int agent_id_to = j;
                            StateType conflict_state_j_from = {paths->at(j)[t][0], paths->at(j)[t][1], paths->at(j)[t][2], (double)t};
                            StateType conflict_state_j_to = {paths->at(j)[t + 1][0], paths->at(j)[t + 1][1], paths->at(j)[t + 1][2], (double)(t + 1)};

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
        int num_conflicts = 0;
        computeTransitionNumberConflicts(state_val_from, state_val_to, num_conflicts);
        subcost = num_conflicts;
    }
};

#endif  // SEARCH_ACTIONSCENE3DROB_HPP