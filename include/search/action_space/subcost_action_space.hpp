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
 * \file   subcost_action_space.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   July 10 2023
 */
#pragma once

// standard includes
#include <functional>
#include <algorithm>
#include <utility>

// project includes
#include "action_space.hpp"
#include "search/heuristics/base_heuristic.hpp"
#include "search/planners/planner.hpp"
#include "mixin/action_space_subcost_mixin.hpp"

// For the constrained action space.
#include "search/common/constraints.hpp"
#include "search/common/conflicts.hpp"
#include "constrained_action_space.hpp"

// For experience accelerated constrained action space.
#include "search/common/experiences.hpp"
#include <search/action_space/experience_accelerated_action_space.hpp>
namespace ims {

/// @brief Base class for ActionSpaces with a subcost. This "subcost" is an additional cost of an edge transition. As an example, in ECBS, this is the number of conflicts that would be created with other agents when traversing this edge.
/// @details This is an actions space extended to be "Constrainable" using a mixin.
class SubcostActionSpace : virtual public ActionSpace, public ActionSpaceSubcostMixin{
public:
    /// @brief Constructor
    explicit SubcostActionSpace(): ActionSpace(), ActionSpaceSubcostMixin() {
    }

    /// @brief Destructor
    ~SubcostActionSpace() = default;

    // Make both the base and the overloaded versions of getSuccessors available.
    using ActionSpaceSubcostMixin::getSuccessors;
    using ActionSpace::getSuccessors;
};

/// @brief Base class for ActionSpaces with constraints.
/// @details This is an actions space extended to be "Constrainable" using a mixin.
class SubcostConstrainedActionSpace : public ConstrainedActionSpace, public SubcostActionSpace{
public:
    /// @brief Constructor
    explicit SubcostConstrainedActionSpace(): ConstrainedActionSpace(), SubcostActionSpace() {};

    /// @brief Destructor
    ~SubcostConstrainedActionSpace() = default;
};


/// @brief Base class for ActionSpaces with constraints.
/// @details This is an actions space extended to be "Constrainable" using a mixin.
class SubcostExperienceAcceleratedConstrainedActionSpace : virtual public SubcostConstrainedActionSpace, public ExperienceAcceleratedActionSpace {
public:
    /// @brief Constructor.
    explicit SubcostExperienceAcceleratedConstrainedActionSpace(): SubcostConstrainedActionSpace(), ExperienceAcceleratedActionSpace() {
        std::cout << "SubcostExperienceAcceleratedConstrainedActionSpace: Constructor" << std::endl;
        experiences_collective_ptr_ = std::make_shared<ExperiencesCollective>();
    }

    /// @brief Destructor
    ~SubcostExperienceAcceleratedConstrainedActionSpace() = default;

    bool isSatisfyingConstraint(const StateType &state_val, const StateType &next_state_val, const std::shared_ptr<Constraint> &constraint_ptr) override = 0;

    /// @brief Get the cost incurred by conflicts upon a give state  transition.
    /// @param state_val
    /// @param next_state_val
    /// @param conflicts_cost
    virtual void computeTransitionConflictsCost(const StateType& state_val, const StateType& next_state_val, double & conflicts_cost) = 0;

    inline int getOrCreateRobotState(const StateType& state_val) override {
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
        else{
            // If cannot mark as goal, but is a goal, report that.
            StateType state_val_wo_time = {state_val.begin(), state_val.end() - 1};
            state_val_wo_time.push_back(-1);
            curr_state->state = state_val_wo_time;
            auto it = state_to_id_.find(curr_state);
            if (it != state_to_id_.end()) {
                std::cout << "ExperienceAcceleratedConstrainedActionSpace: getOrCreateRobotState: state is a goal, but cannot be marked as such. Time now is " << state_time << " and last constraint time is " << last_constraint_time << std::endl;
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


    /// @brief To comply with the ExperienceAcceleratedActionSpace interface.
    /// @param state_id The state to get the valid experience subpaths for.
    /// @param subpaths The vector of subpaths -- to be updated with the subpaths.
    inline void getValidExperienceSubpathsFromState(int state_id,
                                                    std::vector<std::vector<std::vector<int>>>&    experience_seq_subpaths,
                                                    std::vector<std::vector<std::vector<double>>>& experience_seq_subpaths_transition_costs,
                                                    std::vector<std::vector<std::vector<double>>>& experience_seq_paths_transition_subcosts) {
        // Get the state configuration that corresponds to the state id.
        auto query_robot_state = states_[state_id];

        // Get the state without time.
        StateType state_val_wo_time = {query_robot_state->state.begin(), query_robot_state->state.end() - 1};

        // Get all the experiences that this state is part of. The sub-experiences are suffixes of stored experiences, all stored with the query state at the start.
        std::vector<SeqPathType> subexperiences;
        std::vector<SeqPathTransitionCostsType> subexperiences_transition_costs;

        experiences_collective_ptr_->getSubExperiencesFromState(state_val_wo_time, subexperiences, subexperiences_transition_costs);

        // Retime the subexperiences to start at the time of the query state.
        auto query_state_time = (TimeType)query_robot_state->state.back();
        for (int subexperience_ix {0}; subexperience_ix < subexperiences.size(); subexperience_ix++){
            SeqPathType& subexperience = subexperiences[subexperience_ix];
            SeqPathTransitionCostsType& subexperience_transition_costs = subexperiences_transition_costs[subexperience_ix];
            // The time of the first state in the first sequence is the time of the query state.
            TimeType t_current = query_state_time;
            // Got through the sequences and propagate the time.
            for (int seq_ix{0}; seq_ix < subexperience.size(); seq_ix++){
                PathType& seq = subexperience[seq_ix];
                std::vector<double>& seq_transition_costs = subexperience_transition_costs[seq_ix];
                for (int state_ix{0}; state_ix < seq.size(); state_ix++){
                    // The first state gets the current time.
                    if (state_ix == 0){
                        seq[state_ix].push_back(t_current);
                    }
                    else{
                        // The rest get the current time plus the increment.
                        seq[state_ix].push_back(t_current + subexperience_transition_costs[seq_ix][state_ix - 1]);
                        // Update the current time.
                        t_current = seq[state_ix].back();
                    }
                }
            }
        }

        // Get a valid prefix for each of the subexperiences.
        for (int i = 0; i < subexperiences.size(); i++) {
            // Get the subexperience and costs.
            SeqPathType& subexperience = subexperiences[i];
            SeqPathTransitionCostsType & subexperience_transition_costs = subexperiences_transition_costs[i];

            // Get the prefix. We do this by walking through the sequence paths and adding all sequences that are valid w.r.t constraints.
            SeqPathType seq_path_valid_for_reuse;
            SeqPathTransitionCostsType seq_path_valid_for_reuse_transition_costs;
            SeqPathTransitionCostsType seq_path_valid_for_reuse_transition_subcosts;
            std::vector<std::vector<int>> seq_path_state_ids_valid_for_reuse;
            for (int seq_ix = 0; seq_ix < subexperience.size(); seq_ix++){
                PathType& seq = subexperience[seq_ix];
                std::vector<double>& seq_transition_costs = subexperience_transition_costs[seq_ix];
                std::vector<int> seq_state_ids;
                std::vector<double> seq_transition_subcosts;
                bool is_seq_valid = true;
                for (int state_ix = 0; state_ix < seq.size() - 1; state_ix++){
                    const StateType & robot_state = seq[state_ix];
                    const StateType & next_robot_state = seq[state_ix + 1];
                    if (isSatisfyingAllConstraints(robot_state, next_robot_state)){
                        seq_state_ids.push_back(getOrCreateRobotStateNonGoal(robot_state));
                        double transition_conflict_cost = 0;
                        computeTransitionConflictsCost(robot_state, next_robot_state, transition_conflict_cost);
                        if (transition_conflict_cost > 0){
                            is_seq_valid = false;
                            break;
                        }
                        else{
                            seq_transition_subcosts.push_back(transition_conflict_cost);
                        }
                    }
                    else{
                        is_seq_valid = false;
                        break;
                    }
                }
                if (is_seq_valid && !seq_state_ids.empty()){
                    // Add the last state id to the sequence and a zero subcost cost.
                    seq_state_ids.push_back(getOrCreateRobotStateNonGoal(seq.back()));
                    seq_transition_subcosts.push_back(0.0);

                    seq_path_state_ids_valid_for_reuse.push_back(seq_state_ids);
                    seq_path_valid_for_reuse_transition_costs.push_back(seq_transition_costs);
                    seq_path_valid_for_reuse_transition_subcosts.push_back(seq_transition_subcosts);
                }
                else{
                    break;
                }
            }

            // If we got here, then we are done processing the experience. We found a prefix that is valid w.r.t constraints. (Could be empty, partial, or full.)
            if (seq_path_state_ids_valid_for_reuse.empty()) {
                // If the prefix is empty, then we should not add it to the open list.
                continue;
            }

            experience_seq_subpaths.push_back(seq_path_state_ids_valid_for_reuse);
            experience_seq_subpaths_transition_costs.push_back(seq_path_valid_for_reuse_transition_costs);
            experience_seq_paths_transition_subcosts.push_back(seq_path_valid_for_reuse_transition_subcosts);
        }
    }


    /// @brief To comply with the ExperienceAcceleratedActionSpace interface.
    /// @param state_id The state to get the valid experience subpaths for.
    /// @param subpaths The vector of subpaths -- to be updated with the subpaths.
    inline void getValidExperienceSubpathsFromState(int state_id,
                                             std::vector<std::vector<std::vector<int>>>& experience_seq_subpaths,
                                             std::vector<std::vector<std::vector<double>>>& experience_seq_subpaths_transition_costs) override {
        throw std::runtime_error("getValidExperienceSubpathsFromState not implemented for SubcostExperienceAcceleratedConstrainedActionSpace");
//        // Get the state configuration that corresponds to the state id.
//        auto query_robot_state = states_[state_id];
//
//        // Get the state without time.
//        StateType state_val_wo_time = {query_robot_state->state.begin(), query_robot_state->state.end() - 1};
//
//        // Get all the experiences that this state is part of. The subexperiences are suffixes of stored experiences, all stored with the query state at the start.
//        std::vector<PathType> subexperiences;
//        std::vector<std::vector<double>> subexperiences_transition_costs;
//
//        experiences_collective_ptr_->getSubExperiencesFromState(state_val_wo_time, subexperiences, subexperiences_transition_costs);
//
//        // Retime the subexperiences to start at the time of the query state.
//        auto query_state_time = (TimeType)query_robot_state->state.back();
//        for (auto& subexperience : subexperiences) {
//            for (int i = 0; i < subexperience.size(); i++) {
//                StateType& state = subexperience[i];
//                state.push_back(query_state_time + i); // NOTE(yoraish): assumes integer time increments.
//            }
//        }
//
//
//        // Get a valid prefix for each of the subexperiences.
//        for (int i = 0; i < subexperiences.size(); i++) {
//            // Get the subexperience.
//            PathType& subexperience = subexperiences[i];
//            std::vector<double>& subexperience_transition_costs = subexperiences_transition_costs[i];
//
//            // Get the prefix.
//            std::vector<StateType> valid_states_for_reuse;
//            std::vector<int> valid_state_ids_for_reuse;
//            std::vector<double> valid_states_for_reuse_costs;
//
//            for (int state_ix = 0; state_ix < subexperience.size() - 1; state_ix++) {
//                // Get the state.
//                StateType robot_state = subexperience[state_ix];
//                StateType next_robot_state = subexperience[state_ix + 1];
//
//                // Check if the state is valid w.r.t constraints.
//                if (isSatisfyingAllConstraints(robot_state, next_robot_state)) {
//                    valid_states_for_reuse.push_back(robot_state);
//
//                    // Create a state_id if one not already exists.
//                    int valid_state_id = getOrCreateRobotStateNonGoal(robot_state);
//                    valid_state_ids_for_reuse.push_back(valid_state_id);
//
//                    // Keep track of the transition costs.
//                    valid_states_for_reuse_costs.push_back(subexperience_transition_costs[state_ix]);
//
//                } else {
//                    // If not, break and add the data to our returned objects.
//                    break;
//                }
//            }
//
//            // If we got here, then we are done processing the experience. We found a prefix that is valid w.r.t constraints. (Could be empty, partial, or full.)
//
//            // The last transition cost is zero.
//            if (!valid_states_for_reuse_costs.empty()) {
//                valid_states_for_reuse_costs.back() = 0.0;
//            }
//
//            if (valid_state_ids_for_reuse.empty()) {
//                // If the prefix is empty, then we should not add it to the open list.
//                continue;
//            }
//
//            subpaths.push_back(valid_state_ids_for_reuse);
//            subpath_transition_costs.push_back(valid_states_for_reuse_costs);
//        }
    }

    [[deprecated("Use the new getSuccessorsExperienceAccelerated (for sequences) instead.")]]
    virtual bool getSuccessorsExperienceAccelerated(int curr_state_ind,
                                std::vector<int> &successors,
                                std::vector<double>& costs,
                                std::vector<double>& subcosts) {
        std::vector<std::vector<int>> seqs_state_ids;
        std::vector<std::vector<double>> seqs_transition_costs;
        std::vector<std::vector<double>> seqs_transition_subcosts;
        bool success = getSuccessorsExperienceAccelerated(curr_state_ind, seqs_state_ids, seqs_transition_costs, seqs_transition_subcosts);
        if (success) {
            for (int i = 0; i < seqs_state_ids.size(); i++) {
                if (seqs_state_ids[i].size() != 2) {
                    std::cout << RED << "SubcostExperienceAcceleratedConstrainedActionSpace: getSuccessorsExperienceAccelerated: Expected 2 states for each edge, but got " << seqs_state_ids[i].size() << RESET << std::endl;
                    throw std::runtime_error("SubcostExperienceAcceleratedConstrainedActionSpace: getSuccessorsExperienceAccelerated: Expected 2 states for each edge.");
                }
                successors.push_back(seqs_state_ids[i].back());
                costs.push_back(vectorSum(seqs_transition_costs[i]));
                subcosts.push_back(vectorSum(seqs_transition_subcosts[i]));
            }
        }
        return success;
    }

    virtual bool getSuccessorsExperienceAccelerated(int curr_state_ind,
                            std::vector<std::vector<int>>& seqs_state_ids,
                            std::vector<std::vector<double>> & seqs_transition_costs,
                            std::vector<std::vector<double>> & seqs_transition_subcosts) {
        throw std::runtime_error("getSuccessorsExperienceAccelerated not implemented for abstract SubcostExperienceAcceleratedConstrainedActionSpace");
    }
};

}  // namespace ims
