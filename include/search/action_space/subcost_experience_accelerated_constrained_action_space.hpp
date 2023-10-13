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
 * \file   subcost_constrained_action_space.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   Oct 05 2023
 */
#pragma once

// standard includes
#include <functional>
#include <algorithm>
#include <utility>

// Project includes.
#include "experience_accelerated_constrained_action_space.hpp"
#include "subcost_action_space.hpp"

namespace ims {

/// @brief Base class for ActionSpaces with constraints.
/// @details This is an actions space extended to be "Constrainable" using a mixin.
class SubcostExperienceAcceleratedConstrainedActionSpace : public ExperienceAcceleratedConstrainedActionSpace{//, public SubcostActionSpace{ 
public:
    /// @brief Constructor
    explicit SubcostExperienceAcceleratedConstrainedActionSpace(): ExperienceAcceleratedConstrainedActionSpace(){};//, SubcostActionSpace() {};

    /// @brief Destructor
    ~SubcostExperienceAcceleratedConstrainedActionSpace() = default;

    /// @brief Get the cost incurred by conflicts upon a give state  transition.
    /// @param state_val 
    /// @param next_state_val 
    /// @param conflicts_cost 
    virtual void computeTransitionConflictsCost(const StateType& state_val, const StateType& next_state_val, double & conflicts_cost) = 0;

    /// @brief To comply with the ExperienceAcceleratedActionSpace interface.
    /// @param state_id The state to get the valid experience subpaths for.
    /// @param subpaths The vector of subpaths -- to be updated with the subpaths.
    inline void getValidExperienceSubpathsFromState(int state_id, std::vector<std::vector<int>>& subpaths, std::vector<std::vector<double>>& subpath_transition_costs,  std::vector<std::vector<double>>& subpath_transition_subcosts) {
        // Get the state configuration that corresponds to the state id.
        auto query_robot_state = states_[state_id];
        
        // Get the state without time.
        StateType state_val_wo_time = {query_robot_state->state.begin(), query_robot_state->state.end() - 1};

        // Get all the experiences that this state is part of. The subexperiences are suffixes of stored experiences, all stored with the query state at the start.
        std::vector<PathType> subexperiences;
        std::vector<std::vector<double>> subexperiences_transition_costs;

        experiences_collective_ptr_->getSubExperiencesFromState(state_val_wo_time, subexperiences, subexperiences_transition_costs); // SLOW AND SLOWING.

        // Retime the subexperiences to start at the time of the query state.
        TimeType query_state_time = query_robot_state->state.back();
        for (auto& subexperience : subexperiences) {
            for (int i = 0; i < subexperience.size(); i++) {
                StateType& state = subexperience[i];
                state.push_back(query_state_time + i); // NOTE(yoraish): assumes integer time increments.
            }
        }
        
        // Get a valid prefix for each of the subexperiences.
        for (int i = 0; i < subexperiences.size(); i++) {
            // Get the subexperience.
            PathType& subexperience = subexperiences[i];
            std::vector<double>& subexperience_transition_costs = subexperiences_transition_costs[i];

            // Get the prefix.
            std::vector<StateType> valid_states_for_reuse;
            std::vector<int> valid_state_ids_for_reuse;
            std::vector<double> valid_states_for_reuse_costs;
            std::vector<double> valid_states_for_reuse_subcosts;
            
            for (int state_ix = 0; state_ix < subexperience.size() - 1; state_ix++) {
                // Get the state.
                StateType robot_state = subexperience[state_ix];
                StateType next_robot_state = subexperience[state_ix + 1];

                // Check if the state is valid w.r.t constraints.
                if (isSatisfyingConstraints(robot_state, next_robot_state)) {
                    valid_states_for_reuse.push_back(robot_state);

                    // Create a state_id if one not already exists.
                    int state_id = getOrCreateRobotStateNonGoal(robot_state); // This is also SLOW apparently.
                    valid_state_ids_for_reuse.push_back(state_id);

                    // Keep track of the transition costs.
                    valid_states_for_reuse_costs.push_back(subexperiences_transition_costs[i][state_ix]);

                    // Compute the transition subcosts, as induced by the conflicts that the transition would cause.
                    double transition_conflict_cost = 0;
                    computeTransitionConflictsCost(robot_state, next_robot_state, transition_conflict_cost);
                    valid_states_for_reuse_subcosts.push_back(transition_conflict_cost);

                } else {
                    // If not, break and add the data to our returned objects.
                    break;
                }
            }

            // If we got here, then we are done processing the experience. We found a prefix that is valid w.r.t constraints. (Could be empty, partial, or full.)        
            
            // The last transition cost is zero.
            if (!valid_states_for_reuse_costs.empty()) {
                valid_states_for_reuse_costs.back() = 0.0;
                valid_states_for_reuse_subcosts.back() = 0.0;
            }

            if (valid_state_ids_for_reuse.empty()) {
                // If the prefix is empty, then we should not add it to the open list.
                continue;
            }

            subpaths.push_back(valid_state_ids_for_reuse);
            subpath_transition_costs.push_back(valid_states_for_reuse_costs);
            subpath_transition_subcosts.push_back(valid_states_for_reuse_subcosts);

        }
    }



};

}  // namespace ims

