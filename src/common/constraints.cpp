/*
 * Copyright (C) 2024, Yorai Shaoul
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
 * \file   constraints.cpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   2024-01-19
*/


// Project includes.
#include <search/common/constraints.hpp>
#include <search/action_space/constrained_action_space.hpp>

namespace ims {

// Setters.
/// @brief Set the constraints.
/// @param constraints The constraints to set.
void ConstraintsCollective::setConstraints(const std::vector<std::shared_ptr<Constraint>>& constraints) {
    constraints_ptrs_.clear();
    time_to_constraints_ptrs_.clear();
    last_constraint_time_ = -1;

    // Set the constraints and their counts after.
    constraints_ptrs_ = constraints;

    // Set the time_to_constraints_ptrs_ map.
    for (const auto& constraint_ptr : constraints_ptrs_) {
        for (int t = constraint_ptr->getTimeInterval().first; t <= constraint_ptr->getTimeInterval().second; t++) {
            time_to_constraints_ptrs_[t].push_back(constraint_ptr);
            
            // Update the latest time that has constraints, if necessary.
            last_constraint_time_ = std::max(last_constraint_time_, t);
        }
    }
}

void ConstraintsCollective::addConstraint(const std::shared_ptr<Constraint>& constraint) {
    constraints_ptrs_.push_back(constraint);

    // Add to the time_to_constraints_ptrs_ map. This is only done if the time interval is not (-1, -1).
    TimeType t_from = constraint->getTimeInterval().first;
    TimeType t_to = constraint->getTimeInterval().second;
    // TODO(yoraish): incorporate the action discretization into this for loop. This comes from the actionspace, so will have to think how to incorporate it. Currently the assumption is that the time is moving in integer steps.
    if (t_from == -1 && t_to == -1) {
        return;
    }

    // TODO(yoraish): Check if there are any existing constraints of the same type and at the same interval. If so, then we update them instead of adding a new constraint.
    // for (const auto& existing_constraint_ptr : time_to_constraints_ptrs_[t_from]) {
    //     if (existing_constraint_ptr->type == constraint->type && existing_constraint_ptr->getTimeInterval() == constraint->getTimeInterval()) {
    //         existing_constraint_ptr.update(constraint);
    //     }
    // }

    for (TimeType t = t_from; t <= t_to; t++) {
        time_to_constraints_ptrs_[t].push_back(constraint);

        // Update the latest time that has constraints, if necessary.
        last_constraint_time_ = std::max(last_constraint_time_, t);
    }

    // Update the existing safe intervals with the new constraint.
    updateSafeIntervals(constraint);
}

/// @brief Set the last time that has constraints. This time is often used to check if a planner is allowed to terminate (find a goal). If there are outstanding constraints, then the planner should not terminate. If we impose constraints that are "always active," then we need a way to tell planners "after this time t, keep satisfying the "always" constraints but you are allowed to find the goal." This is one way to do this.
void ConstraintsCollective::setLastConstraintTimeToAtLeast(TimeType t) {
    last_constraint_time_ = std::max(last_constraint_time_, t);
}

void ConstraintsCollective::addConstraints(const std::vector<std::shared_ptr<Constraint>>& constraints) {
    for (const auto& constraint_ptr : constraints) {
        addConstraint(constraint_ptr);
    }
}

/// @brief Set the constraints context.
/// @param context The constraints context to set.
void ConstraintsCollective::setContext(const std::shared_ptr<ConstraintsContext>& context) {
    context_ptr_ = context;
}

/// @brief Clear the constraints context.
void ConstraintsCollective::clearContext() {
    context_ptr_.reset();
}

/// @brief Clear the constraints.
void ConstraintsCollective::clear() {
    constraints_ptrs_.clear();
    time_to_constraints_ptrs_.clear();
    last_constraint_time_ = -1;   
}

// Getters.
/// @brief Get the context pointer.
/// @return The context pointer.
std::shared_ptr<ConstraintsContext> ConstraintsCollective::getConstraintsContext() const {
    return context_ptr_;
}
/// @brief Get the context pointer in a non-const way.
/// @return The context pointer.
std::shared_ptr<ConstraintsContext> ConstraintsCollective::getConstraintsContextNonConst() {
    return context_ptr_;
}

/// @brief Get the constraints.
/// @return The constraints.
const std::vector<std::shared_ptr<Constraint>>& ConstraintsCollective::getConstraints() const {
    return constraints_ptrs_;
}

/// String.
/// @brief Get a string with information about the constraint.
std::string ConstraintsCollective::toString() const {
    std::string str = "ConstraintsCollective: \n";
    for (const auto& constraint_ptr : constraints_ptrs_) {
        str += "    " + constraint_ptr->toString() + "\n";
    }
    return str;
}

/// @brief Get the greatest time that is constrained.
/// @return The greatest time that is constrained.
int ConstraintsCollective::getLastConstraintTime() const {
    return last_constraint_time_;
}

/// @brief Create safe intervals for a configuration state.
/// @param state The state configuration without a time component.
void ConstraintsCollective::createSafeIntervals(const StateType & state){

    int num_states_in_map_before = (int)state_to_safe_intervals_.size();

    // Get the initial safe intervals. This is an empty vector.
    std::vector<SafeIntervalType> safe_intervals = {};
    // Create the initial unsafe intervals. This will be populated with the constrained time intervals.
    std::vector<SafeIntervalType> unsafe_intervals = {};

    // Iterate over constraints and split the safe interval every time a constraint is imposed on this state.
    for (const auto& constraint_ptr : constraints_ptrs_) {
        // Get the time interval of the constraint.
        std::pair<TimeType, TimeType> constraint_time_interval = constraint_ptr->getTimeInterval();
        StateType state_from = state;
        state_from.push_back(constraint_time_interval.first);
        StateType state_to = state;
        state_to.push_back(constraint_time_interval.second);

        // Ask the action space to determine if the state is valid w.r.t. the constraint.
        if (!context_ptr_->action_space_ptr->isSatisfyingConstraint(state_from, state_to, constraint_ptr)) {
            // If the state is valid, continue. Otherwise, split the relevant safe intervals.
            unsafe_intervals.push_back(constraint_time_interval);
        }
    }

    // Sort the unsafe intervals according to their start time.
    std::sort(unsafe_intervals.begin(), unsafe_intervals.end(), [](const SafeIntervalType& a, const SafeIntervalType& b) {
        return a.first < b.first;
    });

    // Merge unsafe intervals that overlap.
    if (!unsafe_intervals.empty()) {
        std::vector<SafeIntervalType> merged_unsafe_intervals = {unsafe_intervals[0]};
        for (int i = 1; i < unsafe_intervals.size(); i++) {
            if (merged_unsafe_intervals.back().second >= unsafe_intervals[i].first) {
                merged_unsafe_intervals.back().second = std::max(merged_unsafe_intervals.back().second, unsafe_intervals[i].second);
            } else {
                merged_unsafe_intervals.push_back(unsafe_intervals[i]);
            }
        }

        // Update the unsafe intervals.
        unsafe_intervals = merged_unsafe_intervals;
    }

    // Create the safe intervals. 
    // If there are no unsafe intervals, then the safe interval is zero to infinity.
    if (unsafe_intervals.empty()) {
        safe_intervals.emplace_back(0, INF_TIME_TYPE);
    }
    else{
        // If there are unsafe intervals, then the safe intervals are the intervals between the unsafe intervals. Starting from time zero.
        TimeType first_safe_time = 0;
        for (const auto& unsafe_interval : unsafe_intervals) {
            safe_intervals.emplace_back(first_safe_time, unsafe_interval.first - 1);
            first_safe_time = unsafe_interval.second + 1;
        }
        // Add the last safe interval.
        safe_intervals.emplace_back(first_safe_time, INF_TIME_TYPE);
    }

    // Add the safe intervals to the map.
    state_to_safe_intervals_[state] = safe_intervals;

    int num_states_in_map_after = (int)state_to_safe_intervals_.size();
    if (num_states_in_map_after <= num_states_in_map_before) {
        throw std::runtime_error("The number of states in the map did not increase.");
    }
}

/// @brief Get the safe intervals for a given state. If those do not exist then create them.
/// @param state The state configuration. Should not include a time component.
/// @return The safe intervals.
void ConstraintsCollective::getOrCreateSafeIntervals(const StateType& state, std::vector<SafeIntervalType>& safe_intervals) {
    // If the state is not in the map, then create the safe intervals.
    if (state_to_safe_intervals_.find(state) == state_to_safe_intervals_.end()) {
        createSafeIntervals(state);
    }

    // Return the safe intervals.
    safe_intervals = state_to_safe_intervals_[state];
}

/// @brief Update the currently existing safe intervals with a new constraint.
/// @param constraint_ptr 
void ConstraintsCollective::updateSafeIntervals(const std::shared_ptr<Constraint>& constraint_ptr) {
    // Get the time interval of the constraint.
    std::pair<TimeType, TimeType> constraint_time_interval = constraint_ptr->getTimeInterval();

    // Iterate over the states configurations (that already have safe intervals) and update their safe intervals.
    for (auto& state_to_safe_intervals_pair : state_to_safe_intervals_) {
        // Get the state.
        const StateType& state = state_to_safe_intervals_pair.first;
        // Get the safe intervals.
        std::vector<SafeIntervalType>& safe_intervals = state_to_safe_intervals_pair.second;
        // Create a vector of updated safe intervals. This will eventually take the place of the current safe intervals.
        std::vector<SafeIntervalType> updated_safe_intervals = {};

        // Iterate over the safe intervals and update them without modifying the original vector.
        for (const auto& safe_interval : safe_intervals) {
            // Check if the safe interval has any overlap with the constraint time interval.
            if (!(safe_interval.first > constraint_time_interval.second || safe_interval.second < constraint_time_interval.first)) {
                // Check if this new constraint invalidates this configuration.
                StateType state_from = state;
                state_from.push_back(constraint_time_interval.first);
                StateType state_to = state;
                state_to.push_back(constraint_time_interval.second);

                // Ask the action space to determine if the state is valid w.r.t. the constraint.
                if (context_ptr_->action_space_ptr->isSatisfyingConstraint(state_from, state_to, constraint_ptr)) {
                    // Include the safe interval in the updated vector.
                    updated_safe_intervals.push_back(safe_interval);
                } else {
                    // If the start and end times of the constraint are equal, then this timestep is forbidden.
                    if (constraint_time_interval.first == constraint_time_interval.second) {
                        // Update the safe interval based on the constraint.
                        if (safe_interval.first < constraint_time_interval.first) {
                            updated_safe_intervals.emplace_back(safe_interval.first, constraint_time_interval.first - 1);
                        }
                        if (safe_interval.second > constraint_time_interval.second) {
                            updated_safe_intervals.emplace_back(constraint_time_interval.second + 1, safe_interval.second);
                        }
                    // Otherwise, the first timestep is allowed (and everything after it is not, including the end time).
                    } else {
                        // Update the safe interval based on the constraint.
                        if (safe_interval.first <= constraint_time_interval.first) {
                            updated_safe_intervals.emplace_back(safe_interval.first, constraint_time_interval.first);
                        }
                        if (safe_interval.second > constraint_time_interval.second) {
                            updated_safe_intervals.emplace_back(constraint_time_interval.second + 1, safe_interval.second);
                        }
                    }
                }
            } else {
                // No overlap, include the safe interval as is.
                updated_safe_intervals.push_back(safe_interval);
            }
        }

        // Update the original vector with the new safe intervals.
        safe_intervals = std::move(updated_safe_intervals);
    }
}

int ConstraintsCollective::getNumSafeIntervals(){
    int num = 0;
    for (const auto& state_to_safe_intervals_pair : state_to_safe_intervals_) {
        num += (int)state_to_safe_intervals_pair.second.size();
    }
    return num;
}


}  // namespace ims

