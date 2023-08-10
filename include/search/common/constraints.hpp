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
 * \file   constraints.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   July 13 2023
*/

#ifndef SEARCH_COMMON_CONSTRAINTS_HPP
#define SEARCH_COMMON_CONSTRAINTS_HPP

// standard includes
#include <functional>
#include <algorithm>
#include <utility>
#include <vector>
#include <memory>

namespace ims {

enum ConstraintType {
    UNSET_CONSTRAINT = -1,
    VERTEX_CONSTRAINT = 0,
    EDGE_CONSTRAINT = 1,
};

/// @brief Base class for all search constraints.
struct Constraint {
    /// @brief Constructor
    explicit Constraint() = default;

    /// @brief Virtual destructor.
    virtual ~Constraint() = default;

    /// @brief Get a string with information about the constraint.
    virtual std::string toString() const = 0;

    /// @brief The time of the constraint.
    /// @note This may be changed to a double for continuous time scenarios.
    virtual std::pair<int, int> getTimeInterval() const = 0;

    /// @brief The type of the constraint.
    ConstraintType type;
};

/// @brief Base class for all search constraint contexts.
/// @details This class is used to store context variables that are required for constraint checking and satisfaction. For example, in the case of PBS, the context for constraint satisfaction is the paths of all the agents whose paths were already found. This class is used to store these paths, and is passed to the ConstrainedActionSpace via the setConstraintsContext method.
struct ConstraintsContext {
    /// @brief Constructor
    explicit ConstraintsContext() = default;

    /// @brief Virtual destructor.
    virtual ~ConstraintsContext() = default;

    /// @brief The type of the constraint.
};

/// @brief Base class for all search constraints.
/// @details This class is used to store a set of constraints, and a context for constraint satisfaction.
struct ConstraintsCollective {
    /// @brief Constructor
    explicit ConstraintsCollective() = default;

    /// @brief Virtual destructor.
    virtual ~ConstraintsCollective() = default;

    // Setters.
    /// @brief Set the constraints.
    /// @param constraints The constraints to set.
    void setConstraints(const std::vector<std::shared_ptr<Constraint>>& constraints) {
        constraints_ptrs_ = constraints;

        // Set the time_to_constraints_ptrs_ map.
        time_to_constraints_ptrs_.clear();
        for (const auto& constraint_ptr : constraints_ptrs_) {
            for (int t = constraint_ptr->getTimeInterval().first; t <= constraint_ptr->getTimeInterval().second; t++) {
                time_to_constraints_ptrs_[t].push_back(constraint_ptr);
                
                // Update the latest time that has constraints, if necessary.
                last_constraint_time_ = std::max(last_constraint_time_, t);
            }
        }
    }

    void addConstraint(const std::shared_ptr<Constraint>& constraint) {
        constraints_ptrs_.push_back(constraint);

        // Add to the time_to_constraints_ptrs_ map.
        // TODO(yoraish): incorporate the action discretization into this for loop. This comes from the actionspace, so will have to think how to incorporate it. Currently the assumption is that the time is moving in integer steps.
        for (TimeType t = constraint->getTimeInterval().first; t <= constraint->getTimeInterval().second; t++) {
            time_to_constraints_ptrs_[t].push_back(constraint);

            // Update the latest time that has constraints, if necessary.
            last_constraint_time_ = std::max(last_constraint_time_, t);
        }
    }

    /// @brief Set the constraints context.
    /// @param context The constraints context to set.
    void setContext(const std::shared_ptr<ConstraintsContext>& context) {
        context_ptr_ = context;
    }

    /// @brief Clear the constraints context.
    void clearContext() {
        context_ptr_.reset();
    }

    /// @brief Clear the constraints.
    void clear() {
        constraints_ptrs_.clear();
    }

    // Getters.
    /// @brief Get the context pointer.
    /// @return The context pointer.
    std::shared_ptr<ConstraintsContext> getConstraintsContext() const {
        return context_ptr_;
    }

    /// @brief Get the constraints.
    /// @return The constraints.
    const std::vector<std::shared_ptr<Constraint>>& getConstraints() const {
        return constraints_ptrs_;
    }

    /// String.
    /// @brief Get a string with information about the constraint.
    std::string toString() const {
        std::string str = "ConstraintsCollective: \n";
        for (const auto& constraint_ptr : constraints_ptrs_) {
            str += "    " + constraint_ptr->toString() + "\n";
        }
        return str;
    }

    /// @brief Get the greatest time that is constrained.
    /// @return The greatest time that is constrained.
    int getLastConstraintTime() const {
        return last_constraint_time_;
    }

private:
    /// @brief Map from a timestep to a set of constraint pointers.
    std::unordered_map<int, std::vector<std::shared_ptr<Constraint>>> time_to_constraints_ptrs_ = {};

    // Member Variables.
    /// @brief The type of the constraint.
    std::shared_ptr<ConstraintsContext> context_ptr_ = std::make_shared<ConstraintsContext>();

    /// @brief The type of the constraint.
    std::vector<std::shared_ptr<Constraint>> constraints_ptrs_ = {};

    /// @brief The last time that has constraints imposed on it.
    TimeType last_constraint_time_ = -1;
};

/// @brief An object for mapping [agent_ids] to a set of constraints collective.
using MultiAgentConstraintsCollective = std::unordered_map<int, ConstraintsCollective>;

/// @brief An object for mapping [agent_ids][timestamp] to a state.
using MultiAgentPaths = std::unordered_map<int, std::vector<StateType>>;

}  // namespace ims

#endif //SEARCH_COMMON_CONSTRAINTS_HPP