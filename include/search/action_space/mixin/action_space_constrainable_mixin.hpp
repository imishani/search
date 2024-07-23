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
 * \file   action_space_constrainable_mixin.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   August 10 2023
 */
#pragma once

// standard includes
#include <functional>
#include <algorithm>
#include <utility>
#include <boost/functional/hash.hpp>

// project includes
#include <search/action_space/action_space.hpp>
#include <search/heuristics/base_heuristic.hpp>
#include <search/planners/planner.hpp>
#include <search/common/constraints.hpp>
#include <search/common/conflicts.hpp>

namespace ims {

/// @brief A trait class (mixin) for allowing an ActionSpace to be constrained.
/// @details This class is used extend an ActionSpace with constraints. The pure virtual isConstrainedStateValid method is added, and must be implemented by the user if they want to use constraint-based search algorithms. In these algorithms, this method wil replace the familiar isStateValid method of Action Spaces. This new method is used to check if a given state is valid with respect to the constraints.
class ActionSpaceConstrainableMixin {
public:
    /// @brief Constructor
    explicit ActionSpaceConstrainableMixin(){
        constraints_collective_ptr_ = std::make_shared<ConstraintsCollective>();
    }

    /// @brief Destructor
    ~ActionSpaceConstrainableMixin() = default;

    /// @brief Set the constraints. The ConstraintsCollective object includes (at least) two objects, a set of constraints, and a context.
    /// @param constraints The constraints to set.
    void setConstraintsCollective(const std::shared_ptr<ConstraintsCollective>& constraints_collective) {
        constraints_collective_ptr_ = constraints_collective;
    }

    /// @brief Get the constraints collective.
    /// @return The constraints collective.
    ConstraintsCollective getConstraintsCollective() const {
        return *constraints_collective_ptr_;
    }

    /// @brief Clear the constraints.
    void clearConstraints() { constraints_collective_ptr_->clear(); }

    /// @brief Set the constraints context.
    /// @param context The constraints context to set.
    void setConstraintsContext(const std::shared_ptr<ConstraintsContext>& context) {
        constraints_collective_ptr_->setContext(context);
    }

    /// @brief Find conflicts given a set of paths.
    /// @param paths The paths to check for conflicts.
    /// @return A vector of conflicts.
    /// @brief Ask for conflicts between paths.
    /// @param paths The paths to be looked at.
    /// @param conflicts_ptrs The conflicts that were found. This is populated by the function.
    /// @param conflict_types The types of conflicts that are requested.
    /// @param max_conflicts The maximum number of conflicts to return.
    /// @param names The names of the agents.
    /// @param time_start The start time from which to check for conflicts. Inclusive. -1 defaults to zero.
    /// @param time_end The end time until which to check for conflicts. Inclusive. -1 defaults to the end.
    virtual void getPathsConflicts(std::shared_ptr<MultiAgentPaths> paths,
                           std::vector<std::shared_ptr<Conflict>>& conflicts_ptrs, 
                           const std::vector<ConflictType>& conflict_types,
                           int max_conflicts, 
                           const std::vector<std::string> &names,
                           TimeType time_start = 0,
                           TimeType time_end = -1) = 0;

    // Member variables.
    /// @brief The constraints.
    std::shared_ptr<ConstraintsCollective> constraints_collective_ptr_ = std::make_shared<ConstraintsCollective>();
};

}  // namespace ims

