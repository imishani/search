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
 * \file   constrained_search.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   July 10 2023
 */

#ifndef SEARCH_CONSTRAINEDSEARCH_HPP
#define SEARCH_CONSTRAINEDSEARCH_HPP

// standard includes
#include <functional>
#include <algorithm>
#include <utility>

// project includes
#include <search/common/action_space.hpp>
#include <search/common/base_heuristic.hpp>
#include <search/planners/planner.hpp>
#include <search/common/constraints.hpp>
#include <search/common/conflicts.hpp>

namespace ims {

/// @brief Base class for ActionSpaces with constraints.
/// @details This class is used extend an ActionSpace with constraints. The pure virtual isConstrainedStateValid method is added, and must be implemented by the user if they want to use constraint-based search algorithms. In these algorithms, this method wil replace the familiar isStateValid method of Action Spaces. This new method is used to check if a given state is valid with respect to the constraints.
class ConstrainedActionSpace : public ActionSpace {
public:
    /// @brief Constructor
    explicit ConstrainedActionSpace(){
        std::cout << "ConstrainedActionSpace: Constructor" << std::endl;
        constraints_collective_ptr_ = std::make_shared<ConstraintsCollective>();
    }

    /// @brief Destructor
    ~ConstrainedActionSpace() = default;

    // The additions to the ActionSpace class are the following:
    // 1. A method to set the constraints.
    // 2. A method to set the constraints context.
    // 3. A method to find conflicts given a set of paths.
    // 4. The implementation of isStateValid that takes in a state, and may use the stored constraints and paths to check if the state is valid.

    /// @brief Set the constraints. The ConstraintsCollective object includes (at least) two objects, a set of constraints, and a context.
    /// @param constraints The constraints to set.
    void setConstraintsCollective(const std::shared_ptr<ConstraintsCollective>& constraints_collective) {
        constraints_collective_ptr_ = constraints_collective;
    }

    /// @brief Clear the constraints.
    void clearConstraints() { constraints_collective_ptr_->clear(); }

    /// @brief Set the constraints context.
    /// @param context The constraints context to set.
    void setConstraintsContext(const std::shared_ptr<ConstraintsContext>& context) {
        constraints_collective_ptr_->setContext(context);
    }

    /// @brief Reset the constrained action space.
    void resetPlanningData() override {
        ActionSpace::resetPlanningData();
        // constraints_collective_ptr_->clear();
    }

    /// @brief Find conflicts given a set of paths.
    /// @param paths The paths to check for conflicts.
    /// @return A vector of conflicts.
    virtual void getPathsConflicts(std::shared_ptr<MultiAgentPaths> paths, std::vector<std::shared_ptr<Conflict>>& conflicts_ptrs, int max_conflicts = 1) = 0;

    // Member variables.
    /// @brief The constraints.
    std::shared_ptr<ConstraintsCollective> constraints_collective_ptr_;
};

}  // namespace ims

#endif  // SEARCH_CONSTRAINEDSEARCH_HPP