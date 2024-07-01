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
#pragma once

// standard includes
#include <functional>
#include <algorithm>
#include <utility>

// project includes
#include "action_space.hpp"
#include "search/heuristics/base_heuristic.hpp"
#include "search/planners/planner.hpp"
#include "search/common/constraints.hpp"
#include "search/common/conflicts.hpp"
#include "mixin/action_space_constrainable_mixin.hpp"

namespace ims {

/// @brief Base class for ActionSpaces with constraints.
/// @details This is an actions space extended to be "Constrainable" using a mixin.
class ConstrainedActionSpace : virtual public ActionSpace, public ActionSpaceConstrainableMixin{
public:
    /// @brief Constructor
    explicit ConstrainedActionSpace(): ActionSpace(), ActionSpaceConstrainableMixin() {
        constraints_collective_ptr_ = std::make_shared<ConstraintsCollective>();
    }

    /// @brief Destructor
    ~ConstrainedActionSpace() = default;

    /// @brief Whether the state is satisfying a specific constraint.
    /// @param state_val The state value to check. Timed.
    /// @param next_state_val The next state value to check. Timed.
    /// @param constraint_ptr The constraint to check.
    /// @return True if the constraint is satisfied, false otherwise.
    virtual bool isSatisfyingConstraint(const StateType &state_val, const StateType &next_state_val, const std::shared_ptr<Constraint> &constraint_ptr) = 0;

    /// @brief If a given timed configuration transition is valid with respect to all of the constraints. It is assumed that agents can be at state, and so next_state along with the transition to it is checked.
    inline bool isSatisfyingAllConstraints(const StateType& state, const StateType& next_state) {
        for (const auto& constraint_ptr : constraints_collective_ptr_->getConstraints()) {
            if (!isSatisfyingConstraint(state, next_state, constraint_ptr)) {
                return false;
            }
        }
        return true;
    }

    /// @brief Get safe intervals for a given configuration.
    /// @param state_id The id of the configuration. The RobotState.
    /// @return A vector of safe intervals.
    virtual void getSafeIntervals(int state_id, std::vector<SafeIntervalType>& safe_intervals){
        const auto & curr_state = this->getRobotState(state_id);
        // Ask the constraints collective for the safe intervals of this state configuration.
        constraints_collective_ptr_->getOrCreateSafeIntervals(curr_state->state, safe_intervals);
    }

};



}  // namespace ims

