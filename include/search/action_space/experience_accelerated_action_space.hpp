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
 * \file   experience_accelerated_action_space.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   Oct 05 2023
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
#include "search/common/experiences.hpp"
#include <search/action_space/constrained_action_space.hpp>
#include <search/action_space/mixin/action_space_experience_acceleratable_mixin.hpp>

namespace ims {

/// @brief Base class for ActionSpaces with constraints.
/// @details This is a constrained action space extended to be "Experience Acceleratable" using a mixin. We add this mixin to a constrained action space and not adding both the "Constrainable" and the "ExperienceAcceleratable" mixins separately because we want to make sure that this class could be a derived class from a constrained action space.
class ExperienceAcceleratedActionSpace : virtual public ActionSpace, public ActionSpaceExperienceAcceleratableMixin {
public:
    /// @brief Constructor.
    explicit ExperienceAcceleratedActionSpace(): ActionSpace(), ActionSpaceExperienceAcceleratableMixin() {
        std::cout << "ExperienceAcceleratedActionSpace: Constructor" << std::endl;
        experiences_collective_ptr_ = std::make_shared<ExperiencesCollective>();
    }

    /// @brief Destructor
    ~ExperienceAcceleratedActionSpace() = default;


    // A method that guarantees any state passed creates a new state_id or reuses one that is not the goal. No goal checking is done (i.e., setting the time to -1).
    inline int getOrCreateRobotStateNonGoal(const StateType& state_val) {
        // Check if the state exists
        auto *curr_state = new ims::RobotState;

        // If that failed, then we should check if the state exists at the current time.
        curr_state->state = state_val;
        auto it = state_to_id_.find(curr_state);
        if (it != state_to_id_.end()) {
            delete curr_state;
            return it->second;
        }

        // If we do not have any memory of a state like this, with or without time set to -1, then we should create a new state.
        states_.push_back(curr_state);
        int state_id = (int)states_.size() - 1;
        state_to_id_[curr_state] = state_id;
        return state_id;
    }

    /// @brief To comply with the ActionSpaceExperienceAcceleratableMixin interface.
    /// @param state_id The state to get the valid experience subpaths for.
    /// @param subpaths The vector of subpaths -- to be updated with the subpaths.
    void getValidExperienceSubpathsFromState(int state_id,
                                             std::vector<std::vector<std::vector<int>>>& experience_seq_subpaths,
                                             std::vector<std::vector<std::vector<double>>>& experience_seq_subpaths_transition_costs) override = 0;

};

}  // namespace ims
