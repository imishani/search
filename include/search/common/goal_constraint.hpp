/*
 * Copyright (C) 2023, Itamar Mishani
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
 * \file   goal_constraint.hpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   6/19/24
 */

#pragma once

#include "search/common/types.hpp"
#include "search/action_space/action_space.hpp"

namespace ims {

enum GoalType
{
    INVALID_GOAL_TYPE = -1,
    SINGLE_SEARCH_STATE_GOAL,
    SINGLE_SEARCH_STATE_MAPPED_GOAL,
    MULTI_SEARCH_STATE_GOAL,
    MULTI_SEARCH_STATE_MAPPED_GOAL,
    USER_GOAL_CONSTRAINT_FN,
    NUMBER_OF_GOAL_TYPES
};

using GoalConstraintFn = bool (*)(void* user,
                                  const std::shared_ptr<ActionSpace>& action_space_ptr,
                                  int state_id);

inline
bool singleGoalConstraint(void* user,
                          const std::shared_ptr<ActionSpace>& action_space_ptr,
                          int state_id) {
    int goal_state_id = *static_cast<int*>(user);
    return state_id == goal_state_id;
}

inline
bool multiGoalConstraint(void* user,
                          const std::shared_ptr<ActionSpace>& action_space_ptr,
                          int state_id) {
    std::vector<int> goal_state_id = *static_cast<std::vector<int>*>(user);
    return std::find(goal_state_id.begin(), goal_state_id.end(), state_id) != goal_state_id.end();
}

inline
bool invalidGoalConstraint(void* user,
                           const std::shared_ptr<ActionSpace>& action_space_ptr,
                           int state_id) {
    throw std::runtime_error("Invalid goal constraint type");
}


struct GoalConstraint {
    // Type of goal constraint
    GoalType type {INVALID_GOAL_TYPE};
    GoalConstraintFn check_goal {invalidGoalConstraint}; // Function pointer to check goal in case of USER_GOAL_CONSTRAINT_FN
    void* check_goal_user {nullptr}; // User data for the goal checking function in case of USER_GOAL_CONSTRAINT_FN
    std::shared_ptr<ActionSpace> action_space_ptr {nullptr};

    bool checkGoal(int state_id) const {
        if (action_space_ptr == nullptr || type == INVALID_GOAL_TYPE) {
            throw std::runtime_error("Action space pointer is null");
        } else{
            return check_goal(check_goal_user, action_space_ptr, state_id);
        }
    }
};

}

