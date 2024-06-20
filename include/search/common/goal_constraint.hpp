//
// Created by itamar on 6/19/24.
//

#ifndef SEARCH_SEARCH_INCLUDE_SEARCH_COMMON_GOAL_CONSTRAINT_HPP_
#define SEARCH_SEARCH_INCLUDE_SEARCH_COMMON_GOAL_CONSTRAINT_HPP_

#include "search/common/types.hpp"
#include "search/action_space/action_space.hpp"

namespace ims {

enum GoalType
{
    INVALID_GOAL_TYPE = -1,
    SINGLE_SEARCH_STATE_GOAL,
    SINGLE_SEARCH_STATE_MAPPED_GOAL,
    MULTI_SEARCH_STATE_GOAL,
    USER_GOAL_CONSTRAINT_FN,
    NUMBER_OF_GOAL_TYPES
};

using GoalConstraintFn = bool (*)(void* user,
                                  const std::shared_ptr<ActionSpace>& action_space_ptr,
                                  int state_id);

inline
bool checkGoalSingleSearchState(void* user,
                                const std::shared_ptr<ActionSpace>& action_space_ptr,
                                int state_id) {
    int goal_state_id = *static_cast<int*>(user);
    return state_id == goal_state_id;
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

#endif //SEARCH_SEARCH_INCLUDE_SEARCH_COMMON_GOAL_CONSTRAINT_HPP_
