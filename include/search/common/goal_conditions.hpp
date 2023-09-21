


#ifndef SEARCH_COMMON_GOAL_CONDITIONS_HPP
#define SEARCH_COMMON_GOAL_CONDITIONS_HPP

// standard includes
#include <functional>
#include <algorithm>
#include <utility>
#include <vector>
#include <memory>

#include <search/common/types.hpp>
#include <search/action_space/action_space.hpp>


namespace ims {

class GoalCondition
{
public:
    // @brief Returns true if goal state is reached
    virtual bool isGoalState(int state_id) = 0;

    // @brief Returns false if goal condition is definitely not possible.
    // Note: Even if it returns true, it is not guaranteed that the goal condition is possible
    virtual bool checkValidGoalCondition(const std::shared_ptr<ActionSpace>& action_space_ptr) = 0;
};


class GoalConditionExplicitIDs: public GoalCondition
{
protected:
    std::vector<int> m_goal_ids;

public:
    explicit GoalConditionExplicitIDs(std::vector<int> goal_ids): m_goal_ids(goal_ids) {};
    explicit GoalConditionExplicitIDs(int goal_id): m_goal_ids{goal_id} {};

    bool isGoalState(int state_id) override {
        return std::any_of(m_goal_ids.begin(), m_goal_ids.end(), [&state_id](int goal_ind) {return state_id == goal_ind;});
    };

    bool checkValidGoalCondition(const std::shared_ptr<ActionSpace>& action_space_ptr) override {
        bool atLeastOneValid = false;
        for (int goal_id : m_goal_ids) {
            StateType goal_state = action_space_ptr->getRobotState(goal_id)->state;
            if (!action_space_ptr->isStateValid(goal_state)) {
                atLeastOneValid = true;
            }
        }
        return atLeastOneValid;
    };
};

class GoalConditionEscapeLocalRegion: public GoalCondition
{
protected:
    StateType m_region_center;
    double m_region_size;
    std::shared_ptr<ActionSpace> m_action_space_ptr;

public:
    explicit GoalConditionEscapeLocalRegion(StateType region_center, double region_size, std::shared_ptr<ActionSpace> action_space_ptr): 
            m_region_center(region_center), m_region_size(region_size), m_action_space_ptr(action_space_ptr) {};

    bool isGoalState(int state_id) override {
        StateType cur_state = m_action_space_ptr->getRobotState(state_id)->state;
        for (int i = 0; i < cur_state.size(); i++) {
            if (fabs(cur_state[i]-m_region_center[i]) > m_region_size) {
                return true;
            }
        }
        return false;
    };

    bool checkValidGoalCondition(const std::shared_ptr<ActionSpace>& action_space_ptr) override {
        return true;
    }
};


} // ims
#endif //SEARCH_COMMON_GOAL_CONDITIONS_HPP