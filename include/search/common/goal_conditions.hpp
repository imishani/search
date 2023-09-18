


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


/// @brief The standard heuristic functions
namespace ims {

class CheckGoalCondition
{
public:
    virtual bool isGoalState(int state_id) = 0;
};


class CheckExplicitGoalIDs: public CheckGoalCondition
{
protected:
    std::vector<int> m_goal_ids;

public:
    CheckExplicitGoalIDs(std::vector<int> goal_ids): m_goal_ids(goal_ids){};

    bool isGoalState(int state_id) override {
        return std::any_of(m_goal_ids.begin(), m_goal_ids.end(), [&state_id](int goal_ind) {return state_id == goal_ind;});
    };
};

class CheckEscapeLocalRegion: public CheckGoalCondition
{
protected:
    StateType m_region_center;
    double m_region_size;
    std::shared_ptr<ActionSpace> m_action_space_ptr;

public:
    CheckEscapeLocalRegion(StateType region_center, double region_size, std::shared_ptr<ActionSpace> action_space_ptr): 
            m_region_center(region_center), m_region_size(region_size), m_action_space_ptr(action_space_ptr) {};

    bool isGoalState(int state_id) override {
        StateType cur_state = m_action_space_ptr->getRobotState(state_id)->state;
        for (int i = 0; i < cur_state.size(); i++)
        {
            if (fabs(cur_state[i]-m_region_center[i]) > m_region_size)
            {
                return true;
            }
        }
        return false;
    };
};


} // ims
#endif //SEARCH_COMMON_GOAL_CONDITIONS_HPP