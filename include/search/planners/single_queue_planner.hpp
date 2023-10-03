#pragma once

// standard includes
#include <functional>
// Standard includes
#include <utility>
#include <algorithm>

// project includes
#include <search/planners/planner.hpp>
#include "search/heuristics/base_heuristic.hpp"
#include "search/common/queue_general.h"

namespace ims{

/// @class SingleQueuePlanner class.
/// @brief A general planner that uses a single Queue class
class SingleQueuePlanner : public Planner {
public:
    /// @brief The search state
    struct GenericSearchState {
        int robot_state_id;
        int search_id;
        int parent_id;
        double g;

        GenericSearchState(int robot_state_id, int search_id, int parent_id, double g):
                robot_state_id(robot_state_id), search_id(search_id), parent_id(parent_id), g(g) {}
        virtual ~GenericSearchState() = default;
    };

    /// @brief Initialize the planner
    /// @param action_space_ptr The action space
    /// @param starts Vector of start states
    /// @param goals Vector of goal states
    void initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
                                    const std::vector<StateType>& starts,
                                    const std::vector<StateType>& goals);

    /// @brief Initialize the planner
    /// @param action_space_ptr The action space
    /// @param start The start state
    /// @param goal The goal state
    void initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
                            const StateType& start, const StateType& goal) {
        initializePlanner(action_space_ptr, std::vector<StateType>{start}, std::vector<StateType>{goal});
    }

    /// @brief plan a path
    /// @param path The path
    /// @return if the plan was successful or not
    bool plan(std::vector<StateType>& path);

    void resetPlanningData();

protected:
    vector<GenericSearchState*> states_;
    AbstractQueue<GenericSearchState>* main_queue_ = nullptr;

    virtual void populateSearchState(GenericSearchState* state, 
                    GenericSearchState* parent, double cost) = 0;
    virtual bool skipAsAlreadyExpanded(GenericSearchState* state) = 0;
    virtual void addToExpanded(GenericSearchState* state) = 0;
    virtual void createQueue() = 0;


    /// @brief Get the state by id
    /// @param state_id The id of the state
    /// @return The state
    /// @note Use this function only if you are sure that the state exists.
    GenericSearchState* getSearchState(int state_id);

    GenericSearchState* SingleQueuePlanner::createNewSearchState(int robot_state_id,
                                        GenericSearchState* parent, double cost);

    /// @brief Expand the current state
    void expand(int state_id);

    void reconstructPath(int goal_state_id, std::vector<StateType>& path);
};

}  // namespace ims

