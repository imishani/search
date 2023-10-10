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
class SingleQueuePlanner {
public:
    /// @brief The search state
    struct GenericSearchState : public LowerBoundMixin {
        int robot_state_id = -1234;
        int search_id = -1235;
        int parent_id = -1236;
        double g = -1237;

        GenericSearchState(int robot_state_id, int parent_id, double g):
                robot_state_id(robot_state_id), parent_id(parent_id), g(g) {}
        virtual ~GenericSearchState() = default;
        virtual double getLowerBound() const override { return g; }
        void setSearchId(int search_id) { this->search_id = search_id; }
    };

    struct SingleQueueSettings {
        virtual ~SingleQueueSettings() = default;
        virtual bool skipAsAlreadyExpanded(GenericSearchState* state) = 0;
        virtual void addToExpanded(GenericSearchState* state) = 0;
        virtual AbstractQueue<GenericSearchState>* getQueue() = 0;
        virtual void updateQueue() = 0;
        virtual GenericSearchState* createNewSearchState(int robot_state_id,
                                        GenericSearchState* parent, double cost) = 0;
    };

    SingleQueuePlanner(SingleQueueSettings* settings) : settings_(settings),
                        main_queue_(settings->getQueue()) {}
    virtual ~SingleQueuePlanner();


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

private:
    SingleQueueSettings* settings_;

    double time_limit_ = 1000;
    std::chrono::time_point<std::chrono::steady_clock> t_start_;
    std::vector<int> goals_;
    std::shared_ptr<ActionSpace> action_space_ptr_;
    vector<GenericSearchState*> states_;
    AbstractQueue<GenericSearchState>* main_queue_ = nullptr;

    bool isGoalState(GenericSearchState* state) {
        return std::find(goals_.begin(), goals_.end(), state->robot_state_id) != goals_.end();
    }


    /// @brief Get the state by id
    /// @param state_id The id of the state
    /// @return The state
    /// @note Use this function only if you are sure that the state exists.
    GenericSearchState* getSearchState(int state_id);
    void addNewSearchState(GenericSearchState* search_state);

    /// @brief Expand the current state
    void expand(int state_id);

    void reconstructPath(int goal_state_id, std::vector<StateType>& path);

    /////////////////////////////////////////
    /////// Timing Functions ////////////////

    /// @brief start the timer
    void startTimer() { t_start_ = std::chrono::steady_clock::now(); }

    void getTimeFromStart(double &elapsed_time) {
        auto t_end = std::chrono::steady_clock::now();
        double scaler = 1e9;
        elapsed_time = (double)std::chrono::duration_cast<std::chrono::nanoseconds>(t_end - t_start_).count();
        elapsed_time /= scaler;
    }

    bool isTimeOut() {
        double elapsed_time;
        getTimeFromStart(elapsed_time);
        return elapsed_time > time_limit_;
    }
    
};

}  // namespace ims

