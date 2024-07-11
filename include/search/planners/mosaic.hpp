//
// Created by itamar on 7/7/24.
//

#pragma once

// Standard includes
#include <queue>
#include <boost/filesystem.hpp>
// Project includes
#include "planner.hpp"
#include "search/action_space/action_space_mosaic.hpp"
#include "search/controllers/base_controller.hpp"

namespace ims {

/// @struct mosaic parameters
struct MosaicParams : public PlannerParams {
    /// @brief Constructor
    MosaicParams() : PlannerParams() {}
};

class Mosaic : public Planner {
private:

    /// @brief The graph state
    struct GraphState {
        int state_id {UNSET};
        std::shared_ptr<std::vector<int>> in_trajs {nullptr};
        std::shared_ptr<std::vector<std::pair<int, double>>> edges {nullptr};
        bool is_connected_to_goal {false};
        bool is_connected_to_start {false};
    };

    /// @brief Keep track of the states
    std::vector<GraphState*> states_;

    /// @brief The parameters
    MosaicParams params_;

    /// @brief The start state
    GraphState* start_ {nullptr};
    /// @brief The goal state TODO: Change to goal constraint (have it on a different branch)
    GraphState* goal_ {nullptr};

    /// @brief Get the state by id
    /// @param state_id The id of the state
    /// @return The state
    /// @note Use this function only if you are sure that the state exists.
    auto getGraphState(int state_id) -> GraphState*;

    /// @brief Get the state by id or create a new one if it does not exist.
    /// If a search state does not exist yet and a new one is created, it's ID will be set,
    /// and all other member fields will initialize to default values.
    /// @param state_id The id of the state
    /// @return GraphState: The state
    auto getOrCreateGraphState(int state_id) -> GraphState*;

    /// @brief Graph for finding a path from start state to goal constraint.
    /// @details The vertex are the graph indices and the edges are undirected connections between the graphs
    std::unordered_map<int, std::vector<int>> hl_graph_;


    struct TrajectoryState {
        int id {UNSET}; // The id of the trajectory: matches the index in the action space
        // the trajectory
        std::vector<int> trajectory;
        // Edges of the trajectory (trajectory ids, connecting state ids)
        std::vector<std::pair<int, int>> edges;
        // The cost of the trajectory
        double cost {0};
    };

    std::vector<TrajectoryState*> mosaic_trajectories_;

    /// @brief Create a new trajectory from a vector of integers, and return the id of the trajectory
    /// @param trajectory The trajectory as a vector of integers
    /// @return The id of the trajectory
    auto createTrajectory(const std::vector<int>& trajectory) -> TrajectoryState*;

    /// @brief Get the trajectory given an id
    /// @param trajectory_id The id of the trajectory
    /// @return The trajectory
    /// @note Use this function only if you are sure that the trajectory exists
    auto getTrajectory(int trajectory_id) -> TrajectoryState*;


    /// @brief A function that checks if two trajectories are connected
    /// @param traj1_id The id of the first trajectory
    /// @param traj2_id The id of the second trajectory
    /// @return True if the trajectories are connected
    bool areTrajectoriesConnected(int traj1_id, int traj2_id);


public:

    /// @brief Constructor
    /// @param params The planner parameters based on MosaicParams struct
    explicit Mosaic(const MosaicParams& params);

    /// @brief Destructor
    ~Mosaic() override;

    /// @brief Initialize the planner
    /// @param action_space_ptr The action space
    /// @param controllers The controllers
    /// @param start The start state
    /// @param goal The goal state
    void initializePlanner(const std::shared_ptr<ActionSpaceMosaic>& action_space_ptr,
                           std::shared_ptr<std::vector<Controller>>& controllers,
                           const StateType& start, const StateType& goal);

    inline void initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
                                  const StateType& starts,
                                  const StateType& goals) override {
        throw std::runtime_error("Mosaic::initializePlanner() called without controllers.");
    }

    /// @brief plan a path
    /// @param path The path
    /// @return if the plan was successful or not
    bool plan(std::vector<StateType>& path) override;

    void resetPlanningData() override;

    /// @brief Generate function
    void generate();

protected:

    void reconstructPath(std::vector<StateType>& path) override;
    void reconstructPath(std::vector<StateType>& path, std::vector<double>& costs) override;
    void reconstructPath(std::vector<StateType>& path, const std::vector<int>& trajs_path);

    bool isGoalState(int state_id) override;

    /// @brief Check if the goal condition is satisfied
    /// @param trajs_path the high-level path of trajectories in the mosaic
    /// @return True if the goal condition is satisfied
    bool isGoalConditionSatisfied(std::vector<int>& trajs_path);


    void saveData();


    // The action space.
    std::shared_ptr<ActionSpaceMosaic> action_space_ptr_;

    // The controllers.
    std::shared_ptr<std::vector<Controller>> controllers_;

    TrajectoryState* start_traj_ {nullptr};
    TrajectoryState* goal_traj_ {nullptr};
    std::vector<int> trajs_path_;

};


} // namespace ims
