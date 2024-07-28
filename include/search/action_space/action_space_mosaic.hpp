//
// Created by itamar on 7/7/24.
//

#pragma once

// standard includes
#include <functional>
#include <algorithm>
#include <utility>
#include <random>
#include <Eigen/Dense>
#include <queue>
// project includes
#include <search/action_space/action_space.hpp>
#include <search/heuristics/base_heuristic.hpp>
#include <search/planners/planner.hpp>

namespace ims {

class ActionSpaceMosaic: virtual public ActionSpace {
public:

    std::shared_ptr<ActionType> action_type_;

    /// @brief Constructor
    /// @param action_type The type of the action space
    explicit ActionSpaceMosaic(const std::shared_ptr<ActionType>& action_type) :
        ActionSpace() {
        action_type_ = action_type;
    }

    ActionSpaceMosaic() = default;

    /// @brief Destructor
    ~ActionSpaceMosaic() {
        // call the base class destructor
        for (auto trajectory : mosaic_trajectories_) {
            delete trajectory;
        }
//        for (auto state : states_) {
//            delete state;
//        }
    }

    /// @struct Trajectory
    /// @brief A struct to store a trajectory, and assign a id to it.
    struct Trajectory {
        int id {UNSET}; // The id of the trajectory
        Eigen::MatrixXd trajectory; // The trajectory. Each column is a state, and the rows are the trajectory.
        // Edges of the trajectory (trajectory ids, connecting state ids)
        std::vector<std::pair<int, int>> edges; // TODO: remove it. edges should be tracked only inside the planner. We do it here for the controller to have access to it.
    };

    /// @brief Create a new trajectory from Eigen::MatrixXd object, and return the id of the trajectory
    /// @note This function should not be called from your planner, but from the action space, or from a controller.
    /// @param trajectory The trajectory
    /// @return The id of the trajectory
    int createTrajectory(const Eigen::MatrixXd& trajectory) {
        auto* new_trajectory = new Trajectory;
        new_trajectory->trajectory = trajectory;
        new_trajectory->id = (int)mosaic_trajectories_.size();
        mosaic_trajectories_.push_back(new_trajectory);
        return new_trajectory->id;
    }

    /// @brief Create a new trajectory from a vector of integers, and return the id of the trajectory
    /// @param trajectory The trajectory as a vector of integers
    /// @return The id of the trajectory
    int createTrajectory(const std::vector<int>& trajectory) {
        assert(!trajectory.empty());
        assert(trajectory.at(0) < states_.size());
        auto* first_state = getRobotHashEntry(trajectory[0]);
        assert(first_state != nullptr);
        Eigen::MatrixXd trajectory_matrix(trajectory.size(), first_state->state.size());
        for (int i = 0; i < first_state->state.size() && i < trajectory_matrix.cols(); ++i) {
            trajectory_matrix(0, i) = first_state->state[i];
        }
        for (int i = 1; i < trajectory.size(); ++i) {
            auto* state = getRobotHashEntry(trajectory[i]);
            assert(state != nullptr);
            for (int j = 0; j < state->state.size() && j < trajectory_matrix.cols(); ++j) {
                trajectory_matrix(i, j) = state->state[j];
            }
        }
        return createTrajectory(trajectory_matrix);
    }


    /// @brief Get the trajectory given an id
    /// @param trajectory_id The id of the trajectory
    /// @return The trajectory
    /// @note Use this function only if you are sure that the trajectory exists
    auto getTrajectory(int trajectory_id) -> Trajectory* {
        assert(trajectory_id < mosaic_trajectories_.size());
        return mosaic_trajectories_[trajectory_id];
    }

    /// @brief A function that checks if two trajectories are connected
    /// @param traj1_id The id of the first trajectory
    /// @param traj2_id The id of the second trajectory
    /// @return True if the trajectories are connected
    /// TODO: Remove from here. It should be in action space!!
    bool areTrajectoriesConnected(int traj1_id, int traj2_id) {

        std::queue<int> q;
        std::vector<bool> visited(mosaic_trajectories_.size(), false);
        q.push(traj1_id);
        visited[traj1_id] = true;
        while (!q.empty()) {
            int curr_traj_id = q.front();
            q.pop();
            Trajectory* curr_traj = mosaic_trajectories_[curr_traj_id];
            for (auto edge : curr_traj->edges) {
                if (edge.first == traj2_id) {
                    return true;
                }
                if (!visited[edge.first]) {
                    q.push(edge.first);
                    visited[edge.first] = true;
                }
            }
        }
        return false;
    }

    /// @brief A function to lookup whether two trajectories cannot be connected
    /// @param traj1_id The id of the first trajectory
    /// @param traj2_id The id of the second trajectory
    /// @return True if the trajectories are unconnectable
    bool areTrajectoriesUnconnectable(int traj1_id, int traj2_id) {

        return std::any_of(unconnectable_trajectories_.begin(), unconnectable_trajectories_.end(), [traj1_id, traj2_id](const std::pair<int, int>& disconnected_traj) {
            return (disconnected_traj.first == traj1_id && disconnected_traj.second == traj2_id) ||
                   (disconnected_traj.first == traj2_id && disconnected_traj.second == traj1_id);
        });
    }

    std::vector<Trajectory*> mosaic_trajectories_; // The trajectories

    std::vector<std::pair<int, int>> connected_trajectories_; // The connected trajectories

    std::vector<std::pair<int, int>> unconnectable_trajectories_; // The disconnected trajectories. TODO: This can be optimize in many cool ways!
    // TODO: It is important to also make sure that if we cannot guarantee that two trajectories are disconnected, then we allow later on to try to connect them again.

};

}  // namespace ims

