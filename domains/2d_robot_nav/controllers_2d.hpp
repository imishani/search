//
// Created by itamar on 7/6/24.
//

#pragma once

#include "search/common/types.hpp"
#include "search/controllers/base_controller.hpp"
#include "search/planners/wastar.hpp"
#include "search/heuristics/standard_heuristics.hpp"
#include "action_space_2d_rob.hpp"
#include "action_space_2d_rob_mosaic.hpp"

#include <Eigen/Dense>
#include <utility>

namespace ims {


Eigen::VectorXf applyDerivativeFilter(const Eigen::VectorXf &row) {
    Eigen::VectorXf filteredRow(row.size());
    for (int i = 1; i < row.size() - 1; ++i) {
        filteredRow[i] = row[i + 1] - row[i];
    }
    filteredRow[0] = row[1] - row[0];
    filteredRow[row.size() - 1] = row[row.size() - 1] - row[row.size() - 2];
    return filteredRow;
}

// Function to apply a 1D derivative filter. Takes in a batch of rows
Eigen::MatrixXf applyDerivativeFilterBatch(const Eigen::MatrixXf &rows) {
    Eigen::MatrixXf filteredRows(rows.rows(), rows.cols());
    for (int i = 0; i < rows.rows(); ++i) {
        Eigen::VectorXf row = rows.row(i);
        Eigen::VectorXf filteredRow = applyDerivativeFilter(row);
        filteredRows.row(i) = filteredRow;
    }
    return filteredRows;
}

// Function to detect walls using derivative filters
void detectWalls(const std::vector<std::vector<int>> &gridMap,
                 std::vector<std::vector<int>> &wallMap,
                 std::vector<std::vector<std::pair<int, int>>> &trajectories,
                 int batch_size = 10, int stride = 1,
                 double obs_threshold = 100) {
    int rows = (int) gridMap.size();
    int cols = (int) gridMap[0].size();
    wallMap.resize(rows, std::vector<int>(cols, 0));

    std::vector<bool> prev_row_batch(cols, false);
    std::vector<std::vector<std::pair<int, int>>> prev_trajectories(cols, std::vector<std::pair<int, int>>());
    for (int i{0}; i < rows - batch_size; i += stride) {
        // take a batch
        Eigen::MatrixXf batch(batch_size, cols);
        for (int j{0}; j < batch_size; ++j) {
            for (int k{0}; k < cols; ++k) {
                batch(j, k) = (float) gridMap[i + j][k];
            }
        }

        Eigen::MatrixXf filteredBatch = applyDerivativeFilterBatch(batch);
        // generate a vector of size cols, where each element is the sum of the absolute values of the rows in that column
        Eigen::VectorXf colSums = filteredBatch.cwiseAbs().colwise().sum();
        for (int j{0}; j < cols; ++j) {
            if (colSums[j] >= batch_size * obs_threshold) {
                if (prev_row_batch[j]) {
                    // the last states which ar not in the previous trajectory based on the strid
                    for (int k{batch_size - stride}; k < batch_size; ++k) {
                        wallMap[i + k][j] = 1;
                        prev_trajectories[j].emplace_back(i + k, j);
                    }
                } else {
                    for (int k{0}; k < batch_size; ++k) {
                        wallMap[i + k][j] = 1;
                        prev_trajectories[j].emplace_back(i + k, j);
                    }
                    prev_row_batch[j] = true;
                }
            } else {
                if (prev_row_batch[j]) {
                    trajectories.emplace_back(prev_trajectories[j]);
                    prev_trajectories[j].clear();
                    prev_row_batch[j] = false;
                }
            }
        }
    }

    // lets do the same thing for the columns
    std::vector<bool> prev_col_batch(rows, false);
    std::vector<std::vector<std::pair<int, int>>> prev_trajectories_col(rows,
                                                                        std::vector<std::pair<int, int>>());
    for (int i{0}; i < cols - batch_size; i += stride) {
        // take a batch
        Eigen::MatrixXf batch(rows, batch_size);
        for (int j{0}; j < rows; ++j) {
            for (int k{0}; k < batch_size; ++k) {
                batch(j, k) = (float) gridMap[j][i + k];
            }
        }
        Eigen::MatrixXf filteredBatch = applyDerivativeFilterBatch(batch.transpose());
        filteredBatch.transposeInPlace();
        // generate a vector of size cols, where each element is the sum of the absolute values of the rows in that column
        Eigen::VectorXf rowSums = filteredBatch.cwiseAbs().rowwise().sum();
        for (int j{0}; j < rows; ++j) {
            if (rowSums[j] >= batch_size * obs_threshold) {
                if (prev_col_batch[j]) {
                    // the last states which ar not in the previous trajectory based on the strid
                    for (int k{batch_size - stride}; k < batch_size; ++k) {
                        wallMap[j][i + k] = 1;
                        prev_trajectories_col[j].emplace_back(j, i + k);
                    }
                } else {
                    for (int k{0}; k < batch_size; ++k) {
                        wallMap[j][i + k] = 1;
                        prev_trajectories_col[j].emplace_back(j, i + k);
                    }
                    prev_col_batch[j] = true;
                }
            } else {
                if (prev_col_batch[j]) {
                    trajectories.emplace_back(prev_trajectories_col[j]);
                    prev_trajectories_col[j].clear();
                    prev_col_batch[j] = false;
                }
            }
        }
    }
    // TODO: what about other directions?
}


struct WallFollowerController : public Controller {

    struct WallFollowerUserData {
        std::vector<std::vector<int>> map;
        int min_wall_length {10};
    };

    WallFollowerController() {
        type = ControllerType::GENERATOR;
    }

    ~WallFollowerController() {
        delete static_cast<WallFollowerUserData*>(user_data);
    }

    void init(std::vector<std::vector<int>>& map,
              const std::shared_ptr<ActionSpace>& action_space_ptr,
              int min_wall_length = 10) {
        auto* user = new WallFollowerUserData();
        user->map = map;
        user->min_wall_length = min_wall_length;
        this->user_data = user;
        this->as_ptr = action_space_ptr;
    }
};

/// @brief Controller that follows walls
inline std::vector<ActionSequence> ControllerWallsFollower(void* user,
                                                           const std::shared_ptr<ims::ActionSpace>& action_space_ptr) {
    // Try to cast the user data. If it doesn't work, throw error
    auto* user_data = static_cast<WallFollowerController::WallFollowerUserData*>(user);

    std::vector<ActionSequence> generated;
    std::vector<std::vector<int>> wallMap;
    std::vector<std::vector<std::pair<int, int>>> trajectories;
    detectWalls(user_data->map, wallMap, trajectories, user_data->min_wall_length);
    generated.resize(trajectories.size());
    for (int i{0}; i < trajectories.size(); ++i) {
        ActionSequence action_seq;
        for (int j{0}; j < trajectories[i].size(); ++j) {
            action_seq.push_back({static_cast<double>(trajectories[i][j].first),
                                  static_cast<double>(trajectories[i][j].second)});
        }
        generated[i] = action_seq;
    }
    return generated;
};


/// @brief Linear controller
struct LinearController : public Controller {

    explicit LinearController(ControllerType controller_type) {
        type = controller_type;
        if (controller_type == ControllerType::CONNECTOR) {
            two_p_bvp = true;
        }
    }

    struct LinearUserData {
        Eigen::Vector2d start;
        Eigen::Vector2d goal;
        double step_size{};
    };

    ~LinearController() {
        delete static_cast<LinearUserData*>(user_data);
    }

    void init(double step_size,
              const std::shared_ptr<ActionSpace>& action_space_ptr,
              const std::vector<double>& start,
              const std::vector<double>& goal) {
        // make sure the no memory leak. If the user data is not null, delete it.
        auto* user = new LinearUserData();
        user->start = {start[0], start[1]};
        user->goal = {goal[0], goal[1]};
        user->step_size = step_size;
        this->user_data = user;
        this->as_ptr = action_space_ptr;
    }

    void reinit(const std::vector<double>& start,
                const std::vector<double>& goal) {
        auto* user = static_cast<LinearUserData*>(user_data);
        user->start = {start[0], start[1]};
        user->goal = {goal[0], goal[1]};
    }
};

/// @brief Linear controller function
inline std::vector<ActionSequence> LinearControllerFn(void* user,
                                                      const std::shared_ptr<ims::ActionSpace>& action_space_ptr) {
    // Try to cast the user data. If it doesn't work, throw error
    auto* user_data = static_cast<LinearController::LinearUserData*>(user);

    std::vector<ActionSequence> generated;
    Eigen::Vector2d start = user_data->start;
    Eigen::Vector2d goal = user_data->goal;
    double step_size = user_data->step_size;
    Eigen::Vector2d diff = goal - start;
    double dist = diff.norm();
    int num_steps = (int) (dist / step_size);
    Eigen::Vector2d step = diff / num_steps;
    ActionSequence action_seq;
    for (int i{0}; i < num_steps; ++i) {
        Eigen::Vector2d new_state = start + i * step;
        action_seq.push_back({new_state[0], new_state[1]});
    }
    generated.push_back(action_seq);
    return generated;
}


/// @brief Weighted A* controller
struct wAStarController : public Controller {

    struct wAStarUserData {
        Eigen::Vector2d start;
        Eigen::Vector2d goal;
        std::shared_ptr<Scene2DRob> scene;
    };

    wAStarController(ControllerType controller_type) {
        type = controller_type;
    }

    ~wAStarController() {
        delete static_cast<wAStarUserData*>(user_data);
    }

    void init(const std::vector<double>& start,
              const std::vector<double>& goal,
              const std::shared_ptr<ActionSpaceMosaic>& action_space_ptr,
              std::shared_ptr<Scene2DRob> scene) {
        // make sure the no memory leak. If the user data is not null, delete it.
        auto* user = new wAStarUserData();
        user->start = {start[0], start[1]};
        user->goal = {goal[0], goal[1]};
        this->user_data = user;
        this->as_ptr = action_space_ptr;
        user->scene = std::move(scene);
    }

    void reinit(const std::vector<double>& start,
                const std::vector<double>& goal) {
        auto* user = static_cast<wAStarUserData*>(user_data);
        user->start = {start[0], start[1]};
        user->goal = {goal[0], goal[1]};
    }

};

/// @brief Weighted A* controller function
inline std::vector<ActionSequence> wAStarControllerFn(void* user,
                                                      const std::shared_ptr<ims::ActionSpace>& action_space_ptr) {
    // Try to cast the user data. If it doesn't work, throw error
    auto* user_data = static_cast<wAStarController::wAStarUserData*>(user);
    auto* action_space_ptr_mosaic = dynamic_cast<ims::ActionSpaceMosaic*>(action_space_ptr.get());
    // get the closest trajectories from action_space_ptr
    std::pair<int, int> traj_pair;
    std::pair<Eigen::VectorXd , Eigen::VectorXd> closest_states;
    for (int i{0}; i < action_space_ptr_mosaic->mosaic_trajectories_.size(); i++) {
        Eigen::MatrixXd traj = action_space_ptr_mosaic->mosaic_trajectories_[i]->trajectory;
        for (int j {0}; j < action_space_ptr_mosaic->mosaic_trajectories_.size(); j++){
            if (action_space_ptr_mosaic->areTrajectoriesConnected(i, j) || i == j) {
                continue;
            }
            Eigen::MatrixXd traj2 = action_space_ptr_mosaic->mosaic_trajectories_[j]->trajectory;
            // find the minimum distance between the two trajectories.
            double min_dist = std::numeric_limits<double>::max();
            for (int k{0}; k < traj.rows(); k++){
                for (int l{0}; l < traj2.rows(); l++){
                    double dist = (traj.row(k) - traj2.row(l)).norm();
                    if (dist < min_dist){
                        min_dist = dist;
                        traj_pair = {i, j};
                        closest_states = {traj.row(k), traj2.row(l)};
                    }
                }
            }
        }
    } // TODO: Fix it. There is a problem here because we need to keep track of the connected trajs better.

    if (closest_states.first.size() == 0 || closest_states.second.size() == 0) {
        std::cout << RED << "Could not find a valid pair of trajectories" << RESET << std::endl;
        return {};
    }

//    // Lets randomly sample trajs
//    do {
//        traj_pair = {rand() % action_space_ptr_mosaic->mosaic_trajectories_.size(),
//                     rand() % action_space_ptr_mosaic->mosaic_trajectories_.size()};
//    } while (action_space_ptr_mosaic->areTrajectoriesConnected(traj_pair.first, traj_pair.second) ||
//             traj_pair.first == traj_pair.second);

    // TODO: multi-start multi-goal
//    Eigen::MatrixXd traj = action_space_ptr_mosaic->mosaic_trajectories_[traj_pair.first]->trajectory;
//    Eigen::MatrixXd traj2 = action_space_ptr_mosaic->mosaic_trajectories_[traj_pair.second]->trajectory;
//    // find the minimum distance between the two trajectories.
//    double min_dist = std::numeric_limits<double>::max();
//
//    for (int k{0}; k < traj.rows(); k++){
//        for (int l{0}; l < traj2.rows(); l++){
//            double dist = (traj.row(k) - traj2.row(l)).norm();
//            if (dist < min_dist){
//                min_dist = dist;
//                closest_states = {traj.row(k), traj2.row(l)};
//            }
//        }
//    }

    auto* heuristic = new ims::EuclideanHeuristic();
    double epsilon = 100.0;
    ims::wAStarParams params (heuristic, epsilon);

    params.time_limit_ = 0.1;
    ActionType2dRob action_type;
    std::shared_ptr<actionSpace2dRob> as = std::make_shared<actionSpace2dRob>(*user_data->scene,
                                                                              action_type);

    ims::wAStar planner(params);
    std::vector<ActionSequence> generated;
//    StateType start = {user_data->start[0], user_data->start[1]};
//    StateType goal = {user_data->goal[0], user_data->goal[1]};
    StateType start = {closest_states.first[0], closest_states.first[1]};
    StateType goal = {closest_states.second[0], closest_states.second[1]};

//    std::cout << "Trajectories: " << traj_pair.first << ", " << traj_pair.second << std::endl;
//    std::cout << "Start: " << start[0] << ", " << start[1] << std::endl;
//    std::cout << "Goal: " << goal[0] << ", " << goal[1] << std::endl;

    try {
        planner.initializePlanner(as, start, goal);
    }
    catch (std::exception& e) {
        std::cout << RED << "Start or goal is not valid!" <<RESET << std::endl;
        return generated;
    }
    std::vector<StateType> path;
    if (!planner.plan(path)) {
        std::cout << RED << "Could not plan" << RESET << std::endl;
        delete heuristic;
        return generated;
    }
    generated.push_back(path);
    action_space_ptr_mosaic->connected_trajectories_.emplace_back(traj_pair);
    // delete the heuristic
    delete heuristic;
    return generated;
}

} // namespace ims

