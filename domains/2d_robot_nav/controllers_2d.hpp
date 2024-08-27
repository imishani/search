//
// Created by itamar on 7/6/24.
//

#pragma once

#include "search/common/types.hpp"
#include "search/common/torch_utils.hpp"
#include "search/controllers/base_controller.hpp"
#include "search/planners/wastar.hpp"
#include "search/heuristics/standard_heuristics.hpp"
#include "action_space_2d_rob.hpp"
#include "action_space_2d_rob_mosaic.hpp"

#include <Eigen/Dense>
#include <utility>

namespace ims {
inline Eigen::VectorXf applyDerivativeFilter(const Eigen::VectorXf &row) {
    Eigen::VectorXf filteredRow(row.size());
    for (int i = 1; i < row.size() - 1; ++i) {
        filteredRow[i] = row[i + 1] - row[i];
    }
    filteredRow[0] = row[1] - row[0];
    filteredRow[row.size() - 1] = row[row.size() - 1] - row[row.size() - 2];
    return filteredRow;
}

// Function to apply a 1D derivative filter. Takes in a batch of rows
inline Eigen::MatrixXf applyDerivativeFilterBatch(const Eigen::MatrixXf &rows) {
    Eigen::MatrixXf filteredRows(rows.rows(), rows.cols());
    for (int i = 0; i < rows.rows(); ++i) {
        Eigen::VectorXf row = rows.row(i);
        Eigen::VectorXf filteredRow = applyDerivativeFilter(row);
        filteredRows.row(i) = filteredRow;
    }
    return filteredRows;
}

// Function to detect walls using derivative filters
inline void detectWalls(const std::vector<std::vector<int> > &gridMap,
                        std::vector<std::vector<int> > &wallMap,
                        std::vector<std::vector<std::pair<int, int> > > &trajectories,
                        int batch_size = 10,
                        int stride = 1,
                        double obs_threshold = 100) {
    int rows = (int) gridMap.size();
    int cols = (int) gridMap[0].size();
    wallMap.resize(rows, std::vector<int>(cols, 0));

    std::vector<bool> prev_row_batch(cols, false);
    std::vector<std::vector<std::pair<int, int> > > prev_trajectories(cols, std::vector<std::pair<int, int> >());
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
    std::vector<std::vector<std::pair<int, int> > > prev_trajectories_col(rows,
                                                                          std::vector<std::pair<int, int> >());
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
        std::vector<std::vector<int> > map;
        int min_wall_length{10};
    };

    WallFollowerController() {
        type = ControllerType::GENERATOR;
    }

    ~WallFollowerController() {
        delete static_cast<WallFollowerUserData *>(user_data);
    }

    void init(std::vector<std::vector<int> > &map,
              const std::shared_ptr<ActionSpace> &action_space_ptr,
              int min_wall_length = 10) {
        auto *user = new WallFollowerUserData();
        user->map = map;
        user->min_wall_length = min_wall_length;
        this->user_data = user;
        this->as_ptr = action_space_ptr;
    }
};

/// @brief Controller that follows walls
inline std::vector<ActionSequence> ControllerWallsFollower(void *user,
                                                           const std::shared_ptr<ims::ActionSpace> &action_space_ptr) {
    // Try to cast the user data. If it doesn't work, throw error
    auto *user_data = static_cast<WallFollowerController::WallFollowerUserData *>(user);

    std::vector<ActionSequence> generated;
    std::vector<std::vector<int> > wallMap;
    std::vector<std::vector<std::pair<int, int> > > trajectories;
    detectWalls(user_data->map, wallMap, trajectories, user_data->min_wall_length);
    generated.resize(trajectories.size());
    for (int i{0}; i < trajectories.size(); ++i) {
        ActionSequence action_seq;
        for (int j{0}; j < trajectories[i].size(); ++j) {
            action_seq.push_back({
                static_cast<double>(trajectories[i][j].first),
                static_cast<double>(trajectories[i][j].second)
            });
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
        delete static_cast<LinearUserData *>(user_data);
    }

    void init(double step_size,
              const std::shared_ptr<ActionSpace> &action_space_ptr,
              const std::vector<double> &start,
              const std::vector<double> &goal) {
        // make sure the no memory leak. If the user data is not null, delete it.
        auto *user = new LinearUserData();
        user->start = {start[0], start[1]};
        user->goal = {goal[0], goal[1]};
        user->step_size = step_size;
        this->user_data = user;
        this->as_ptr = action_space_ptr;
    }

    void reinit(const std::vector<double> &start,
                const std::vector<double> &goal) {
        auto *user = static_cast<LinearUserData *>(user_data);
        user->start = {start[0], start[1]};
        user->goal = {goal[0], goal[1]};
    }
};

/// @brief Linear controller function
inline std::vector<ActionSequence> LinearControllerFn(void *user,
                                                      const std::shared_ptr<ims::ActionSpace> &action_space_ptr) {
    // Try to cast the user data. If it doesn't work, throw error
    auto *user_data = static_cast<LinearController::LinearUserData *>(user);

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
struct wAStarController : Controller {
    struct wAStarUserData {
        Eigen::Vector2d start;
        Eigen::Vector2d goal;
        std::shared_ptr<Scene2DRob> scene;
    };

    explicit wAStarController(ControllerType controller_type) {
        type = controller_type;
    }

    ~wAStarController() {
        delete static_cast<wAStarUserData *>(user_data);
    }

    void init(const std::vector<double> &start,
              const std::vector<double> &goal,
              const std::shared_ptr<ActionSpaceMosaic> &action_space_ptr,
              std::shared_ptr<Scene2DRob> scene) {
        // make sure the no memory leak. If the user data is not null, delete it.
        auto *user = new wAStarUserData();
        user->start = {start[0], start[1]};
        user->goal = {goal[0], goal[1]};
        this->user_data = user;
        this->as_ptr = action_space_ptr;
        user->scene = std::move(scene);
    }

    void reinit(const std::vector<double> &start,
                const std::vector<double> &goal) {
        auto *user = static_cast<wAStarUserData *>(user_data);
        user->start = {start[0], start[1]};
        user->goal = {goal[0], goal[1]};
    }
};

/// @brief Weighted A* controller function
inline std::vector<ActionSequence> wAStarControllerFn(void *user,
                                                      const std::shared_ptr<ims::ActionSpace> &action_space_ptr) {
    // time the controller using chrono
    auto start_time = std::chrono::high_resolution_clock::now();
    // Try to cast the user data. If it doesn't work, throw error
    auto *user_data = static_cast<wAStarController::wAStarUserData *>(user);
    auto *action_space_ptr_mosaic = dynamic_cast<ims::ActionSpaceMosaic *>(action_space_ptr.get());
    // get the closest trajectories from action_space_ptr
    std::pair<int, int> traj_pair;
    std::pair<Eigen::VectorXd, Eigen::VectorXd> closest_states;

    ///////////////////// Find the closest pair of trajectories /////////////////////
    // Surprisingly, this takes most of the time. TODO: We need to optimize this.
    for (int i{0}; i < action_space_ptr_mosaic->mosaic_trajectories_.size(); i++) {
        Eigen::MatrixXd traj = action_space_ptr_mosaic->mosaic_trajectories_[i]->trajectory;
        for (int j{i}; j < action_space_ptr_mosaic->mosaic_trajectories_.size(); j++) {
            if (action_space_ptr_mosaic->areTrajectoriesConnected(i, j) ||
                action_space_ptr_mosaic->areTrajectoriesUnconnectable(i, j) || (i == j)) {
                continue;
            }
            Eigen::MatrixXd traj2 = action_space_ptr_mosaic->mosaic_trajectories_[j]->trajectory;
            // find the minimum distance between the two trajectories.
            double min_dist = std::numeric_limits<double>::max();
            for (int k{0}; k < traj.rows(); k++) {
                for (int l{0}; l < traj2.rows(); l++) {
                    double dist = (traj.row(k) - traj2.row(l)).norm();
                    if (dist < min_dist) {
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

    ////////////////////////////////////////////////////////////////////////////////////

    //    // Lets randomly sample trajs
    //    do {
    //        traj_pair = {rand() % action_space_ptr_mosaic->mosaic_trajectories_.size(),
    //                     rand() % action_space_ptr_mosaic->mosaic_trajectories_.size()};
    //    } while (action_space_ptr_mosaic->areTrajectoriesConnected(traj_pair.first, traj_pair.second) ||
    //             traj_pair.first == traj_pair.second);
    //
    //    // TODO: multi-start multi-goal. It is already implemented just need to decide how to do it here.
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

    auto *heuristic = new ims::EuclideanHeuristic();
    double epsilon = 100.0;
    ims::wAStarParams params(heuristic, epsilon);

    params.time_limit_ = 0.005;
    params.verbose = false;
    ActionType2dRob action_type;
    std::shared_ptr<actionSpace2dRob> as = std::make_shared<actionSpace2dRob>(*user_data->scene,
                                                                              action_type);

    ims::wAStar planner(params);
    std::vector<ActionSequence> generated;
    StateType start = {closest_states.first[0], closest_states.first[1]};
    StateType goal = {closest_states.second[0], closest_states.second[1]};

    try {
        planner.initializePlanner(as, start, goal);
    } catch (std::exception &e) {
        std::cout << RED << e.what() << RESET << std::endl;
        return generated;
    }
    std::vector<StateType> path;
    if (!planner.plan(path)) {
        std::cout << RED << "Could not plan" << RESET << std::endl;
        action_space_ptr_mosaic->unconnectable_trajectories_.emplace_back(traj_pair);
        delete heuristic;
        return generated;
    }
    generated.push_back(path);
    action_space_ptr_mosaic->connected_trajectories_.emplace_back(traj_pair);
    // delete the heuristic
    delete heuristic;

    // time the controller using chrono
    auto end_time = std::chrono::high_resolution_clock::now();
    // milliseconds
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "Planning time: " << duration.count() << " ms" << std::endl;

    return generated;
}

struct VINController : Controller {
    struct VINUserData {
        std::shared_ptr<Scene2DRob> scene;
        // StateType start;
        // StateType goal;
        int number_of_subregions;
        std::pair<int, int> subregion_size;
        int number_of_trajectories;
        int max_horizon;
        int num_iters;
        torch::jit::script::Module model;
        torch::Device device{torch::kCPU};
    };

    /// @brief Constructor
    explicit VINController(ControllerType controller_type) {
        type = controller_type;
    }

    ///@brief Destructor
    ~VINController() {
        delete static_cast<VINUserData *>(user_data);
    }

    /// @brief Initialize the controller
    /// @param model_path The path to the model.
    /// @param action_space_ptr The action space pointer.
    /// @param scene The scene.
    /// @param number_of_subregions The number of subregions to sample from.
    /// @param subregion_size The size of the subregions.
    /// @param max_horizon The maximum horizon.
    /// @param number_of_trajectories
    /// @param num_iters The number of iterations for the value iteration (K).
    /// @param start The start state.
    /// @param goal The goal state.
    /// @param device The device to run the model on.
    void init(const std::string &model_path,
              const std::shared_ptr<ActionSpaceMosaic> &action_space_ptr,
              std::shared_ptr<Scene2DRob> scene,
              int number_of_subregions = 30,
              const std::pair<int, int> &subregion_size = {28, 28},
              int number_of_trajectories = 50,
              int max_horizon = 50,
              int num_iters = 36,
              // const StateType &start ={},
              // const StateType &goal = {},
              const std::string &device = "") {
        auto *user = new VINUserData();
        user->scene = std::move(scene);
        if (device == "cuda") {
            user->device = torch::kCUDA;
        } else if (device == "cpu") {
            user->device = torch::kCPU;
        } else {
            std::cout << RED << "Invalid device. Using system capabilities" << RESET << std::endl;
            user->device = torch_utils::getDevice();
        }

        torch_utils::loadTorchModel(model_path, user->model, user->device);
        user->model.to(user->device);
        convert2DMapToTensor(user->scene->map, map_tensor);

        // user->start = start;
        // user->goal = goal;
        user->max_horizon = max_horizon;
        user->number_of_subregions = number_of_subregions;
        user->subregion_size = subregion_size;
        user->number_of_trajectories = number_of_trajectories;
        user->num_iters = num_iters;
        this->user_data = user;
        this->as_ptr = action_space_ptr;
    }

    torch::Tensor map_tensor;
};

/// @brief VIN controller function
inline std::vector<ActionSequence> VINControllerFn(void *user,
                                                   const std::shared_ptr<ActionSpace> &action_space_ptr) {
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    auto *user_data = static_cast<VINController::VINUserData *>(user);
    auto action_type = ActionType2dRob();
    auto *action_space_ptr_mosaic = dynamic_cast<ims::ActionSpaceMosaic *>(action_space_ptr.get());

    // sample subregions
    std::vector<torch::Tensor> subregions;
    std::vector<StateType> subregion_frames;
    std::random_device rnd;
    std::mt19937 gen(rnd());
    std::uniform_int_distribution<> x_dist(0, user_data->scene->map_size[0] - user_data->subregion_size.first);
    std::uniform_int_distribution<> y_dist(0, user_data->scene->map_size[1] - user_data->subregion_size.second);
    for (int i{0}; i < user_data->number_of_subregions; ++i) {
        StateType center;
        do {
            int x = x_dist(gen);
            int y = y_dist(gen);
            center = {x, y};
        } while (!action_space_ptr->isStateValid(center));
        subregion_frames.push_back(center);
        torch::Tensor subregion = torch::zeros({
            static_cast<long>(user_data->scene->map_size[0]),
            static_cast<long>(user_data->scene->map_size[1])
        });
        for (int j{0}; j < user_data->subregion_size.first; ++j) {
            for (int k{0}; k < user_data->subregion_size.second; ++k) {
                subregion[j + center[0]][k + center[1]] = user_data->scene->map->at(j + center[0]).at(
                    k + center[1]);
            }
        }
        subregion.to(user_data->device);
        subregions.push_back(subregion);
    }
    std::vector<ActionSequence> trajectories;
    for (int i{0}; i < user_data->number_of_trajectories; ++i) {
        // sample a subregion
        int subregion_idx = std::rand() % user_data->number_of_subregions;
        // sample the start and goal
        std::random_device rd;
        std::mt19937 generator(rd());
        std::uniform_int_distribution<> dis(0, user_data->subregion_size.first - 1);
        std::uniform_int_distribution<> dis2(0, user_data->subregion_size.second - 1);
        StateType start;
        do {
            start = {
                subregion_frames[subregion_idx][0] + dis(generator),
                subregion_frames[subregion_idx][1] + dis2(generator)
            };
        } while (!action_space_ptr->isStateValid(start));
        StateType goal;
        do {
            goal = {
                subregion_frames[subregion_idx][0] + dis(generator),
                subregion_frames[subregion_idx][1] + dis2(generator)
            };
        } while (!action_space_ptr->isStateValid(goal));

        StateType start_transformed = {
            start[0] - subregion_frames[subregion_idx][0],
            start[1] - subregion_frames[subregion_idx][1]
        };
        StateType goal_transformed = {
            goal[0] - subregion_frames[subregion_idx][0],
            goal[1] - subregion_frames[subregion_idx][1]
        };

        torch::Tensor value_prior = torch::full({
                                                    subregions.at(subregion_idx).size(0),
                                                    subregions.at(subregion_idx).size(1)
                                                },
                                                -1);
        value_prior[static_cast<long>(goal_transformed[0])][static_cast<long>(goal_transformed[1])] = 10;
        torch::Tensor input = torch::cat({
                                             subregions.at(subregion_idx).unsqueeze(0).unsqueeze(0),
                                             value_prior.unsqueeze(0).unsqueeze(0)
                                         },
                                         1);
        auto state_x = torch::zeros({1});
        auto state_y = torch::zeros({1});
        state_x[0] = start_transformed[0];
        state_y[0] = start_transformed[1];
        // to device
        state_x = state_x.to(user_data->device);
        state_y = state_y.to(user_data->device);
        input = input.to(user_data->device);
        torch::Tensor k = torch::tensor({user_data->num_iters}).to(user_data->device);
        ActionSequence action_seq = {{start[0], start[1]}};
        for (int t{0}; t < user_data->max_horizon; t++) {
            std::vector<torch::jit::IValue> inputs;
            inputs.emplace_back(input);
            inputs.emplace_back(state_x);
            inputs.emplace_back(state_y);
            inputs.emplace_back(k);
            // forward pass;
            auto output = user_data->model.forward(inputs).toTuple();
            // Get the action probabilities (the second element in the tuple)
            auto action_probs = output->elements()[1].toTensor();
            // Get the max action
            auto max_action = action_probs.max(1);
            int action_index = std::get<1>(max_action).item<int>();
            auto action = action_space_ptr_mosaic->action_type_->getPrimActions().at(action_index);

            // Get the next state
            auto next_state = torch::zeros({2});
            next_state[0] = state_x[0] + action[0];
            next_state[1] = state_y[0] + action[1];
            if (action_space_ptr_mosaic->isStateValid({
                next_state[0].item<double>() + subregion_frames[subregion_idx][0],
                next_state[1].item<double>() + subregion_frames[subregion_idx][1]})) {
                action_seq.emplace_back(std::vector{
                    next_state[0].item<double>() + subregion_frames[subregion_idx][0],
                    next_state[1].item<double>() + subregion_frames[subregion_idx][1]
                });
                // check if the goal is reached
                if (next_state[0].item<double>() == goal_transformed[0] &&
                    next_state[1].item<double>() == goal_transformed[1]) {
                    // paths[i] = path;
                    std::cout << GREEN << "Goal reached!" << RESET << std::endl;
                    break;
                }
                state_x = next_state[0].unsqueeze(0);
                state_y = next_state[1].unsqueeze(0);
            } else {
                std::cout << RED << "Invalid state" << RESET << std::endl;
                break;
            }
        }
        if (action_seq.empty()) {
            std::cout << RED << "Could not find a valid trajectory" << RESET << std::endl;
        } else {
            trajectories.push_back(action_seq);
        }
    }
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()
        << "[ms]" << std::endl;
    // Save to tmp file the regions, and generated trjectories

    std::ofstream file;
    file.open("../domains/2d_robot_nav/scripts/regions.txt");
    for (int i{0}; i < subregions.size(); ++i) {
        file << subregion_frames[i][0] << " " << subregion_frames[i][1] << std::endl;
    }
    file.close();
    file.open("../domains/2d_robot_nav/scripts/vin_trajectories.txt");
    for (int i{0}; i < trajectories.size(); ++i) {
        for (int j{0}; j < trajectories[i].size(); ++j) {
            file << trajectories[i][j][0] << " " << trajectories[i][j][1] << std::endl;
        }
        file << std::endl;
    }
    file.close();

    return trajectories;

}
} // namespace ims
