//
// Created by itamar on 7/7/24.
//

#include "search/planners/mosaic.hpp"

ims::Mosaic::Mosaic(const ims::MosaicParams &params)  : Planner(params), params_(params) {}

ims::Mosaic::~Mosaic() {
    for (auto trajectory : mosaic_trajectories_) {
        delete trajectory;
    }
    for (auto state : states_) {
        delete state;
    }
}

void ims::Mosaic::initializePlanner(const std::shared_ptr<ActionSpaceMosaic> &action_space_ptr,
                                    std::shared_ptr<std::vector<Controller>> &controllers,
                                    const StateType &start,
                                    const StateType &goal) {
    // Action space pointer.
    action_space_ptr_ = action_space_ptr;
    action_space_ptr_->resetPlanningData();
    resetPlanningData();
    // Set the controllers
    controllers_ = controllers;


    // Check if start is valid and add it to the action space.
    // check if start is valid
    if (!action_space_ptr_->isStateValid(start)){
        std::cout << "Start state is not valid" << std::endl;
        throw std::runtime_error("Start state is not valid");
    }
    int start_ind_ = action_space_ptr_->getOrCreateRobotState(start);
    start_ = getOrCreateGraphState(start_ind_);
    start_->is_connected_to_start = true;
    // save the start state as a trajectory
    start_traj_ = createTrajectory({start_ind_});
    start_->in_trajs->push_back(start_traj_->id);


    if (!action_space_ptr_->isStateValid(goal)){
        std::cout << "Goal state is not valid" << std::endl;
        throw std::runtime_error("Goal state is not valid");
    }
    int goal_ind_ = action_space_ptr_->getOrCreateRobotState(goal);
    goal_ = getOrCreateGraphState(goal_ind_);
    goal_->is_connected_to_goal = true;
    // save the goal state as a trajectory
    goal_traj_ = createTrajectory({goal_ind_});
    goal_->in_trajs->push_back(goal_traj_->id);

//    saveData();
}

auto ims::Mosaic::getGraphState(int state_id) -> ims::Mosaic::GraphState*{
    assert(state_id < states_.size() && state_id >= 0);
    return states_[state_id];
}

auto ims::Mosaic::getOrCreateGraphState(int state_id) -> ims::Mosaic::GraphState* {
    if (state_id >= states_.size()){
        states_.resize(state_id + 1, nullptr);
    }
    if (states_[state_id] == nullptr){
        assert(state_id < states_.size() && state_id >= 0);
        states_[state_id] = new GraphState;
        states_[state_id]->state_id = state_id;
        states_[state_id]->in_trajs = std::make_shared<std::vector<int>>();
        states_[state_id]->edges = std::make_shared<std::vector<std::pair<int, double>>>();
    }
    return states_[state_id];
}

auto ims::Mosaic::createTrajectory(const std::vector<int> &trajectory) -> TrajectoryState*{
    int id = action_space_ptr_->createTrajectory(trajectory);
    if (id >= mosaic_trajectories_.size()) {
        mosaic_trajectories_.resize(id + 1, nullptr);
    }
    if (mosaic_trajectories_[id] == nullptr) {
        mosaic_trajectories_[id] = new TrajectoryState;
        mosaic_trajectories_[id]->id = id;
        mosaic_trajectories_[id]->trajectory = trajectory;
    }
    return mosaic_trajectories_[id];
}

auto ims::Mosaic::getTrajectory(int trajectory_id) -> ims::Mosaic::TrajectoryState * {
    assert(trajectory_id < mosaic_trajectories_.size());
    return mosaic_trajectories_[trajectory_id];
}

bool ims::Mosaic::areTrajectoriesConnected(int traj1_id, int traj2_id) {
    TrajectoryState* traj1 = mosaic_trajectories_[traj1_id];
    // check if traj2_id is in traj1 edges
    std::any_of(traj1->edges.begin(), traj1->edges.end(), [traj2_id](const std::pair<int, int>& edge) {
        return edge.first == traj2_id;
    });
    return false;
}


bool ims::Mosaic::plan(std::vector<StateType> &path) {
    startTimer();
    generate();

    while (!isTimeOut()) {

        if (isGoalConditionSatisfied(trajs_path_)){
            getTimeFromStart(stats_.time);
            std::cout << "Goal found!" << std::endl;
//            reconstructPath(path, stats_.transition_costs);
            reconstructPath(path, trajs_path_);
            stats_.cost = 0;
//            for (auto& cost : stats_.transition_costs){
//                stats_.cost += cost;
//            }
            stats_.path_length = (int)path.size();
            stats_.num_generated = (int)action_space_ptr_->states_.size();
            saveData();
            return true;
        }

        for (const auto& controller : *controllers_) {
            if (controller.type != ControllerType::CONNECTOR) {
                continue;
            }
            std::vector<ActionSequence> local_mosaic = controller.solve();
            int invalid = 0;
            for (auto& traj : local_mosaic) {
                bool goal_connected = false;
                bool start_connected = false;
                std::vector<int> trajectory;
                for (StateType& robot_state : traj) {
                    if (!action_space_ptr_->isStateValid(robot_state)) {
                        break;
                    }
                    int robot_state_ind = action_space_ptr_->getOrCreateRobotState(robot_state);
                    trajectory.push_back(robot_state_ind);
                    if (robot_state_ind == goal_->state_id) {
                        goal_connected = true;
                    }
                    if (robot_state_ind == start_->state_id) {
                        start_connected = true;
                    }
                }

                if (trajectory.size() == 1) {
                    // if the trajectory is only one state, it is a connector between two trajectories.
                    // add the traj_ids that the start state on the trajectory is connected to, to the new traj
                    auto single_state = getOrCreateGraphState(trajectory[0]);
                    for (auto& traj_id1 : *single_state->in_trajs) {
                        for (auto& traj_id2 : *single_state->in_trajs) {
                            if (traj_id1 != traj_id2) {
                                auto traj1 = getTrajectory(traj_id1);
                                traj1->edges.emplace_back(traj_id2, single_state->state_id);
                                // do the same thing in action space TODO: NOT GOOD PRACTICE!
                                auto connected_traj1 = action_space_ptr_->getTrajectory(traj_id1);
                                connected_traj1->edges.emplace_back(traj_id2, single_state->state_id);
                            }
                        }
                    }
                    continue;
                }


                TrajectoryState* traj_ptr = nullptr;
                if (!trajectory.empty()) {
                    traj_ptr = createTrajectory(trajectory);
                    traj_ptr->cost = 1.0;
                } else invalid++;
                for (int i = 0; i < trajectory.size(); i++) {
                    int curr_state = trajectory[i];
                    GraphState* curr_state_ptr = getOrCreateGraphState(curr_state);
                    assert(curr_state_ptr != nullptr);
                    curr_state_ptr->in_trajs->push_back(traj_ptr->id);
                    if (i > 0) {
                        int prev_state = trajectory[i - 1];
                        GraphState* prev_state_ptr = getOrCreateGraphState(prev_state);
                        // TODO: we assume undirected graph. Fix it.
                        curr_state_ptr->edges->emplace_back(prev_state, 1.0);
                        prev_state_ptr->edges->emplace_back(curr_state, 1.0);
                    } else if (i == 0) {
                        // add the traj_ids that the start state on the trajectory is connected to, to the new traj
                        for (auto& traj_id : *curr_state_ptr->in_trajs) {
                            if (traj_id != traj_ptr->id) {
                                traj_ptr->edges.emplace_back(traj_id, curr_state);
                                auto traj_connected = getTrajectory(traj_id);
                                traj_connected->edges.emplace_back(traj_ptr->id, curr_state);
                                // do the same thing in action space TODO: NOT GOOD PRACTICE!
                                auto connected_traj = action_space_ptr_->getTrajectory(traj_id);
                                connected_traj->edges.emplace_back(traj_ptr->id, curr_state);
                                auto curr_traj = action_space_ptr_->getTrajectory(traj_ptr->id);
                                curr_traj->edges.emplace_back(traj_id, curr_state);

                                action_space_ptr_->connected_trajectories_.emplace_back(traj_ptr->id, traj_id);
                            }
                        }
                    } else if (i == trajectory.size() - 1) {
                        // add the traj_ids that the goal is connected to, to the new traj
                        for (auto& traj_id : *curr_state_ptr->in_trajs) {
                            if (traj_id != traj_ptr->id) {
                                traj_ptr->edges.emplace_back(traj_id, goal_->state_id);
                                auto traj_connected = getTrajectory(traj_id);
                                traj_connected->edges.emplace_back(traj_ptr->id, curr_state);
                                // do the same thing in action space TODO: NOT GOOD PRACTICE!
                                auto connected_traj = action_space_ptr_->getTrajectory(traj_id);
                                connected_traj->edges.emplace_back(traj_ptr->id, curr_state);
                                auto curr_traj = action_space_ptr_->getTrajectory(traj_ptr->id);
                                curr_traj->edges.emplace_back(traj_id, curr_state);

                                action_space_ptr_->connected_trajectories_.emplace_back(traj_ptr->id, traj_id);

                            }
                        }
                    }
                    if (goal_connected) {
                        curr_state_ptr->is_connected_to_goal = true;
                    }
                    if (start_connected) {
                        curr_state_ptr->is_connected_to_start = true;
                    }
                }

                if (start_connected && goal_connected) {
                    std::cout << "Start and goal are connected. Reconstruct path." << std::endl;
                    // raise an error since it is not implemented yet. TODO: implement it.
                    path = std::vector<StateType>(trajectory.size());
                    for (int j = 0; j < trajectory.size(); j++) {
                        path[j] = action_space_ptr_->getRobotState(trajectory[j])->state;
                    }
                    saveData();
                    return true;
//                    throw std::runtime_error("Start and goal are connected, but we haven't implemented reconstruct path from a single controller yet.");
                }
            }
        }
    }
    saveData();
    return false;
}

void ims::Mosaic::generate() {
    for (auto &controller : *controllers_) {
        if (controller.type != ControllerType::GENERATOR) {
            continue;
        }
        std::vector<ActionSequence> local_mosaic = controller.solve();
        int invalid = 0; // debugging purposes
        for (auto& traj : local_mosaic) {
            bool goal_connected = false;
            bool start_connected = false;
            std::vector<int> trajectory;
            for (StateType& robot_state : traj) {
                if (!action_space_ptr_->isStateValid(robot_state)) {
                    break;
                }
                int robot_state_ind = action_space_ptr_->getOrCreateRobotState(robot_state);
                trajectory.push_back(robot_state_ind);
                if (robot_state_ind == goal_->state_id) {
                    goal_connected = true;
                }
                if (robot_state_ind == start_->state_id) {
                    start_connected = true;
                }
            }
            TrajectoryState* traj_ptr = nullptr;
            if (!trajectory.empty()) {
                traj_ptr = createTrajectory(trajectory);
                traj_ptr->cost = 1.0;
            } else invalid++;
            for (int i = 0; i < trajectory.size(); i++) {
                int curr_state = trajectory[i];
                GraphState* curr_state_ptr = getOrCreateGraphState(curr_state);
                assert(curr_state_ptr != nullptr);
                curr_state_ptr->in_trajs->push_back(traj_ptr->id);
                if (i > 0) {
                    int prev_state = trajectory[i - 1];
                    GraphState* prev_state_ptr = getOrCreateGraphState(prev_state);
                    // TODO: we assume undirected graph. Fix it.
                    curr_state_ptr->edges->emplace_back(prev_state, 1.0);
                    prev_state_ptr->edges->emplace_back(curr_state, 1.0);
                }
                if (goal_connected) {
                    curr_state_ptr->is_connected_to_goal = true;
                }
                if (start_connected) {
                    curr_state_ptr->is_connected_to_start = true;
                }
                if (start_connected && goal_connected) {
                    std::cout << "Start and goal are connected. Reconstruct path." << std::endl;
                    // raise an error since it is not implemented yet. TODO: implement it.
                    throw std::runtime_error("Start and goal are connected, "
                                             "but we haven't implemented reconstruct path from a "
                                             "single controller yet.");
                }
            }
        }
    }
}

void ims::Mosaic::resetPlanningData() {
    for (auto state : states_) {
        delete state;
    }
    states_.clear();
    for (auto trajectory : mosaic_trajectories_) {
        delete trajectory;
    }
    mosaic_trajectories_.clear();
    start_ = nullptr;
    goal_ = nullptr;
    stats_ = PlannerStats(); // TODO: update to a new struct.
}


void ims::Mosaic::reconstructPath(std::vector<StateType> &path) {
    // raise an error
    throw std::runtime_error("Not the right function to call. Use the other one.");
}

void ims::Mosaic::reconstructPath(std::vector<StateType> &path, const std::vector<int>& trajs_path) { // TODO: fix. it is NOT correct
    // reconstruct the path from the high-level path of trajectories
    int connecting_state;
    // concatenate the trajectories, but make sure we add only the parts based on the connecting states
    for (int i {0}; i < trajs_path.size(); i++) {
        auto traj = getTrajectory(trajs_path[i]);
        if (i == 0) {
            auto traj_p1 = getTrajectory(trajs_path[i + 1]); // TODO: what if there is one traj?
            connecting_state = traj_p1->edges[trajs_path[i]].second;
            std::cout << "init connecting state: " << connecting_state << std::endl;
            for (int j = 0; j < traj->trajectory.size(); j++) {
                path.push_back(action_space_ptr_->getRobotState(traj->trajectory[j])->state);
                if (traj->trajectory[j] == connecting_state) {
                    break;
                }
            }
        } else if (i == trajs_path.size() - 1) {
            bool found = false;
            for (int j = 0; j < traj->trajectory.size(); j++) {
                if (found) {
                    path.push_back(action_space_ptr_->getRobotState(traj->trajectory[j])->state);
                }
                if (traj->trajectory[j] == connecting_state) {
                    found = true;
                }
            }
        } else {
            auto traj_p1 = getTrajectory(trajs_path[i + 1]);
            int next_connecting_state = traj_p1->edges[trajs_path[i]].second;
            bool found = false;
            for (int j = 0; j < traj->trajectory.size(); j++) {
                if (found) {
                    path.push_back(action_space_ptr_->getRobotState(traj->trajectory[j])->state);
                    if (traj->trajectory[j] == connecting_state) {
                        break;
                    }
                }
                if (traj->trajectory[j] == connecting_state) {
                    found = true;
                    std::cout << "found connecting state: " << connecting_state << std::endl;
                    std::cout << "next connecting state: " << next_connecting_state << std::endl;
                }
            }
        }
    }
}

void ims::Mosaic::reconstructPath(std::vector<StateType> &path, std::vector<double> &costs) {
    // raise an error
    throw std::runtime_error("Not implemented yet. Use the other one.");
}

bool ims::Mosaic::isGoalState(int state_id) {
    return false;
}

bool ims::Mosaic::isGoalConditionSatisfied(std::vector<int>& trajs_path) {
    // run search algorithm (dijkstra) where each node is a trajectory and the edges are the connections between them.
    // the start node is the start trajectory and the goal node is the goal trajectory.
    // if there is a path between the start and goal nodes, return true and the high-level path (the sequence of trajectory ids).
    // if there is no path, return false

    // create the open list as a priority queue
    std::priority_queue<std::pair<double, std::pair<int, int>>, std::vector<std::pair<double, std::pair<int, int>>>,
        std::greater<>> open_list;
    std::vector<double> costs(mosaic_trajectories_.size(), std::numeric_limits<double>::max());
    std::vector<int> parents(mosaic_trajectories_.size(), -1);
    open_list.emplace(0, std::make_pair(start_traj_->id, start_->state_id));
    costs[start_traj_->id] = 0;
    while (!open_list.empty()) {
        auto curr = open_list.top();
        open_list.pop();
        int curr_id = curr.second.first;
        if (curr_id == goal_traj_->id) {
            // reconstruct the path
            int curr_parent = curr_id;
            while (curr_parent != -1) {
                trajs_path.push_back(curr_parent);
                curr_parent = parents[curr_parent];
            }
            std::reverse(trajs_path.begin(), trajs_path.end());
            return true;
        }
        for (auto& edge : mosaic_trajectories_[curr_id]->edges) {
            int next_id = edge.first;
            double cost = mosaic_trajectories_[curr_id]->cost;
            if (costs[next_id] > costs[curr_id] + cost) {
                costs[next_id] = costs[curr_id] + cost;
                parents[next_id] = curr_id;
                open_list.emplace(costs[next_id], std::make_pair(next_id, curr_id));
            }
        }
    }
    return false;
}

void ims::Mosaic::saveData() {
    // save all states to a file
    std::ofstream states_file;
    states_file.open("states_mosaic.txt");
    // delete the file if it exists
    if (states_file.is_open()){
        states_file.close();
        std::remove("states_mosaic.txt");
    }
    states_file.open("states_mosaic.txt");
    for (auto& state : states_) {
        states_file << state->state_id;
//        std::cout << "State id: " << state->state_id << std::endl;
        auto state_val = action_space_ptr_->getRobotState(state->state_id)->state;
        for (auto& val : state_val) {
            states_file << "," << val;
        }
        states_file << "\n";
    }
    states_file.close();

    // save trajectories in trajectories directory
    // check if the directory exists. If not, create it. If it does, delete its content
    std::string dir_name = "trajectories";
    if (boost::filesystem::exists(dir_name)){
        boost::filesystem::remove_all(dir_name);
    }
    boost::filesystem::create_directory(dir_name);
    for (auto& traj : mosaic_trajectories_) {
        std::ofstream traj_file;
        std::string traj_file_name = dir_name + "/trajectory_" + std::to_string(traj->id) + ".txt";
        traj_file.open(traj_file_name);
        for (auto& state_id : traj->trajectory) {
            traj_file << state_id;
            auto state_val = action_space_ptr_->getRobotState(state_id)->state;
            for (auto& val : state_val) {
                traj_file << "," << val;
            }
            traj_file << "\n";
        }
        traj_file.close();
    }

    // save the high-level path
    std::ofstream path_file;
    path_file.open("trajs_path_mosaic.txt");
    // delete the file if it exists
    if (path_file.is_open()){
        path_file.close();
        std::remove("trajs_path_mosaic.txt");
    }
    path_file.open("trajs_path_mosaic.txt");
    for (auto& traj_id : trajs_path_) {
        path_file << traj_id << "\n";
    }
    path_file.close();

}


