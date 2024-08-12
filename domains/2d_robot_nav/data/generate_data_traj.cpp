/*
 * Copyright (C) 2024, Itamar Mishani
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Carnegie Mellon University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/*!
 * \file   generate_data_traj.cpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   8/1/24
*/


#include <boost/filesystem.hpp>
#include <vector>
#include <memory>
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <random>
#include <hdf5.h>

// project includes
#include <search/planners/astar.hpp>
#include <search/heuristics/standard_heuristics.hpp>

#include "../action_space_2d_rob.hpp"
#include "../utils.hpp"



/// @brief generate a start and goal pair
/// @param scene the scene
/// @param action_space the action space
/// @param start_goal_dis the maximum distance between the start and the goal
std::pair<StateType, StateType> generateStartGoal(const Scene2DRob& scene,
                                                  const std::shared_ptr<actionSpace2dRob>& action_space,
                                                  int start_goal_dis) {
    std::vector<double> start, goal;
    std::random_device rd;
    std::mt19937 gen(rd());
    while (true) {
        // generate a random start and goal
        start = {std::round(gen() % scene.map_size.at(0)), std::round(gen() % scene.map_size.at(1))};
        goal = {std::round(gen() % scene.map_size.at(0)), std::round(gen() % scene.map_size.at(1))};
        // check if the start and goal are valid
        if (action_space->isStateValid(start) && action_space->isStateValid(goal)) {
            // check if the distance between the start and the goal is greater than the minimum distance
            if (std::sqrt(std::pow(start[0] - goal[0], 2) + std::pow(start[1] - goal[1], 2)) > start_goal_dis) {
                break;
            }
        }
    }
    return {start, goal};
}



int main(int argc, char** argv) {

    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " <map_file> <scale> <num_traj> <start_goal_dis>" << std::endl;
        return 0;
    }
    std::vector<std::string> maps;

    boost::filesystem::path full_path( boost::filesystem::current_path() );
    std::cout << "Current path is : " << full_path.string() << std::endl;
    // At each emplace_back, use the full pathh and concatenate the map name
    maps.emplace_back(full_path.string() + "/../domains/2d_robot_nav/data/hrt201n/hrt201n.map");
    maps.emplace_back(full_path.string() + "/../domains/2d_robot_nav/data/den501d/den501d.map");
    maps.emplace_back(full_path.string() + "/../domains/2d_robot_nav/data/den520d/den520d.map");
    maps.emplace_back(full_path.string() + "/../domains/2d_robot_nav/data/ht_chantry/ht_chantry.map");
    maps.emplace_back(full_path.string() + "/../domains/2d_robot_nav/data/brc203d/brc203d.map");

    std::vector<std::string> dirs = {full_path.string() + "/../domains/2d_robot_nav/data/hrt201n/",
                                     full_path.string() + "/../domains/2d_robot_nav/data/den501d/",
                                     full_path.string() + "/../domains/2d_robot_nav/data/den520d/",
                                     full_path.string() + "/../domains/2d_robot_nav/data/ht_chantry/",
                                     full_path.string() + "/../domains/2d_robot_nav/data/brc203d/",
    };

    int map_index = std::stoi(argv[1]);
    int scale = std::stoi(argv[2]);
    int num_traj = std::stoi(argv[3]);
    int start_goal_dis = std::stoi(argv[4]);

    std::string path = dirs[map_index];
    std::string map_file = maps[map_index];

    std::string type;
    int width, height;
    std::vector<std::vector<int>> map = loadMap(map_file.c_str(), type, width, height, scale);


    // construct the planner
    std::cout << "Constructing planner..." << std::endl;
    // construct planner params
    auto* heuristic = new ims::EuclideanHeuristic();
    // initialize the heuristic
    ims::AStarParams params (heuristic);
    params.verbose = false;
    // construct the scene and the action space
    Scene2DRob scene (map);
    ActionType2dRob action_type;

    // log the results
    std::unordered_map<int, PlannerStats> logs;
    std::unordered_map<int, PathType> paths;
    std::vector<PathType> paths_to_save;
    int less_than_64 = 0;
    for (int i {0}; i < num_traj; i++){
        std::shared_ptr<actionSpace2dRob> ActionSpace = std::make_shared<actionSpace2dRob>(scene, action_type);
        // generate a start and goal pair
        std::pair<StateType, StateType> start_goal = generateStartGoal(scene, ActionSpace, start_goal_dis);
        // round the start and goal to the nearest integer
        std::cout << "Start: " << start_goal.first[0] << ", " << start_goal.first[1] << std::endl;
        std::cout << "Goal: " << start_goal.second[0] << ", " << start_goal.second[1] << std::endl;
        StateType start = start_goal.first;
        StateType goal = start_goal.second;
        for (int j {0}; j < 2; j++){
            start[j] = std::round(start[j]);
            goal[j] = std::round(goal[j]);
        }
        std::cout << "Rounded Start: " << start[0] << ", " << start[1] << std::endl;
        std::cout << "Rounded Goal: " << goal[0] << ", " << goal[1] << std::endl;

        // print the value in the map
        std::cout << "Start value: " << map[(int)start[0]][(int)start[1]] << std::endl;
        std::cout << "Goal value: " << map[(int)goal[0]][(int)goal[1]] << std::endl;


        // construct planner
        ims::AStar planner(params);
        // catch the exception if the start or goal is not valid
        try {
            planner.initializePlanner(ActionSpace, start, goal);
        }
        catch (std::exception& e) {
            std::cout << RED << e.what() << RESET << std::endl;
            continue;
        }
        // plan
        std::cout << "Planning..." << std::endl;
        std::vector<StateType> path_;
        if (!planner.plan(path_)) {
            std::cout << RED << "No path found!" << RESET << std::endl;
            continue;
        }
        else {
            std::cout << GREEN << "Path found!" << RESET << std::endl;
        }

        PlannerStats stats = planner.reportStats();
        std::cout << GREEN << "Planning time: " << stats.time << " sec" << std::endl;
        std::cout << "cost: " << stats.cost << std::endl;
        std::cout << "Path length: " << path_.size() << std::endl;
        if (path_.size() < 64) {
            less_than_64++;
            i--;
            continue;
        }
        std::cout << "Number of nodes expanded: " << stats.num_expanded << std::endl;
        std::cout << "Number of nodes generated: " << stats.num_generated << std::endl;
        std::cout << "suboptimality: " << stats.suboptimality << RESET << std::endl;
        logs[i] = stats; // log the stats
//        paths[i] = path_; // log the path
        // push the path to the paths to save, but only the 64 first steps from each path
        paths_to_save.emplace_back(path_.begin(), path_.begin() + 64);
        paths[i] = paths_to_save.back(); // log the path
    }

    // save the logs to a temporary file
//    logStats(logs, map_index, "Astar");
    std::cout << "Number of paths with less than 64 steps: " << less_than_64 << std::endl;
    std::string path_file = logPaths(paths, map_index, scale);

    std::string plot_path = full_path.string() + "/../domains/2d_robot_nav/scripts/visualize_paths.py";
    std::string command = "python3 " + plot_path + " --filepath " + path_file + " --animate false";
    std::cout << "Running the plot script..." << std::endl;

    auto boo = system(command.c_str());

    // save the paths to a file
    std::string path_file_ = path + "data/" + "paths.h5";
    hid_t file = H5Fcreate(path_file_.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
    hid_t dataset_id;
    hsize_t dims[3];
    dims[0] = paths_to_save.size();
    dims[1] = 64;
    dims[2] = 2;
    dataset_id = H5Dcreate(file, "trajs", H5T_NATIVE_DOUBLE, H5Screate_simple(3, dims, nullptr), H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);

    std::vector<double> data;
    for (int i {0}; i < paths_to_save.size(); i++){
        for (int j {0}; j < 64; j++){
            data.push_back(paths_to_save[i][j].at(0));
            data.push_back(paths_to_save[i][j].at(1));
        }
    }
    H5Dwrite(dataset_id, H5T_NATIVE_DOUBLE, H5Screate_simple(3, dims, nullptr), H5Screate_simple(3, dims, nullptr), H5P_DEFAULT, data.data());
    // Create metadata group
    hid_t group_id = H5Gcreate(file, "metadata", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    // add data size, horizon, and state dim to the metadata
    hsize_t dims_ = 1;
    hid_t dataspace = H5Screate_simple(1, &dims_, nullptr);
    hid_t dataset_size = H5Dcreate(group_id, "size", H5T_NATIVE_INT, dataspace, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    int size = (int)paths_to_save.size();
    H5Dwrite(dataset_size, H5T_NATIVE_INT, H5S_ALL, H5S_ALL, H5P_DEFAULT, &size);
    H5Dclose(dataset_size);
    H5Sclose(dataspace);

    hsize_t dims_2 = 1;
    hid_t dataspace_ = H5Screate_simple(1, &dims_2, nullptr);
    hid_t dataset_horizon = H5Dcreate(group_id, "horizon", H5T_NATIVE_INT, dataspace_, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    int horizon = 64;
    H5Dwrite(dataset_horizon, H5T_NATIVE_INT, H5S_ALL, H5S_ALL, H5P_DEFAULT, &horizon);
    H5Dclose(dataset_horizon);
    H5Sclose(dataspace_);

    hsize_t dims_3 = 1;
    hid_t dataspace_3 = H5Screate_simple(1, &dims_3, nullptr);
    hid_t dataset_state_dim = H5Dcreate(group_id, "state_dim", H5T_NATIVE_INT, dataspace_3, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    int state_dim = 2;
    H5Dwrite(dataset_state_dim, H5T_NATIVE_INT, H5S_ALL, H5S_ALL, H5P_DEFAULT, &state_dim);
    H5Dclose(dataset_state_dim);
    H5Sclose(dataspace_3);

    // add a subgroup within metadata named domain
    hid_t domain_group = H5Gcreate(group_id, "domain", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    // add the domain name, map index and scale to the domain group
    hid_t domain_dataspace = H5Screate(H5S_SCALAR);
    hid_t domain_dataset = H5Dcreate(domain_group, "name", H5T_C_S1, domain_dataspace, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    H5Dwrite(domain_dataset, H5T_C_S1, H5S_ALL, H5S_ALL, H5P_DEFAULT, "2d_robot_nav");
    H5Dclose(domain_dataset);
    H5Sclose(domain_dataspace);

    hid_t map_index_dataspace = H5Screate(H5S_SCALAR);
    hid_t map_index_dataset = H5Dcreate(domain_group, "map_index", H5T_NATIVE_INT, map_index_dataspace, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    H5Dwrite(map_index_dataset, H5T_NATIVE_INT, H5S_ALL, H5S_ALL, H5P_DEFAULT, &map_index);
    H5Dclose(map_index_dataset);
    H5Sclose(map_index_dataspace);

    H5Dclose(dataset_id);
    H5Fclose(file);


    return 0;
}

