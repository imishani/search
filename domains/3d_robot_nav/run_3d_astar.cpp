/*
 * Copyright (C) 2024, Julius A.
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
 * \file   run_3d_astar.cpp
 * \author Julius A. (juliusa@cmu.edu)
 * \date   8/26/24
*/

#include <boost/filesystem.hpp>
#include <iostream>
#include <cmath>
#include <memory>
#include <fstream>

// project includes
#include <search/planners/astar.hpp>
#include <search/heuristics/standard_heuristics.hpp>

#include "action_space_3d_rob.hpp"
#include "utils.hpp"

// Color codes for console output
#define RED "\033[0;31m"
#define GREEN "\033[0;32m"
#define YELLOW "\033[0;33m"
#define RESET "\033[0m"

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cout << "Usage: " << argv[0] << " <map_index> <num_runs> <scale> [cache]" << std::endl;
        return 1;
    }

    std::vector<std::string> maps;
    std::vector<std::string> start_goal_files;

    boost::filesystem::path current_path(boost::filesystem::current_path());
    boost::filesystem::path project_root = current_path.parent_path(); // Assuming build is directly under project root
    std::cout << "Project root path is : " << project_root.string() << std::endl;

    boost::filesystem::path data_path = project_root / "domains" / "3d_robot_nav" / "data";

    // Populate maps and start_goal_files
    for (int i = 1; i <= 5; ++i) {
        maps.push_back((data_path / ("map_" + std::to_string(i) + ".map")).string());
        start_goal_files.push_back((data_path / ("start_goal_" + std::to_string(i) + ".txt")).string());
    }

    int map_index = std::stoi(argv[1]) - 1; // Convert to 0-based index
    int num_runs = std::stoi(argv[2]);
    int scale = std::stoi(argv[3]);
    bool cache = false;
    if (argc > 4) {
        cache = std::stoi(argv[4]);
    } else {
        std::cout << YELLOW << "Didn't specify whether to save the path or not. Default is not to save." << RESET << std::endl;
    }

    if (map_index < 0 || map_index >= maps.size()) {
        std::cerr << "Error: Invalid map index. Must be between 1 and " << maps.size() << std::endl;
        return 1;
    }

    std::string map_file = maps[map_index];
    std::string start_goal_file = start_goal_files[map_index];

    std::cout << "Loading map from: " << map_file << std::endl;
    std::cout << "Loading start/goal from: " << start_goal_file << std::endl;

    std::string type;
    int width, height, depth;
    std::vector<std::vector<std::vector<int>>> map;

    if (!loadMap(map_file.c_str(), type, width, height, depth, map, scale)) {
        std::cerr << "Failed to load map. Exiting." << std::endl;
        return 1;
    }

    std::cout << "Map dimensions: " << width << " x " << height << " x " << depth << std::endl;

    std::vector<std::vector<double>> starts, goals;
    try {
        loadStartsGoalsFromFile(starts, goals, scale, num_runs, start_goal_file);
    } catch (const std::exception& e) {
        std::cerr << "Error loading start/goal positions: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "First start position: " << starts[0][0] << ", " << starts[0][1] << ", " << starts[0][2] << std::endl;
    std::cout << "First goal position: " << goals[0][0] << ", " << goals[0][1] << ", " << goals[0][2] << std::endl;

    // construct the planner
    std::cout << "Constructing planner..." << std::endl;
    auto* heuristic = new ims::EuclideanHeuristic();
    ims::AStarParams params(heuristic);
    std::shared_ptr<Scene3DRob> scene = std::make_shared<Scene3DRob>(map);
    ActionType3dRob action_type;

    // log the results
    std::unordered_map<int, PlannerStats> logs;
    std::unordered_map<int, PathType> paths;

    for (int i = 0; i < starts.size(); i++) {
        std::cout << "\nProcessing start/goal pair " << i + 1 << " of " << starts.size() << std::endl;
        std::cout << "Start: " << starts[i][0] << ", " << starts[i][1] << ", " << starts[i][2] << std::endl;
        std::cout << "Goal: " << goals[i][0] << ", " << goals[i][1] << ", " << goals[i][2] << std::endl;

        for (int j = 0; j < 3; j++) {
            starts[i][j] = std::round(starts[i][j]);
            goals[i][j] = std::round(goals[i][j]);
        }

        std::cout << "Rounded Start: " << starts[i][0] << ", " << starts[i][1] << ", " << starts[i][2] << std::endl;
        std::cout << "Rounded Goal: " << goals[i][0] << ", " << goals[i][1] << ", " << goals[i][2] << std::endl;

        // Check if start and goal are within map bounds
        if (starts[i][0] < 0 || starts[i][0] >= width || 
            starts[i][1] < 0 || starts[i][1] >= height || 
            starts[i][2] < 0 || starts[i][2] >= depth) {
            std::cout << RED << "Error: Start position (" << starts[i][0] << ", " << starts[i][1] << ", " << starts[i][2] 
                      << ") is out of map bounds (" << width << " x " << height << " x " << depth << ")" << RESET << std::endl;
            continue;
        }
        if (goals[i][0] < 0 || goals[i][0] >= width || 
            goals[i][1] < 0 || goals[i][1] >= height || 
            goals[i][2] < 0 || goals[i][2] >= depth) {
            std::cout << RED << "Error: Goal position (" << goals[i][0] << ", " << goals[i][1] << ", " << goals[i][2] 
                      << ") is out of map bounds (" << width << " x " << height << " x " << depth << ")" << RESET << std::endl;
            continue;
        }

        // print the value in the map
        std::cout << "Start value in map: " << map[(int)starts[i][2]][(int)starts[i][1]][(int)starts[i][0]] << std::endl;
        std::cout << "Goal value in map: " << map[(int)goals[i][2]][(int)goals[i][1]][(int)goals[i][0]] << std::endl;

        std::shared_ptr<ConstrainedActionSpace3dRob> ActionSpace = std::make_shared<ConstrainedActionSpace3dRob>(scene, action_type);
        ims::AStar planner(params);

        try {
            std::cout << "Initializing planner..." << std::endl;
            planner.initializePlanner(ActionSpace, starts[i], goals[i]);
        }
        catch (std::exception& e) {
            std::cout << RED << "Failed to initialize planner: " << e.what() << RESET << std::endl;
            continue;
        }

        std::cout << "Planning..." << std::endl;
        std::vector<StateType> path_;
        if (!planner.plan(path_)) {
            std::cout << RED << "No path found!" << RESET << std::endl;
        }
        else {
            std::cout << GREEN << "Path found!" << RESET << std::endl;
            if (cache) {
                boost::filesystem::path experiences_path = data_path / "experiences";
                boost::filesystem::create_directories(experiences_path);
                std::string path_file = (experiences_path / ("path_" + std::to_string(map_index + 1) + "_" + std::to_string(i) + ".csv")).string();
                std::ofstream file(path_file);
                file << "Experience," << path_.size() << ",3" << std::endl;
                for (const auto& state : path_) {
                    file << state[0] << "," << state[1] << "," << state[2] << std::endl;
                }
                file.close();
                std::cout << "Path saved to: " << path_file << std::endl;
            }
        }

        PlannerStats stats = planner.reportStats();
        std::cout << GREEN << "Planning time: " << stats.time << " sec" << std::endl;
        std::cout << "Cost: " << stats.cost << std::endl;
        std::cout << "Path length: " << path_.size() << std::endl;
        std::cout << "Number of nodes expanded: " << stats.num_expanded << std::endl;
        std::cout << "Number of nodes generated: " << stats.num_generated << std::endl;
        std::cout << "Suboptimality: " << stats.suboptimality << RESET << std::endl;

        logs[i] = stats;
        paths[i] = path_;
    }

    std::cout << "\nSaving logs and paths..." << std::endl;
    logStats(logs, map_index + 1, "Astar3D");
    std::string path_file = logPaths(paths, map_index + 1, scale);

    std::string plot_path = (project_root / "domains" / "3d_robot_nav" / "scripts" / "visualize_paths.py").string();
    if (!boost::filesystem::exists(plot_path)) {
        std::cerr << YELLOW << "Warning: Visualization script not found at " << plot_path << RESET << std::endl;
        std::cout << "You can visualize the results manually using the generated 'paths_tmp.csv' file." << std::endl;
    } else {
        std::string command = "python3 " + plot_path + " --filepath " + path_file + " --mapfile " + map_file;
        std::cout << "Running the plot script: " << command << std::endl;
        int result = system(command.c_str());
        if (result != 0) {
            std::cerr << RED << "Error running the plot script. Return code: " << result << RESET << std::endl;
        }
    }

    std::cout << GREEN << "3D A* pathfinding completed successfully." << RESET << std::endl;
    return 0;
}