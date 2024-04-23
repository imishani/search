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
 * \file   run_2d_lpastar.cpp
 * \author Carina Sanborn (czsanbor@andrew.cmu.edu)
 * \date   2/14/24
*/


#include <boost/filesystem.hpp>
#include <vector>
#include <memory>
#include <iostream>
#include <string>
#include <cmath>
#include <random>
#include <algorithm>

// project includes
#include <search/planners/lpastar.hpp>
#include <search/heuristics/standard_heuristics.hpp>
#include "action_space_2d_rob.hpp"
#include "utils.hpp"

/// @brief 
/// @param partial_map 
/// @return 
std::string createPartialMapFile(std::vector<std::vector<int>> partial_map) {
    int width = partial_map.size();
    int height = partial_map[0].size();
    std::string map_type = "octile";
    std::string partial_map_file = logPartialMap(partial_map, height, width, map_type);
    return partial_map_file;
}

/// @brief 
/// @param map 
/// @return 
std::vector<std::vector<size_t>> convertAndShuffleIndices(const std::vector<std::vector<int>> map) {
    std::vector<std::vector<size_t>> map_indices;
    
    for (size_t y = 0; y < map.size(); ++y) {
        for (size_t x = 0; x < map[y].size(); ++x) {
            map_indices.push_back({x, y});
        }
    }

    std::random_device rd;
    std::mt19937 g(rd());
    shuffle(map_indices.begin(), map_indices.end(), g);
    return map_indices;
}

/// @brief Splits vector of map indices into five equal parts
/// @param indices Vector of shuffled map indices
/// @return Vector containing all the map indices equally split into five parts
std::vector<std::vector<std::vector<size_t>>> splitIndices(const std::vector<std::vector<size_t>>& indices) {
    std::vector<std::vector<std::vector<size_t>>> result;
    int partSize = indices.size() / 5;
    for (int i = 0; i < 5; ++i) {
        std::vector<std::vector<size_t>> part(indices.begin() + i * partSize, indices.begin() + (i + 1) * partSize);
        result.push_back(part);
    }
    return result;
}

/// @brief Rebuilds the map with only the obstacles specifed in a specified percentage of the original map
/// @param map Vector of the original map values
/// @param map_parts Vector a shuffled map indices split into five equal parts
/// @param index The max index of map parts we will use for the new map
/// @return The new partial map
std::vector<std::vector<int>> reconstructMap(std::vector<std::vector<int>> map, std::vector<std::vector<std::vector<size_t>>> map_parts, int index) {
    if (index >= map_parts.size() - 1) {
        // use entire original map
        return map;
    } else {
        // create empty map of 0s
        std::vector<std::vector<int>> result(map.size(), std::vector<int>(map[0].size(), 0));
        // add obstacles from original map
        for (int i = 0; i <= index; i++) {
            std::vector<std::vector<size_t>> included_indices = map_parts[i];
            for (const std::vector<size_t> pos : included_indices) {
                result[pos[1]][pos[0]] = map[pos[1]][pos[0]];
            }
        }
        return result;
    }
}

int main(int argc, char** argv) {

    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <map_file> <num_runs> <scale> <path>" << std::endl;
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

    std::vector<std::string> starts_goals_path = {full_path.string() + "/../domains/2d_robot_nav/data/hrt201n/",
                                                  full_path.string() + "/../domains/2d_robot_nav/data/den501d/",
                                                  full_path.string() + "/../domains/2d_robot_nav/data/den520d/",
                                                  full_path.string() + "/../domains/2d_robot_nav/data/ht_chantry/",
                                                  full_path.string() + "/../domains/2d_robot_nav/data/brc203d/",
    };

    int map_index = std::stoi(argv[1]);
    int num_runs = std::stoi(argv[2]);
    int scale = std::stoi(argv[3]);
    std::string path = starts_goals_path[map_index];

    std::string map_file = maps[map_index];

    std::string type;
    int width, height;
    std::vector<std::vector<int>> map = loadMap(map_file.c_str(), type, width, height, scale);
    std::vector<std::vector<size_t>> shuffled_map_indices = convertAndShuffleIndices(map);

    std::vector<std::vector<std::vector<size_t>>> map_parts = splitIndices(shuffled_map_indices);

    std::vector<std::vector<int>> curr_map = reconstructMap(map, map_parts, 0);

    std::vector<std::vector<double>> starts, goals;
    loadStartsGoalsFromFile(starts, goals, scale, num_runs, path);

    // construct the planner
    std::cout << "Constructing planner..." << std::endl;
    // construct planner params
    ims::EuclideanHeuristic* heuristic = new ims::EuclideanHeuristic();
    ims::LPAStarParams params (heuristic);
    // construct the scene and the action space
    Scene2DRob scene (curr_map);
    ActionType2dRob action_type;

    // log the results
    std::unordered_map<int, PlannerStats> logs;
    std::unordered_map<int, PathType> paths;
    for (int i {0}; i < starts.size(); i++){
        // round the start and goal to the nearest integer
        std::cout << "Start: " << starts[i][0] << ", " << starts[i][1] << std::endl;
        std::cout << "Goal: " << goals[i][0] << ", " << goals[i][1] << std::endl;
        for (int j {0}; j < 2; j++){
            starts[i][j] = std::round(starts[i][j]);
            goals[i][j] = std::round(goals[i][j]);
        }
        std::cout << "Rounded Start: " << starts[i][0] << ", " << starts[i][1] << std::endl;
        std::cout << "Rounded Goal: " << goals[i][0] << ", " << goals[i][1] << std::endl;

        // print the value in the map
        std::cout << "Start value: " << curr_map[(int)starts[i][0]][(int)starts[i][1]] << std::endl;
        std::cout << "Goal value: " << curr_map[(int)goals[i][0]][(int)goals[i][1]] << std::endl;

        std::shared_ptr<actionSpace2dRob> action_space = std::make_shared<actionSpace2dRob>(scene, action_type);
        // construct planner
        ims::LPAStar planner(params);
        // catch the exception if the start or goal is not valid
        try {
            planner.initializePlanner(action_space, starts[i], goals[i]);
        }
        catch (std::exception& e) {
            std::cout << RED << "Start or goal is not valid!" <<RESET << std::endl;
            continue;
        }
        std::vector<StateType> path_;
        for (int k {0}; k < 5; k++) {
            // plan
            std::cout << "Planning..." << std::endl;
            path_.clear();
            if (!planner.plan(path_)) {
                std::cout << RED << "No path found!" << RESET << std::endl;
            }
            else {
                std::cout << GREEN << "Path found!" << RESET << std::endl;

                // save the logs to a temporary file
                logStats(logs, map_index, "LPAstar");

                std::string path_file = logPaths(paths, map_index, scale);
                std::string partial_map_file = createPartialMapFile(curr_map);
                
                std::string image_name = "/run_2d_lpastar_map" + std::to_string(k) + ".png";
                std::string image_path = full_path.string() + image_name;

                std::string plot_path = full_path.string() + "/../domains/2d_robot_nav/scripts/visualize_paths_partial_map.py";
                std::string command = "python3 " + plot_path + " --filepath " + path_file + " --mapfile " + partial_map_file + " --imagepath " + image_path;
                std::cout << "Running the plot script..." << std::endl;

                system(command.c_str());

                std::cout << "Completed running plot script. Plot saved to " + full_path.string() + image_name << std::endl;
            }
            if (k != 4) {
                // update map
                curr_map = reconstructMap(map, map_parts, k+1);
                std::vector<std::vector<size_t>> updated_indices = map_parts[k+1];
                Scene2DRob scene (curr_map);
                action_space->updateActionSpace(scene);
                planner.updateVertices(updated_indices, curr_map);
            }
        }
        PlannerStats stats = planner.reportStats();
        std::cout << GREEN << "Planning time: " << stats.time << " sec" << std::endl;
        std::cout << "cost: " << stats.cost << std::endl;
        std::cout << "Path length: " << path_.size() << std::endl;
        std::cout << "Number of nodes expanded: " << stats.num_expanded << std::endl;
        std::cout << "Number of nodes generated: " << stats.num_generated << std::endl;
        std::cout << "suboptimality: " << stats.suboptimality << RESET << std::endl;
        logs[i] = stats; // log the stats
        paths[i] = path_; // log the path
        
        // reset map
        curr_map = reconstructMap(map, map_parts, 0);
    }

    // curr_map = reconstructMap(map, map_parts, 4);
    // // save the logs to a temporary file
    // logStats(logs, map_index, "LPAstar");

    // std::string path_file = logPaths(paths, map_index, scale);
    // std::string partial_map_file = createPartialMapFile(curr_map);
    
    // std::string image_name = "/run_2d_lpastar_map.png";
    // std::string image_path = full_path.string() + image_name;

    // std::string plot_path = full_path.string() + "/../domains/2d_robot_nav/scripts/visualize_paths_partial_map.py";
    // std::string command = "python3 " + plot_path + " --filepath " + path_file + " --mapfile " + partial_map_file + " --imagepath " + image_path;
    // std::cout << "Running the plot script..." << std::endl;

    // system(command.c_str());

    // std::cout << "Completed running plot script. Plot saved to " + full_path.string() + image_name << std::endl;

    return 0;
}
