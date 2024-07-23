/*
 * Copyright (C) 2023, Itamar Mishani
 * All rights reserved.
 */
/*!
 * \file   run_3d_astar.cpp
 * \author Julius Arolovitch (juliusa@cmu.edu)
 * \date   07.22.2024
*/

#include <boost/filesystem.hpp>
#include <vector>
#include <memory>
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>

#include <search/planners/astar.hpp>
#include <search/heuristics/standard_heuristics.hpp>

#include "action_space_3d_rob.hpp"
#include "utils.hpp"

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <map_file> <num_runs> <scale> <path>" << std::endl;
        return 0;
    }
    std::vector<std::string> maps;

    boost::filesystem::path full_path(boost::filesystem::current_path());
    std::cout << "Current path is : " << full_path.string() << std::endl;
    
    maps.emplace_back(full_path.string() + "/../domains/3d_robot_nav/data/map1.map");
    maps.emplace_back(full_path.string() + "/../domains/3d_robot_nav/data/map2.map");

    std::vector<std::string> starts_goals_path = {
        full_path.string() + "/../domains/3d_robot_nav/data/map1/",
        full_path.string() + "/../domains/3d_robot_nav/data/map2/",
    };

    int map_index = std::stoi(argv[1]);
    int num_runs = std::stoi(argv[2]);
    int scale = std::stoi(argv[3]);
    bool cache = false;
    try {
        cache = std::stoi(argv[4]);
    }
    catch (std::exception& e) {
        std::cout << YELLOW << "Didn't specify whether to save the path or not. Default is not to save." << RESET << std::endl;
    }

    std::string path = starts_goals_path[map_index];
    std::string map_file = maps[map_index];

    std::string type;
    int width, height, depth;
    std::vector<std::vector<std::vector<int>>> map = load3DMap(map_file.c_str(), type, width, height, depth, scale);

    std::vector<std::vector<double>> starts, goals;
    load3DStartsGoalsFromFile(starts, goals, scale, num_runs, path);

    std::cout << "Constructing planner..." << std::endl;
    auto* heuristic = new ims::EuclideanHeuristic();
    ims::AStarParams params(heuristic);
    Scene3DRob scene(map);
    ActionType3dRob action_type;

    std::unordered_map<int, PlannerStats> logs;
    std::unordered_map<int, PathType> paths;
    for (int i = 0; i < starts.size(); i++) {
        std::cout << "Start: " << starts[i][0] << ", " << starts[i][1] << ", " << starts[i][2] << std::endl;
        std::cout << "Goal: " << goals[i][0] << ", " << goals[i][1] << ", " << goals[i][2] << std::endl;
        for (int j = 0; j < 3; j++) {
            starts[i][j] = std::round(starts[i][j]);
            goals[i][j] = std::round(goals[i][j]);
        }
        std::cout << "Rounded Start: " << starts[i][0] << ", " << starts[i][1] << ", " << starts[i][2] << std::endl;
        std::cout << "Rounded Goal: " << goals[i][0] << ", " << goals[i][1] << ", " << goals[i][2] << std::endl;

        std::cout << "Start value: " << map[(int)starts[i][0]][(int)starts[i][1]][(int)starts[i][2]] << std::endl;
        std::cout << "Goal value: " << map[(int)goals[i][0]][(int)goals[i][1]][(int)goals[i][2]] << std::endl;

        std::shared_ptr<ConstrainedActionSpace3dRob> ActionSpace = std::make_shared<ConstrainedActionSpace3dRob>(std::make_shared<Scene3DRob>(scene), action_type);
        ims::AStar planner(params);
        try {
            planner.initializePlanner(ActionSpace, starts[i], goals[i]);
        }
        catch (std::exception& e) {
            std::cout << RED << "Start or goal is not valid!" << RESET << std::endl;
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
                std::string path_file = path + "experiences/" + "path_" + std::to_string(i) + ".csv";
                std::ofstream file(path_file);
                file << "Experience," << path_.size() << "," << 3 << std::endl;
                for (int j = 0; j < path_.size(); j++) {
                    file << path_[j][0] << "," << path_[j][1] << "," << path_[j][2] << std::endl;
                }
                file.close();
            }
        }

        PlannerStats stats = planner.reportStats();
        std::cout << GREEN << "Planning time: " << stats.time << " sec" << std::endl;
        std::cout << "cost: " << stats.cost << std::endl;
        std::cout << "Path length: " << path_.size() << std::endl;
        std::cout << "Number of nodes expanded: " << stats.num_expanded << std::endl;
        std::cout << "Number of nodes generated: " << stats.num_generated << std::endl;
        std::cout << "suboptimality: " << stats.suboptimality << RESET << std::endl;
        logs[i] = stats;
        paths[i] = path_;
    }

    logStats(logs, map_index, "Astar3D");

    std::string path_file = log3DPaths(paths, map_index, scale);

    std::string plot_path = full_path.string() + "/../domains/3d_robot_nav/scripts/visualize_3d_paths.py";
    std::string command = "python3 " + plot_path + " --filepath " + path_file;
    std::cout << "Running the plot script..." << std::endl;

    system(command.c_str());

    return 0;
}