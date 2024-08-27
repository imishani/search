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
 * \file   run_2d_mosaic.cpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   Jul 07 2024
*/


#include <boost/filesystem.hpp>
#include <vector>
#include <memory>
#include <iostream>
#include <string>
#include <cmath>

// project includes
#include <search/planners/astar.hpp>
#include <search/heuristics/standard_heuristics.hpp>

#include "search/planners/mosaic.hpp"
#include "action_space_2d_rob_mosaic.hpp"
#include "utils.hpp"
#include "controllers_2d.hpp"


///@brief loading the 2d grid map. It is a txt file of 0s and 1s. If the value is 1, it is an obstacle.
bool load2DGrid(int map_idx, std::vector<std::vector<int>>& map) {
    // load the map via the map index.
    std::string map_file = "./../domains/2d_robot_nav/data/gridworld/maps/200x200/map_" + std::to_string(map_idx) + ".map";
    // check if the file exists
    if (!boost::filesystem::exists(map_file)) {
        std::cout << "The map file does not exist!" << std::endl;
        return false;
    }
    FILE *f;
    f = fopen(map_file.c_str(), "r");
    if (f)
    {
        std::vector<int> row;
        int val;
        while (fscanf(f, "%d", &val) != EOF) {
            // check if end of line
            row.push_back(val*100);
            if (fgetc(f) == '\n') {
                map.push_back(row);
                row.clear();
            }
        }
        fclose(f);
    }
    return true;

}


bool sampleStartsGoals(const std::vector<std::vector<int>>& map, std::vector<std::vector<double>>& starts,
                       std::vector<std::vector<double>>& goals, int num_runs) {
    // sample the starts and goals
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, static_cast<int>(map.size()) - 1);
    for (int i {0}; i < num_runs; i++) {
        std::vector<double> start = {static_cast<double>(dis(gen)), static_cast<double>(dis(gen))};
        std::vector<double> goal = {static_cast<double>(dis(gen)), static_cast<double>(dis(gen))};
        while ((map.at(static_cast<int>(start[0])).at(static_cast<int>(start[1])) > 0) ||
            (map.at(static_cast<int>(goal[0])).at(static_cast<int>(goal[1])) > 0)) {

            start = {static_cast<double>(dis(gen)), static_cast<double>(dis(gen))};
            goal = {static_cast<double>(dis(gen)), static_cast<double>(dis(gen))};
        }
        starts.push_back(start);
        goals.push_back(goal);
    }
    return true;
}

int main(int argc, char** argv) {

    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <map_file_idx> <num_runs> <path>" << std::endl;
        return 0;
    }


    boost::filesystem::path full_path(boost::filesystem::current_path() );
    std::cout << "Current path is : " << full_path.string() << std::endl;

    int map_idx = std::stoi(argv[1]);
    int num_runs = std::stoi(argv[2]);

    std::vector<std::vector<int>> map;
    if (!load2DGrid(map_idx, map)) {
        std::cout << "Failed to load the map!" << std::endl;
        return 0;
    }

    std::vector<std::vector<double>> starts, goals;
    // sample the starts and goals
    sampleStartsGoals(map, starts, goals, num_runs);

    ims::MosaicParams params;
    params.time_limit_ = 50;
    params.verbose = false;

    ////////////////////////////////////////////////////
    // auto* heuristic = new ims::EuclideanHeuristic();
    // ims::AStarParams params (heuristic);

    ////////////////////////////////////////////////////
    // construct the scene and the action space
    Scene2DRob scene (map);
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
        std::cout << "Start value: " << map[(int)starts[i][0]][(int)starts[i][1]] << std::endl;
        std::cout << "Goal value: " << map[(int)goals[i][0]][(int)goals[i][1]] << std::endl;

        std::shared_ptr<ActionSpace2dRobMosaic> ActionSpace = std::make_shared<ActionSpace2dRobMosaic>(scene,
                                                                                                       action_type);
        ///////////////////////////////////////////////////////
        // std::shared_ptr<actionSpace2dRob> ActionSpace = std::make_shared<actionSpace2dRob>(scene, action_type);
        //
        // ims::AStar planner(params);
        ///////////////////////////////////////////////////////

        // construct planner
        ims::Mosaic planner(params);


        std::shared_ptr<std::vector<ims::Controller>> controllers = std::make_shared<std::vector<ims::Controller>>();
        ims::WallFollowerController controller;
        controller.init(map, ActionSpace);
        controller.solver_fn = ims::ControllerWallsFollower;

//        controller.type = ims::ControllerType::GENERATOR;
//        controller.solver_fn = Controller2d;
//        StateType user = {starts[i][0] + 3, starts[i][1]};
//        controller.user_data = &user;
//        controller.as_ptr = ActionSpace;

        controllers->push_back(controller);

//        ims::LinearController controller2 (ims::ControllerType::CONNECTOR);
//        controller2.init(1, ActionSpace, starts[i], goals[i]);
//        controllers->push_back(controller2);

        ims::wAStarController controller2 (ims::ControllerType::CONNECTOR);
        controller2.init(starts[i], goals[i], ActionSpace, std::make_shared<Scene2DRob>(scene));
        controller2.solver_fn = ims::wAStarControllerFn;
        controllers->push_back(controller2);

        // catch the exception if the start or goal is not valid
        try {
            planner.initializePlanner(ActionSpace, controllers, starts[i], goals[i]);
            // planner.initializePlanner(ActionSpace, starts[i], goals[i]);
        }
        catch (std::exception& e) {
            std::cout << e.what() << std::endl;
            std::cout << RED << "Start or goal is not valid! in planning query: " << i << RESET << std::endl;
            continue;
        }
        // plan
        std::cout << "Planning..." << std::endl;
        std::vector<StateType> path_;
        if (!planner.plan(path_)) {
            std::cout << RED << "No path found!" << RESET << std::endl;
            path_.clear();
            continue;
        }

        else
            std::cout << MAGENTA << "Path found!" << RESET << std::endl;
        PlannerStats stats = planner.reportStats();
        std::cout << GREEN << "Planning time: " << stats.time << " sec" << std::endl;
        std::cout << "cost: " << stats.cost << std::endl;
        std::cout << "Path length: " << path_.size() << std::endl;
        std::cout << "Number of nodes expanded: " << stats.num_expanded << std::endl;
        std::cout << "Number of nodes generated: " << stats.num_generated << std::endl;
        std::cout << "suboptimality: " << stats.suboptimality << RESET << std::endl;
        logs[i] = stats; // log the stats
        paths[i] = path_; // log the path
    }

    // save the logs to a temporary file
    // logStats(logs, map_index, "mosaic");
//
    std::string path_file = logPaths(paths, map_idx, 200);

    std::string plot_path = full_path.string() + "/../domains/2d_robot_nav/scripts/visualize_paths.py";
//    std::string command = "python3 " + plot_path + " --filepath " + path_file + " --path_ids 49";
    std::string command = "python3 " + plot_path + " --filepath " + path_file + " --map_type gridworld";
    std::cout << "Running the plot script..." << std::endl;
    auto boo = system(command.c_str());

    return 0;
}
