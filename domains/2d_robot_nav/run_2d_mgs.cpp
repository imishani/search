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
 * \file   run_2d_mgs.cpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   01/02/25
*/


#include <boost/filesystem.hpp>
#include <vector>
#include <memory>
#include <iostream>
#include <string>
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp> // Get the cv circle.

// project includes
#include "search/planners/mgs.hpp"
#include "search/heuristics/standard_heuristics.hpp"
#include "action_space_2d_rob_mgs.hpp"
#include "utils.hpp"


int main(int argc, char** argv) {

    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <map_file> <num_runs> <scale> <path>" << std::endl;
        return 0;
    }
    std::vector<std::string> maps;

    boost::filesystem::path full_path(boost::filesystem::current_path() );
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
    cv::Mat img;
    std::vector<std::vector<int>> map = loadMap(map_file.c_str(), img, type, width, height, scale);

    std::vector<std::vector<double>> starts, goals;
    loadStartsGoalsFromFile(starts, goals, scale, num_runs, path);

    // construct the planner
    std::cout << "Constructing planner..." << std::endl;
    // construct planner params
    auto* heuristic = new ims::EuclideanHeuristic();
    int graphs_number = 3;
    ims::MGSParams params (heuristic, graphs_number);
    params.time_limit_ = 5;
    // construct the scene and the action space
    Scene2DRob scene (map);
    ActionType2dRob action_type;

    // log the results
    std::unordered_map<int, ims::MGSPlannerStats> logs;
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

        std::shared_ptr<ActionSpace2dRobMGS> ActionSpace = std::make_shared<ActionSpace2dRobMGS>(scene,
                                                                                                 action_type);
        // construct planner
        ims::MGS planner(params);
        // catch the exception if the start or goal is not valid
        try {
            planner.initializePlanner(ActionSpace, starts[i], goals[i], params.g_num_);
        }
        catch (std::exception& e) {
            std::cout << e.what() << std::endl;
            std::cout << "Start or goal is not valid!" << std::endl;
            continue;
        }
        // plan
        std::cout << "Planning..." << std::endl;
        std::vector<StateType> path_;
        if (!planner.plan(path_)) {
            std::cout << RED << "No path found!" << RESET << std::endl;
            path_.clear();
            continue;
//            return 0;
        }
        else
            std::cout << "Path found!" << std::endl;
        ims::MGSPlannerStats stats = planner.reportStats();
        logs[i] = stats; paths[i] = path_;
        std::cout << GREEN << "Planning time: " << stats.time << " sec" << std::endl;
        std::cout << "cost: " << stats.cost << std::endl;
        std::cout << "Path length: " << path_.size() << std::endl;
        std::cout << "Number of nodes expanded: " << stats.num_expanded << std::endl;
        std::cout << "Number of nodes generated: " << stats.num_generated << std::endl;
        std::cout << "suboptimality: " << stats.suboptimality << RESET << std::endl;

        // draw the start in red and goal in green
        img.at<cv::Vec3b>((int)starts[i][1], (int)starts[i][0]) = cv::Vec3b(0,0,255);
        img.at<cv::Vec3b>((int)goals[i][1], (int)goals[i][0]) = cv::Vec3b(0,255,0);
        cv::circle(img, cv::Point((int)starts[i][0], (int)starts[i][1]), 2, cv::Scalar(0,0,255), 1);
        cv::circle(img, cv::Point((int)goals[i][0], (int)goals[i][1]), 2, cv::Scalar(0,255,0), 1);

        // draw the path in blue but skip the start and goal
        for (int j {1}; j < path_.size()-1; j++){
            img.at<cv::Vec3b>((int)path_[j][1], (int)path_[j][0]) = cv::Vec3b(255,0,0);
        }
    }

    // save the logs to a file in current directory
    std::string log_file = "logs_mgs_map" + std::to_string(map_index) + "_" + "mgs_" +
        std::to_string(params.g_num_) +".csv";
    // save logs object to a file
    std::ofstream file(log_file);
    // header line
    file << "Problem,Time,Cost,Num_expanded,Num_generated" << std::endl;
    for (auto& log : logs) {
        file << log.first << "," << log.second.time << "," << log.second.cost << "," <<
        log.second.num_expanded << "," << log.second.num_generated << std::endl;
    }
    file.close();

    // save the paths to a temporary file
    std::string path_file = "paths_tmp.csv";
    // save paths object to a file
    std::ofstream file2(path_file);
    // header line
    file2 << "Problem,Scale,PathsNumber," << map_index << "," << scale << "," << paths.size() << std::endl;
    for (auto& path2 : paths) {
        file2 << path2.first << "," << path2.second.size() << std::endl;
        for (auto& state : path2.second) {
            file2 << state[0] << "," << state[1] << std::endl;
        }
    }
    file2.close();

    std::string image_name = "/run_2d_mgs_map.jpg";

    bool check_img_write = cv::imwrite(full_path.string() + image_name, img);

    if (!check_img_write) {
        std::cout << "Mission - Saving the image, FAILED" << std::endl;
    } else {
        std::cout << "Successfully saved the image to " + full_path.string() + image_name << std::endl;
    }

//    cv::namedWindow("Map", cv::WINDOW_NORMAL);
//    cv::imshow("Map", img);
//    cv::waitKey(0);

    std::string plot_path = full_path.string() + "/../domains/2d_robot_nav/scripts/visualize_paths.py";
    std::string command = "python3 " + plot_path + " " + path_file;
    std::cout << "Running the plot script..." << std::endl;
    system(command.c_str());

    return 0;
}
