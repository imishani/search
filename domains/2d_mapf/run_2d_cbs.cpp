/*
 * Copyright (C) 2023, Yorai Shaoul
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
 * \file   planner.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   09/07/2023
*/


#include <boost/filesystem.hpp>
#include <vector>
#include <memory>
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
// Get the cv circle.
#include <opencv2/imgproc.hpp>

// project includes
#include <search/planners/multi_agent/cbs.hpp>
#include <search/heuristics/standard_heuristics.hpp>

// Note(yoraish): Leaving quotation marks include since it is a local file for the example.
#include "action_space_2d_rob.hpp"

double to_degrees(double rads)
{
    return rads * 180.0 / M_PI;
}

double roundOff(double value, unsigned char prec)
{
    double pow_10 = pow(10.0, (double)prec);
    return round(value * pow_10) / pow_10;
}


std::vector<std::vector<int>> loadMap(const char *fname, cv::Mat& img,
                                      std::string& type, int& width,
                                      int& height, int scale=1) {

    std::vector<std::vector<int>> map;
    FILE *f;
    f = fopen(fname, "r");

    if (f)
    {   
        if (fscanf(f, "type octile\nheight %d\nwidth %d\nmap\n", &height, &width))
        {
            // Print the width height.
            map.resize(width, std::vector<int>(height));

            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    char c;
                    do {
                        fscanf(f, "%c", &c);
                    } while (isspace(c));

                    map[x][y] = (c == '.' || c == 'G' || c == 'S' || c == 'T') ? 0 : 100;
                }
            }
        }

        fclose(f);
    }

    std::vector<std::vector<int>> scaled_map;
    int scaled_height = scale*height;
    int scaled_width = scale*width;
    scaled_map.resize(scaled_width, std::vector<int>(scaled_height));

    for (int y = 0; y < scaled_height; y++)
    {
        for (int x = 0; x < scaled_width; x++)
        {
            scaled_map[x][y] = map[x/scale][y/scale];
        }
    }

    img = cv::Mat(scaled_height, scaled_width, CV_8UC3);

    for (int y = 0; y < scaled_height; y++)
    {
        for (int x = 0; x < scaled_width; x++)
        {
            img.at<cv::Vec3b>(y,x) = (scaled_map[x][y] > 0) ? cv::Vec3b(0,0,0) : cv::Vec3b(255,255,255);
        }
    }
    return scaled_map;
}

void loadStartsGoalsFromFile(std::vector<std::vector<double>>& starts, std::vector<std::vector<double>>& goals, int scale, int num_agents, const std::string& path)
{
    std::ifstream starts_fin(path + "nav2d_starts.txt");
    std::ifstream goals_fin(path + "nav2d_goals.txt");

    for (int j = 0; j < num_agents; ++j)
    {
        std::vector<double> start, goal;
        double val_start, val_goal;
        for (int i = 0; i < 2; ++i)
        {
            starts_fin >> val_start;
            goals_fin >> val_goal;
            start.push_back(scale*val_start);
            goal.push_back(scale*val_goal);
        }
        std::cout << "Read start and goal: (" << start[0] << ", " << start[1] << ") -> (" << goal[0] << ", " << goal[1] << ")" << std::endl;
        start[2] = to_degrees(start[2]);
        goal[2] = to_degrees(goal[2]);
        starts.emplace_back(start);
        goals.emplace_back(goal);

        double cost, length;
        starts_fin >> cost;
        starts_fin >> length;
    }
}


int main(int argc, char** argv) {

    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <map_file> <num_agents> <scale> <path>" << std::endl;
        return 0;
    }
    std::vector<std::string> maps;

    boost::filesystem::path full_path( boost::filesystem::current_path() );
    std::cout << "Current path is : " << full_path.string() << std::endl;
    // At each emplace_back, use the full pathh and concatenate the map name
    maps.emplace_back("../domains/2d_robot_nav/data/hrt201n/hrt201n.map");
    maps.emplace_back("../domains/2d_robot_nav/data/den501d/den501d.map");
    maps.emplace_back("../domains/2d_robot_nav/data/den520d/den520d.map");
    maps.emplace_back("../domains/2d_robot_nav/data/ht_chantry/ht_chantry.map");
    maps.emplace_back("../domains/2d_robot_nav/data/brc203d/brc203d.map");
    maps.emplace_back("../domains/2d_mapf/data/corridor10/corridor10.map");
    maps.emplace_back("../domains/2d_mapf/data/corridor20/corridor20.map");
    maps.emplace_back("../domains/2d_mapf/data/clutter32/clutter32.map");
    maps.emplace_back("../domains/2d_mapf/data/hallway_6/hallway_6.map");

    std::vector<std::string> starts_goals_path = {"../domains/2d_robot_nav/data/hrt201n/",
                                        "../domains/2d_robot_nav/data/den501d/",
                                        "../domains/2d_robot_nav/data/den520d/",
                                        "../domains/2d_robot_nav/data/ht_chantry/",
                                        "../domains/2d_robot_nav/data/brc203d/",
                                        "../domains/2d_mapf/data/corridor10/",
                                        "../domains/2d_mapf/data/corridor20/",
                                        "../domains/2d_mapf/data/clutter32/",
                                        "../domains/2d_mapf/data/hallway_6/"
    };

    int map_index = std::stoi(argv[1]);
    int num_agents = std::stoi(argv[2]);
    int scale = std::stoi(argv[3]);
    std::string path = starts_goals_path[map_index];
    std::string map_file = maps[map_index];

    std::string type;
    int width, height;
    cv::Mat img;
    std::cout << "Loading map from path " << map_file << std::endl;
    std::vector<std::vector<int>> map = loadMap(map_file.c_str(), img, type, width, height, scale);

    std::vector<std::vector<double>> starts, goals;
    loadStartsGoalsFromFile(starts, goals, scale, num_agents, path);

    // Construct the planner.
    std::cout << "Constructing planner..." << std::endl;

    // Construct the parameters.
    ims::CBSParams params;

    // Construct the scene and the action space.
    scene2DRob scene (map);
    actionType2dRob action_type;

    // Construct the parameters.
    std::vector<StateType> start_state_vals;
    std::vector<StateType> goal_state_vals;

    for (int i {0}; i < num_agents; i++){
        // Round the start and goal to the nearest integer.
        std::cout << "Start: " << starts[i][0] << ", " << starts[i][1] << std::endl;
        std::cout << "Goal: " << goals[i][0] << ", " << goals[i][1] << std::endl;
        for (int j {0}; j < 2; j++){
            starts[i][j] = std::round(starts[i][j]);
            goals[i][j] = std::round(goals[i][j]);
        }

        // Add the time to the start and goal.
        std::cout << "Start: " << starts[i][0] << ", " << starts[i][1] << ", " << starts[i][2] << std::endl;
        starts[i].emplace_back(0);
        goals[i].emplace_back(-1);
        
        start_state_vals.emplace_back(starts[i]);
        goal_state_vals.emplace_back(goals[i]);

        std::cout << "Rounded Start: " << starts[i][0] << ", " << starts[i][1] << std::endl;
        std::cout << "Rounded Goal: " << goals[i][0] << ", " << goals[i][1] << std::endl;

        // Print the value in the map.
        std::cout << "Start value: " << map[(int)starts[i][0]][(int)starts[i][1]] << std::endl;
        std::cout << "Goal value: " << map[(int)goals[i][0]][(int)goals[i][1]] << std::endl;
    }

    // Now we have all the star and end configurations of the agents stored in start_state_vals and goal_state_vals.
    // Let's create an action space for each agent.
    std::cout << "Creating action spaces..." << std::endl;
    std::vector<std::shared_ptr<ims::ConstrainedActionSpace>> action_spaces;
    for (int i {0}; i < num_agents; i++){
        action_spaces.emplace_back(std::make_shared<ConstrainedActionSpace2dRob>(scene, action_type));
        std::cout << "Action space " << i << " created." << std::endl;
    }

    // Now, use the action spaces to create the planner. The planner itself creates derived classes for each of the action spaces, called ConstrainedActionSpace.
    ims::CBS planner(params);
    planner.initializePlanner(action_spaces, start_state_vals, goal_state_vals);

    // Plan.
    ims::MultiAgentPaths paths;
    planner.plan(paths);
    
    if (paths.empty()){
        std::cout << "No solution found." << std::endl;
        return 0;
    }

    // Print the paths.
    std::cout << "Paths: " << std::endl;
    for (int i {0}; i < num_agents; i++){
        std::cout << "Agent " << i << ": ";
        for (int j {0}; j < paths[i].size(); j++){
            std::cout << "[" << paths[i][j][0] << ", " << paths[i][j][1] << ", " << paths[i][j][2] << "]";
            if (j < paths[i].size() - 1){
                std::cout << ", ";
            }
        }
        std::cout << std::endl;
    }

    PlannerStats stats = planner.reportStats();
    std::cout << GREEN << "Planning time: " << stats.time << " sec" << std::endl;
    std::cout << "cost: " << stats.cost << std::endl;
    std::cout << "Number of nodes expanded: " << stats.num_expanded << std::endl;
    std::cout << "Number of nodes generated: " << stats.num_generated << std::endl;
    std::cout << "suboptimality: " << stats.suboptimality << RESET << std::endl;

    // Save the paths to a paths.yaml file.
    std::ofstream fout("paths.yaml");
    // Merge the full_path and the map path, adjusting for '../' in the map path.
    std::string map_global_path = (boost::filesystem::weakly_canonical(boost::filesystem::current_path() / boost::filesystem::path(map_file))).string();
     
    fout << "map_path: \"" << map_global_path << "\"" << std::endl;
    fout << "paths: [ " << std::endl;
    for (int i {0}; i < num_agents; i++){
        fout << "       [ \n";
        for (int j {0}; j < paths[i].size(); j++){
            fout << "           [" << paths[i][j][0] << ", " << paths[i][j][1] << ", " << paths[i][j][2] << "]";
            if (j < paths[i].size() - 1){
                fout << ",\n";
            }
            else if (i < num_agents - 1){
                fout << "],\n";
            }
            else {
                fout << "]\n";
            }
        }
    }
    fout << "]" << std::endl;
    fout.close();

    return 0;
}

