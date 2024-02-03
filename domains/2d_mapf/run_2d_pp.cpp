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
 * \file   run_2d_pp.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   Sept 15 2023
*/


#include <boost/filesystem.hpp>
#include <vector>
#include <memory>
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>

// project includes
#include <search/planners/multi_agent/prioritized_planning.hpp>
#include <search/heuristics/standard_heuristics.hpp>

// Note(yoraish): Leaving quotation marks include since it is a local file for the example.
#include "action_space_2d_rob.hpp"
#include "../2d_robot_nav/utils.hpp"


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
    maps.emplace_back("../domains/2d_mapf/data/hallway_3/hallway_3.map");

    std::vector<std::string> starts_goals_path = {"../domains/2d_robot_nav/data/hrt201n/",
                                        "../domains/2d_robot_nav/data/den501d/",
                                        "../domains/2d_robot_nav/data/den520d/",
                                        "../domains/2d_robot_nav/data/ht_chantry/",
                                        "../domains/2d_robot_nav/data/brc203d/",
                                        "../domains/2d_mapf/data/corridor10/",
                                        "../domains/2d_mapf/data/corridor20/",
                                        "../domains/2d_mapf/data/clutter32/",
                                        "../domains/2d_mapf/data/hallway_6/",
                                        "../domains/2d_mapf/data/hallway_3/"

    };

    int map_index = std::stoi(argv[1]);
    int num_agents = std::stoi(argv[2]);
    int scale = std::stoi(argv[3]);
    std::string path = starts_goals_path[map_index];
    std::string map_file = maps[map_index];

    std::string type;
    int width, height;
    std::cout << "Loading map from path " << map_file << std::endl;
    std::vector<std::vector<int>> map = loadMap(map_file.c_str(), type, width, height, scale);

    std::vector<std::vector<double>> starts, goals;
    loadStartsGoalsFromFile(starts, goals, scale, num_agents, path);

    // Construct the planner.
    std::cout << "Constructing planner..." << std::endl;

    // Construct the parameters.
    ims::PrioritizedPlanningParams params;
    params.low_level_heuristic_ptrs;
    for (int i {0}; i < num_agents; i++){
        params.low_level_heuristic_ptrs.emplace_back(new ims::EuclideanRemoveTimeHeuristic);
    }

    // Construct the scene and the action space.
    std::shared_ptr<MAPFScene2DRob> scene = std::make_shared<MAPFScene2DRob>();
    scene->loadMap(map_file);
    ActionType2dTimeRob action_type;

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
    ims::PrioritizedPlanning planner(params);
    planner.initializePlanner(action_spaces, start_state_vals, goal_state_vals);

    // Plan.
    MultiAgentPaths paths;
    planner.plan(paths);
    
    if (paths.empty()){
        std::cout << "No solution found." << std::endl;
        return 0;
    }

    // Print the paths.
    std::cout << "Paths: " << std::endl;
    for (int i {0}; i < num_agents; i++){
        std::cout << "Agent " << i << ": ";
        for (int j {0}; j < paths[i]->size(); j++){
            std::cout << "[" << paths[i]->at(j)[0] << ", " << paths[i]->at(j)[1] << ", " << paths[i]->at(j)[2] << "]";
            if (j < paths[i]->size() - 1){
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
        for (int j {0}; j < paths[i]->size(); j++){
            fout << "           [" << paths[i]->at(j)[0] << ", " << paths[i]->at(j)[1] << ", " << paths[i]->at(j)[2] << "]";
            if (j < paths[i]->size() - 1){
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

    // Execute the visualization script.
    std::string command = "python3 ../src/scripts/visualize_2d_time_paths.py --paths-yaml paths.yaml --fps 1 && open paths.gif";
    std::cout << "Running the plot script..." << std::endl;
    system(command.c_str());

    return 0;
}


