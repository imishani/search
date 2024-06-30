/*
 * Copyright (C) 2024, Yorai Shaoul
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
 * \file   run_2d_ecbs_sipp.cpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   2024-06-12
*/

#include <boost/program_options.hpp> // For argument parsing.
#include <boost/filesystem.hpp>
#include <vector>
#include <memory>
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>

// project includes
#include <search/planners/multi_agent/cbs.hpp>
#include <search/planners/multi_agent/ecbs.hpp>
#include <search/planners/multi_agent/cbs_sipp.hpp>
#include <search/heuristics/standard_heuristics.hpp>

// Note(yoraish): Leaving quotation marks include since it is a local file for the example.
#include "action_space_2d_rob.hpp"
#include "../2d_robot_nav/utils.hpp"
#include "instance.h"


/// Example runs:
/// For custom datasets: ./run_2d_mapf_cbs_or_ecbs -m 6 -a ignore -n 4 -h 1.1
/// For mapf datasets: ./run_2d_mapf_cbs_or_ecbs --map_file_path=domains/2d_mapf/datasets/mapf-map/den312d.map
///                         -a domains/2d_mapf/datasets/scen-random/den312d-random-1.scen -n 10
///                         --high_level_focal_suboptimality=1.5
int main(int argc, char** argv) {
    namespace po = boost::program_options;
    // Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")

		// params for the input instance and experiment settings
        ("workspace_path,-d", po::value<string>()->default_value("/../"), "relative path to search directory from executable")
		("map_file_path,m", po::value<string>()->required(), "relative map file path from search directory")
        ("agent_file_path,a", po::value<string>()->required(), "relative agent file path from search directory")
		("num_agents,n", po::value<int>()->required(), "number of agents")
        ("high_level_focal_suboptimality,-h", po::value<double>()->default_value(1.0),
                    "high level suboptimality, default value of 1 is identical to CBS")
        ;

    po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help")) {
		std::cout << desc << std::endl;
		return 1;
	}

    po::notify(vm);
	if (vm["high_level_focal_suboptimality"].as<double>() < 1) {
		throw std::invalid_argument("high_level_focal_suboptimality must be >= 1");
	}

    boost::filesystem::path full_path( boost::filesystem::current_path() );
    std::cout << "Current path is : " << full_path.string() << std::endl;

    int num_agents = vm["num_agents"].as<int>();
    const string PATH_TO_WORKSPACE = full_path.string() + vm["workspace_path"].as<string>();

    // Load the instance, handles parsing the map and agent files.
    MAPFInstance instance;
    instance.loadInstanceFromArguments(PATH_TO_WORKSPACE, vm["map_file_path"].as<string>(),
                                            vm["agent_file_path"].as<string>(), num_agents);
    std::vector<StateType> start_state_vals = instance.getStartsWithTime();
    std::vector<StateType> goal_state_vals = instance.getGoalsWithTime();

    // Construct the planner.
    std::cout << "Constructing planner..." << std::endl;
    bool useECBS = (vm["high_level_focal_suboptimality"].as<double>() > 1.0);

    ims::MultiAgentPaths paths;
    PlannerStats stats;

    if (!useECBS) {
        // Now we have all the start and end configurations of the agents stored in start_state_vals and goal_state_vals.
        // Let's create an action space for each agent.
        std::cout << "Creating action spaces..." << std::endl;
        // NOTE(yoraish): For SIPP the action type is NOT TIMED.
        ActionType2dRob action_type;
        // Print the available actions.
        std::cout << "Available actions should be untimed:: " << std::endl;
        for (const Action & a : action_type.getPrimActions()){
            std::cout << " *   " << a << std::endl;
        }

        std::vector<std::shared_ptr<ims::ConstrainedActionSpace>> action_spaces;
        for (int i {0}; i < num_agents; i++){
            action_spaces.emplace_back(std::make_shared<ConstrainedActionSpace2dRob>(instance.getSceneInterface2D(), action_type));
            std::cout << "Action space " << i << " created." << std::endl;
        }

        // Construct the parameters.
        ims::CBSParams params;
        params.weight_low_level_heuristic = 1.0;
        params.verbose = false;

        for (int i {0}; i < num_agents; i++){
            // Note that the heuristic is not timed.
            params.low_level_heuristic_ptrs.emplace_back(new ims::EuclideanHeuristic);
        }
        std::cout << "Creating the planner..." << std::endl;
        ims::CBSSIPP planner(params);
        std::cout << "Initializing the planner..." << std::endl;
        planner.initializePlanner(action_spaces, start_state_vals, goal_state_vals);
        std::cout << "Planning..." << std::endl;
        // Plan.
        planner.plan(paths);
        stats = planner.reportStats();

    }
    else {
        // Now we have all the start and end configurations of the agents stored in start_state_vals and goal_state_vals.
        // Let's create an action space for each agent.
        std::cout << "Creating action spaces..." << std::endl;
        // NOTE(yoraish): For SIPP the action type is NOT TIMED.
        ActionType2dRob action_type;
        // Print the available actions.
        std::cout << "Available actions should be untimed:: " << std::endl;
        for (const Action & a : action_type.getPrimActions()){
            std::cout << " *   " << a << std::endl;
        }

        std::vector<std::shared_ptr<ims::SubcostConstrainedActionSpace>> action_spaces;
        for (int i {0}; i < num_agents; i++){
            action_spaces.emplace_back(std::make_shared<ConstrainedActionSpace2dRob>(instance.getSceneInterface2D(), action_type));
            std::cout << "Action space " << i << " created." << std::endl;
        }

        // Construct the parameters.
        ims::ECBSParams params;
        params.high_level_focal_suboptimality = vm["high_level_focal_suboptimality"].as<double>();
        params.low_level_focal_suboptimality = params.high_level_focal_suboptimality;
        params.weight_low_level_heuristic = 1.0;
        params.verbose = false;

        for (int i {0}; i < num_agents; i++){
            // Note that the heuristic is not timed.
            params.low_level_heuristic_ptrs.emplace_back(new ims::EuclideanHeuristic);
        }
        std::cout << "Creating the planner..." << std::endl;
        ims::ECBSSIPP planner(params);
        std::cout << "Initializing the planner..." << std::endl;
        planner.initializePlanner(action_spaces, start_state_vals, goal_state_vals);
        std::cout << "Planning..." << std::endl;
        // Plan.
        planner.plan(paths);
        stats = planner.reportStats();
    }


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

    std::cout << GREEN << "Planning time: " << stats.time << " sec" << std::endl;
    std::cout << "cost: " << stats.cost << std::endl;
    std::cout << "Number of nodes expanded: " << stats.num_expanded << std::endl;
    std::cout << "Number of nodes generated: " << stats.num_generated << std::endl;
    std::cout << "suboptimality: " << stats.suboptimality << RESET << std::endl;

    // Save the paths to a paths.yaml file.
    std::ofstream fout("paths.yaml");
    // Merge the full_path and the map path, adjusting for '../' in the map path.
    // std::string map_global_path = (boost::filesystem::weakly_canonical(boost::filesystem::current_path() / boost::filesystem::path(instance.map_file_))).string();
    string map_global_path = boost::filesystem::weakly_canonical(instance.map_file_).string();
    // string map_global_path = (boost::filesystem::weakly_canonical(boost::filesystem::current_path() / boost::filesystem::path(instance.map_file_))).string();


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

    // Execute the visualization script.
    std::string command = "python3 ../src/scripts/visualize_2d_time_paths.py --paths-yaml paths.yaml --fps 1 && xdg-open paths.gif";
    std::cout << "Running the plot script..." << std::endl;
    system(command.c_str());

    return 0;
}


