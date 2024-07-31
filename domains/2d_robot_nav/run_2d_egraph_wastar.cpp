//
// Created by spencer on 2024-06-12.
//
#include <boost/filesystem.hpp>
#include <vector>
#include <memory>
#include <iostream>
#include <fstream>
#include <string>
#include <random>
#include <cmath>

// project includes
#include "search/planners/egraph_wastar.hpp"
#include "search/heuristics/standard_heuristics.hpp"
#include "search/heuristics/base_heuristic.hpp"
#include "experience_heuristic_2d.hpp"
#include "action_space_2d_rob_experience.hpp"
#include "utils.hpp"



int main(int argc, char** argv) {

    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <map_file> <num_runs> <scale> <path>" << std::endl;
        return 0;
    }
    std::vector<std::string> maps;

    // boost::filesystem::path full_path( boost::filesystem::current_path() );
    // std::cout << "Current path is : " << full_path.string() << std::endl;

    // At each emplace_back, use the full pathh and concatenate the map name
    maps.emplace_back("/home/spencer/repos/search/domains/2d_robot_nav/data/hrt201n/hrt201n.map");
    maps.emplace_back("/home/spencer/repos/search/domains/2d_robot_nav/data/den501d/den501d.map");
    maps.emplace_back("/home/spencer/repos/search/domains/2d_robot_nav/data/den520d/den520d.map");
    maps.emplace_back("/home/spencer/repos/search/domains/2d_robot_nav/data/ht_chantry/ht_chantry.map");
    maps.emplace_back("/home/spencer/repos/search/domains/2d_robot_nav/data/brc203d/brc203d.map");

    std::vector<std::string> starts_goals_path = {"/home/spencer/repos/search/domains/2d_robot_nav/data/hrt201n/",
                                                  "/home/spencer/repos/search/domains/2d_robot_nav/data/den501d/",
                                                  "/home/spencer/repos/search/domains/2d_robot_nav/data/den520d/",
                                                  "/home/spencer/repos/search/domains/2d_robot_nav/data/ht_chantry/",
                                                  "/home/spencer/repos/search/domains/2d_robot_nav/data/brc203d/",
    };

    int map_index = std::stoi(argv[1]);
    int num_runs = std::stoi(argv[2]);
    int scale = std::stoi(argv[3]);
    // try to check argv[4] to check if the user wants to save the path for experience
    bool cache = false;
    try {
        cache = std::stoi(argv[4]);
    }
    catch (std::exception& e) {
        std::cout << YELLOW << "Didn't specify whether to save the path or not. Default is not to save." << RESET << std::endl;
    }

    std::string path = starts_goals_path[map_index];
    std::string experience_path = path + "experiences/";
    std::string map_file = maps[map_index];

    std::string type;
    int width, height;
    std::vector<std::vector<int>> map = loadMap(map_file.c_str(), type, width, height, scale);

    std::vector<std::vector<double>> starts, goals;
    loadStartsGoalsFromFile(starts, goals, scale, num_runs, path);

    double lower = -5;
    double upper = 5;
    std::uniform_real_distribution<double> unif(lower, upper);
    // unsigned seed = std::chrono::steady_clock::now().time_since_epoch().count();
    std::default_random_engine rd;

    //
    // for(int i{0}; i<starts.size(); i++)
    // {
    //     for(int j{0}; j<starts[i].size(); j++)
    //     {
    //         starts[i][j] += unif(rd);
    //     }
    // }
    //
    // for(int i{0}; i<goals.size(); i++)
    // {
    //     for(int j{0}; j<goals[i].size(); j++)
    //     {
    //         goals[i][j] += unif(rd);
    //     }
    // }


    // construct the planner
    std::cout << "Constructing planner..." << std::endl;
    // construct the scene and the action space

    // construct scene
    Scene2DRob scene (map);

    // log the results
    std::unordered_map<int, PlannerStats> logs;
    std::unordered_map<int, PathType> paths;
    for (int i {0}; i < starts.size(); i++){
        std::cout << "******************************************" << std::endl;
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

        // construct action type and other inputs for planner
        ActionType2dRob action_type;
        std::shared_ptr<ActionSpaceEGraph2DRob> ActionSpace = std::make_shared<ActionSpaceEGraph2DRob>(scene, action_type);

        double epsilon = 10.0;
        double egraph_epsilon = 5.0;
        std::shared_ptr<ims::EuclideanHeuristic> origin_heuristic = std::make_shared<ims::EuclideanHeuristic>();
        auto* heuristic = new ims::GenericEGraphHeuristic(origin_heuristic, ActionSpace);
        heuristic->setEGraphWeight(egraph_epsilon);

        // construct planner params
        ims::ExperienceWAStarParams params(heuristic, epsilon, egraph_epsilon, experience_path);
        // construct planner
        ims::ExperienceWAstar planner(params);
        // catch the exception if the start or goal is not valid
        try {
            planner.initializePlanner(ActionSpace, starts[i], goals[i]);
        }
        catch (std::exception& e) {
            std::cout << RED << "Start or goal is not valid!" << RESET << std::endl;
            continue;
        }
        // plan
        std::cout << "Planning..." << std::endl;
        std::vector<StateType> path_;
        if (!planner.plan(path_)) {
            std::cout << "No path found!" << std::endl;
            continue;
//            return 0;
        }
        else{
            std::cout << "Path found!" << std::endl;
            if (cache) {
                // Save the path to a file as csv
                std::string path_file = path + "experiences/" + "path_" + std::to_string(i) + ".csv";
                std::ofstream file(path_file);
                // header line
                file << "Experience," << path_.size() << "," << 2 << std::endl;
                // write the path
                for (int j {0}; j < path_.size(); j++){
                    file << path_[j][0] << "," << path_[j][1] << std::endl;
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

        delete heuristic;
    }

    // save the logs to a temporary file
    logStats(logs, map_index, "egraph_wastar");

    std::string path_file = logPaths(paths, map_index, scale);

    // std::string plot_path = full_path.string() + "/../domains/2d_robot_nav/scripts/visualize_paths.py";
    // std::string command = "python3 " + plot_path + " --filepath " + path_file;
    // std::cout << "Running the plot script..." << std::endl;
    //
    // system(command.c_str());

    return 0;
}
