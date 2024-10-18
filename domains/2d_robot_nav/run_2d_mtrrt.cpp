
#include <boost/filesystem.hpp>
#include <vector>
#include <memory>
#include <iostream>
#include <fstream>
#include <string>
#include <random>
#include <cmath>

// project includes
#include "search/planners/sampling_based/mtrrt.hpp"
#include "search/heuristics/standard_heuristics.hpp"
#include "search/heuristics/base_heuristic.hpp"
#include "action_space_2d_rob_mtrrt.hpp"
#include "utils.hpp"



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

    std::vector<std::vector<double>> starts, goals;
    loadStartsGoalsFromFile(starts, goals, scale, num_runs, path);

    // construct the planner
    std::cout << "Constructing planner..." << std::endl;
    // construct planner params
    auto heuristic = new ims::EuclideanHeuristic();
    double step_size = 7.0;
    int max_itr = 10000;
    ims::MTRRTParams params (heuristic, step_size, max_itr);
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

        std::shared_ptr<ActionSpaceMTRRT2DRob> ActionSpace = std::make_shared<ActionSpaceMTRRT2DRob>(scene, action_type);
        // construct planner
        ims::MTRRT planner(params);
        // catch the exception if the start or goal is not valid
        try {
            planner.initializePlanner(ActionSpace, starts[i], goals[i]);
        }
        catch (std::exception& e) {
            std::cout << RED << "Start or goal is not valid!" <<RESET << std::endl;
            continue;
        }
        // plan
        std::cout << "Planning..." << std::endl;
        std::vector<StateType> path_;
        if (!planner.plan(path_)) {
            std::cout << RED << "No path found!" << RESET << std::endl;
        }
        else
            std::cout << GREEN << "Path found!" << RESET << std::endl;
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
    logStats(logs, map_index, "wAstar");

    std::string path_file = logPaths(paths, map_index, scale);

    std::string plot_path = full_path.string() + "/../domains/2d_robot_nav/scripts/visualize_paths.py";
    std::string command = "python3 " + plot_path + " --filepath " + path_file;
    std::cout << "Running the plot script..." << std::endl;

    system(command.c_str());

    return 0;
}
