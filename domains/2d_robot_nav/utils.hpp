//
// Created by itamar on 8/7/23.
//

#ifndef SEARCH_SEARCH_DOMAINS_2D_ROBOT_NAV_UTILS_HPP_
#define SEARCH_SEARCH_DOMAINS_2D_ROBOT_NAV_UTILS_HPP_

// standard includes
#include <fstream>
// library includes
#include "search/common/types.hpp"

double radsToDegs(double rads)
{
    return rads * 180.0 / M_PI;
}

double degsToRads(double degs)
{
    return degs * M_PI / 180.0;
}

std::vector<std::vector<int>> loadMap(const char *fname) {
    int width, height;
    std::vector<std::vector<int>> map;
    FILE *f;
    f = fopen(fname, "r");
    char type[99];

    if (f) {
        if (fscanf(f, "type %s\nheight %d\nwidth %d\nmap\n", type, &height, &width)) {
            map.resize(width, std::vector<int>(height));

            for (int y = 0; y < height; y++) {
                for (int x = 0; x < width; x++) {
                    if (!strcmp(type, "octile_costmap")) {
                        int d;
                        fscanf(f, "%d", &d);
                        map[x][y] = d;
                    }
                    else if (!strcmp(type, "octile")) {
                        char c;
                        do {
                            fscanf(f, "%c", &c);
                        } while (isspace(c));

                        map[x][y] = (c == '.' || c == 'G' || c == 'S' || c == 'T') ? 0 : 100;
                    }
                    else {
                        throw std::runtime_error("Invalid map type.");
                    }
                }
            }
        }
        else {
            throw std::runtime_error("Unable to scan file.");
        }

        fclose(f);
    }
    else {
        throw std::runtime_error("Unable to open file.");
    }

    return map;
}

void loadStartsGoalsFromFile(std::vector<std::vector<double>>& starts, std::vector<std::vector<double>>& goals, 
                             int scale, int num_runs, const std::string& path)
{
    assert(scale == 1);
    std::ifstream starts_fin(path + "nav2d_starts.txt");
    std::ifstream goals_fin(path + "nav2d_goals.txt");

    for (int j = 0; j < num_runs; ++j) {
        std::vector<double> start, goal;
        double val_start, val_goal;
        for (int i = 0; i < 2; ++i) {
            starts_fin >> val_start;
            goals_fin >> val_goal;
            start.push_back(scale*val_start);
            goal.push_back(scale*val_goal);
        }

        starts.emplace_back(start);
        goals.emplace_back(goal);

        double cost, length;
        starts_fin >> cost;
        starts_fin >> length;
    }
}

/// @brief Rounds the number to the nearest multiple of the discretization.
/// @param discretization The rounding factor.
/// @param num The number to be rounded.
/// @return The discretizaed version of the number.
double roundByDiscretization(double discretization, double num) {
    return round(num / discretization) * discretization; 
}

/// @brief Discretizes each element in the action vector according to the state_discretization vector.
/// @param state A vector representing the state of the robot.
/// @param state_discretization A vector contains the numbers each state value should be rounded to the closest multiple of.
/// @return The discretized version of the inputed state.
StateType discretizeState(const StateType& state, const StateType& state_discretization) {
    StateType discretized_state = {};
    for (int i = 0; i < state.size(); i++) {
        discretized_state.push_back(roundByDiscretization(state_discretization[i], state[i]));
    }
    return discretized_state;
}

/// @brief Removes duplicate actions from a vector of actions.
/// @param actions A vector of actions (a.k.a a vector of vectors).
/// @return The input vector with all the duplicates removed.
std::vector<Action> removeDuplicateActions(std::vector<Action> actions) {
    // Step 1: Sort the vector
    std::sort(actions.begin(), actions.end());

    // Step 2: Use std::unique to rearrange the vector and get the end iterator of the unique elements
    auto last = std::unique(actions.begin(), actions.end());

    // Step 3: Erase the duplicates from the vector
    actions.erase(last, actions.end());
    return actions;
}

/// @brief Given two points on a grid, this function calculates the discretized points on the line segment that connects the points.
/// @param start The starting state of the line segment.
/// @param end The ending state of the line segment.
/// @param state_discretization A vector contains the numbers each state value should be rounded to the closest multiple of.
/// @return A list of discretized states (PathType) on the line segment.
PathType getDiscretePointsOnLine(const StateType& start, const StateType& end, const StateType& state_discretization) {
    double x1 = start[0];
    double x2 = end[0];
    double y1 = start[1];
    double y2 = end[1];

    double x_discretization = state_discretization[0];
    double y_discretization = state_discretization[1];
    
    double discretized_distance  = sqrt(pow(((x1 - x2)/x_discretization),2) + pow(((y1 - y2)/y_discretization),2));
    double delta_t = 1 / (2 * discretized_distance);
    
    double theta = end[2];
    PathType points = {};
    for (double t = 0; t <= 1; t += delta_t) {
        double x = x1*t + x2*(1-t);
        double y = y1*t + y2*(1-t);
        points.push_back({x, y, theta});
    }
    return points;
}

/// @brief Log the statistics of a planner to a file.
/// @param stats The statistics of the planner.
/// @param map_index The index of the map.
/// @param planner_name The name of the planner.
void logStats(const std::unordered_map<int, PlannerStats>& stats,
              int map_index,
              const std::string& planner_name) {
    // save the logs to a file in current directory
    std::string log_file = "logs_map" + std::to_string(map_index) + "_" + planner_name + ".csv";
    // save logs object to a file
    std::ofstream file(log_file);
    // header line
    file << "Problem,Time,Cost,Num_expanded,Num_generated" << std::endl;
    for (auto& log : stats) {
        file << log.first << "," << log.second.time << "," << log.second.cost << "," <<
             log.second.num_expanded << "," << log.second.num_generated << std::endl;
    }
    file.close();
}

std::string logPaths(const std::unordered_map<int, PathType>& paths,
              int map_index, int scale, int threshold=500) {

    // save the paths to a temporary file
    std::string path_file = "paths_tmp.csv";
    // save paths object to a file
    std::ofstream file(path_file);
    // header line
    file << "Problem,Scale,Threshold,PathsNumber," << map_index << "," << scale << "," << threshold << "," << paths.size() << std::endl;
    for (auto& path : paths) {
        file << path.first << "," << path.second.size() << std::endl;
        for (auto& state : path.second) {
            file << state[0] << "," << state[1] << std::endl;
        }
    }
    file.close();
    return path_file;
}

void process_start_goal(const std::vector<std::vector<int>>& map, std::vector<double>& start, 
                                                            std::vector<double>& goal) {
    // round the start and goal to the nearest integer
    std::cout << "Start: " << start[0] << ", " << start[1] << std::endl;
    std::cout << "Goal: " << goal[0] << ", " << goal[1] << std::endl;
    for (int j {0}; j < 2; j++) {
        start[j] = std::round(start[j]);
        goal[j] = std::round(goal[j]);
    }
    std::cout << "Rounded Start: " << start[0] << ", " << start[1] << std::endl;
    std::cout << "Rounded Goal: " << goal[0] << ", " << goal[1] << std::endl;

    // print the value in the map
    std::cout << "Start value: " << map[(int)start[0]][(int)start[1]] << std::endl;
    std::cout << "Goal value: " << map[(int)goal[0]][(int)goal[1]] << std::endl;
}

int find_plan(std::shared_ptr<ims::ActionSpace> ActionSpace, ims::Planner *planner,
              const std::vector<double> &start, const std::vector<double> &goal, std::vector<StateType> *path_) {        
    // catch the exception if the start or goal is not valid
    try {
        planner -> initializePlanner(ActionSpace, start, goal);
    }
    catch (std::exception& e) {
        std::cout << "Start or goal is not valid!" << std::endl;
        return 1;
    }

    std::cout << "Planning..." << std::endl;
    if (!planner -> plan(*path_)) {
        std::cout << "No path found!" << std::endl;
        std::cout << "Rounded Start1: " << start[0] << ", " << start[1] << std::endl;
        std::cout << "Rounded Goal1: " << goal[0] << ", " << goal[1] << std::endl;
        return 1;
    }
    else
        std::cout << "Path found!" << std::endl;

    PlannerStats stats = planner -> reportStats();
    std::cout << GREEN << "Planning time: " << stats.time << " sec" << std::endl;
    std::cout << "cost: " << stats.cost << std::endl;
    std::cout << "Path length: " << path_ -> size() << std::endl;
    std::cout << "Number of nodes expanded: " << stats.num_expanded << std::endl;
    std::cout << "Number of nodes generated: " << stats.num_generated << std::endl;
    std::cout << "suboptimality: " << stats.suboptimality << RESET << std::endl;

    return 0;
}

void display_image(std::unordered_map<int, PathType> paths, int mapidx, int scale, int threshold) {
    boost::filesystem::path full_path( boost::filesystem::current_path() );
    std::string path_file = logPaths(paths, mapidx, scale, threshold);

    std::string plot_path = full_path.string() + "/../domains/2d_robot_nav/scripts/visualize_paths.py";
    std::string command = "python3 " + plot_path + " --filepath " + path_file;
    std::cout << "Running the plot script..." << std::endl;
    system(command.c_str());
}

void findAllPlans(std::shared_ptr<ims::ActionSpace> ActionSpace, ims::Planner *planner,
                  std::vector<std::vector<double>> starts,
                  std::vector<std::vector<double>> goals,
                  std::vector<std::vector<int>>* map,
                  int mapidx, int scale, int threshold) {

    // log the results
    // std::unordered_map<int, FocalSearchPlannerStats> logs;
    std::unordered_map<int, PathType> paths;

    for (int i {0}; i < starts.size(); i++){
        process_start_goal(*map, starts[i], goals[i]); // modifies starts and goals
        std::vector<StateType> path_;
        if (find_plan(ActionSpace, planner, starts[i], goals[i], &path_) != 0) {
            continue;
        }
        // logs[i] = stats; // log the stats
        paths[i] = path_; // log the path

        planner->resetPlanningData();
    }
    display_image(paths, mapidx, scale, threshold);
}

#endif //SEARCH_SEARCH_DOMAINS_2D_ROBOT_NAV_UTILS_HPP_
