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

std::vector<std::vector<int>> loadMap(const char *fname,
                                      std::string& type, int& width,
                                      int& height, int scale=1) {

    std::vector<std::vector<int>> map;
    FILE *f;
    f = fopen(fname, "r");

    if (f)
    {
        if (fscanf(f, "type octile\nheight %d\nwidth %d\nmap\n", &height, &width))
        {
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

    return scaled_map;
}

void loadStartsGoalsFromFile(std::vector<std::vector<double>>& starts, std::vector<std::vector<double>>& goals, int scale, int num_runs, const std::string& path)
{
    std::ifstream starts_fin(path + "nav2d_starts.txt");
    std::ifstream goals_fin(path + "nav2d_goals.txt");

    for (int j = 0; j < num_runs; ++j)
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
              int map_index,
              int scale) {
    // save the paths to a temporary file
    std::string path_file = "paths_tmp.csv";
    // save paths object to a file
    std::ofstream file(path_file);
    // header line
    file << "Problem,Scale,PathsNumber," << map_index << "," << scale << "," << paths.size() << std::endl;
    for (auto& path : paths) {
        file << path.first << "," << path.second.size() << std::endl;
        for (auto& state : path.second) {
            file << state[0] << "," << state[1] << std::endl;
        }
    }
    file.close();
    return path_file;
}


#endif //SEARCH_SEARCH_DOMAINS_2D_ROBOT_NAV_UTILS_HPP_
