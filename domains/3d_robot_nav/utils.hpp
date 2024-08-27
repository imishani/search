//
// Created by itamar on 8/7/23.
//

#ifndef SEARCH_SEARCH_DOMAINS_2D_ROBOT_NAV_UTILS_HPP_
#define SEARCH_SEARCH_DOMAINS_2D_ROBOT_NAV_UTILS_HPP_

#include <fstream>
#include "search/common/types.hpp"

double radsToDegs(double rads)
{
    return rads * 180.0 / M_PI;
}

double degsToRads(double degs)
{
    return degs * M_PI / 180.0;
}

bool loadMap(const char *fname, std::string& type, int& width, int& height, int& depth, 
             std::vector<std::vector<std::vector<int>>>& map, int scale=1) {
    FILE *f = fopen(fname, "r");
    if (!f) {
        std::cerr << "Error: Unable to open file " << fname << std::endl;
        return false;
    }

    if (fscanf(f, "type octile\nheight %d\nwidth %d\ndepth %d\nmap\n", &height, &width, &depth) != 3) {
        std::cerr << "Error: Invalid file format" << std::endl;
        fclose(f);
        return false;
    }

    std::cout << "Loading map with dimensions: " << width << " x " << height << " x " << depth << std::endl;

    if (static_cast<long long>(depth) * width * height > 1e9) {
        std::cerr << "Error: Map dimensions are too large" << std::endl;
        fclose(f);
        return false;
    }

    try {
        map.resize(depth, std::vector<std::vector<int>>(width, std::vector<int>(height)));
    } catch (const std::bad_alloc& e) {
        std::cerr << "Error: Unable to allocate memory for the map" << std::endl;
        fclose(f);
        return false;
    }

    for (int z = 0; z < depth; z++) {
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                char c;
                do {
                    if (fscanf(f, "%c", &c) != 1) {
                        std::cerr << "Error: Unexpected end of file" << std::endl;
                        fclose(f);
                        return false;
                    }
                } while (isspace(c));
                map[z][x][y] = (c == '.' || c == 'G' || c == 'S' || c == 'T') ? 0 : 100;
            }
        }
        char c;
        fscanf(f, "%c", &c);
    }

    fclose(f);
    std::cout << "Map loaded successfully" << std::endl;

    if (scale != 1) {
        std::cout << "Scaling map by factor of " << scale << std::endl;
        int scaled_depth = scale * depth;
        int scaled_height = scale * height;
        int scaled_width = scale * width;

        if (static_cast<long long>(scaled_depth) * scaled_width * scaled_height > 1e9) {
            std::cerr << "Error: Scaled map dimensions are too large" << std::endl;
            return false;
        }

        std::vector<std::vector<std::vector<int>>> scaled_map;
        try {
            scaled_map.resize(scaled_depth, std::vector<std::vector<int>>(scaled_width, std::vector<int>(scaled_height)));
        } catch (const std::bad_alloc& e) {
            std::cerr << "Error: Unable to allocate memory for the scaled map" << std::endl;
            return false;
        }

        for (int z = 0; z < scaled_depth; z++) {
            for (int y = 0; y < scaled_height; y++) {
                for (int x = 0; x < scaled_width; x++) {
                    scaled_map[z][x][y] = map[z/scale][x/scale][y/scale];
                }
            }
        }

        map = std::move(scaled_map);
        depth = scaled_depth;
        height = scaled_height;
        width = scaled_width;
        std::cout << "Map scaled successfully. New dimensions: " << width << " x " << height << " x " << depth << std::endl;
    }

    return true;
}

void loadStartsGoalsFromFile(std::vector<std::vector<double>>& starts, 
                             std::vector<std::vector<double>>& goals, 
                             int scale, int num_runs, const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open file: " + path);
    }

    starts.clear();
    goals.clear();

    std::string line;
    int count = 0;
    while (std::getline(file, line) && count < num_runs) {
        std::istringstream iss(line);
        std::vector<double> start(3), goal(3);

        for (int i = 0; i < 3; ++i) {
            if (!(iss >> start[i])) {
                throw std::runtime_error("Error reading start position");
            }
            start[i] *= scale;
        }

        for (int i = 0; i < 3; ++i) {
            if (!(iss >> goal[i])) {
                throw std::runtime_error("Error reading goal position");
            }
            goal[i] *= scale;
        }

        starts.push_back(start);
        goals.push_back(goal);
        ++count;
    }

    if (starts.empty() || goals.empty()) {
        throw std::runtime_error("No start/goal positions loaded");
    }

    std::cout << "Loaded " << starts.size() << " start/goal pairs" << std::endl;
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
    double x1 = start[0], y1 = start[1], z1 = start[2];
    double x2 = end[0], y2 = end[1], z2 = end[2];

    double x_discretization = state_discretization[0];
    double y_discretization = state_discretization[1];
    double z_discretization = state_discretization[2];
    
    double discretized_distance = sqrt(pow((x1 - x2)/x_discretization, 2) + 
                                       pow((y1 - y2)/y_discretization, 2) + 
                                       pow((z1 - z2)/z_discretization, 2));
    double delta_t = 1 / (2 * discretized_distance);
    
    PathType points;
    for (double t = 0; t <= 1; t += delta_t) {
        double x = x1 + t * (x2 - x1);
        double y = y1 + t * (y2 - y1);
        double z = z1 + t * (z2 - z1);
        points.push_back({x, y, z});
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
    std::string path_file = "paths_tmp.csv";
    std::ofstream file(path_file);
    file << "Problem,Scale,PathsNumber," << map_index << "," << scale << "," << paths.size() << std::endl;
    for (const auto& path : paths) {
        file << path.first << "," << path.second.size() << std::endl;
        for (const auto& state : path.second) {
            file << state[0] << "," << state[1] << "," << state[2] << std::endl;
        }
    }
    file.close();
    return path_file;
}

std::string logActionPrimsMap(const std::map<double, std::pair<std::vector<Action>, std::vector<Action>>>& action_prims_map, 
                              const StateType& state_discretization) {
    std::string action_prims_map_file = "action_prims_map_tmp.csv";
    std::ofstream file(action_prims_map_file);
    
    std::ostringstream oss;
    std::copy(state_discretization.begin(), state_discretization.end() - 1, std::ostream_iterator<double>(oss, " "));
    oss << state_discretization.back();
    file << "ActionPrimsNumber," << action_prims_map.size() << ",StateDiscretization," << oss.str() << std::endl;
    for (const auto& theta_action_prims : action_prims_map) {
        file << theta_action_prims.first << "," << theta_action_prims.second.first.size() << "," << theta_action_prims.second.second.size() << std::endl;
        for (const Action& action_prim : theta_action_prims.second.first) {
            file << action_prim[0] << "," << action_prim[1] << "," << action_prim[2] << "," << action_prim[3] << std::endl;
        }
        for (const Action& action_prim : theta_action_prims.second.second) {
            file << action_prim[0] << "," << action_prim[1] << "," << action_prim[2] << "," << action_prim[3] << std::endl;
        }
    }
    file.close();
    return action_prims_map_file;
}


#endif //SEARCH_SEARCH_DOMAINS_2D_ROBOT_NAV_UTILS_HPP_
