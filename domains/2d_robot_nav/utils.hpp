//
// Created by itamar on 8/7/23.
//

#ifndef SEARCH_SEARCH_DOMAINS_2D_ROBOT_NAV_UTILS_HPP_
#define SEARCH_SEARCH_DOMAINS_2D_ROBOT_NAV_UTILS_HPP_

// standard includes
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
// library includes
#include "search/common/types.hpp"
#include "action_space_2d_rob.hpp"

double radsToDegs(double rads)
{
    return rads * 180.0 / M_PI;
}

std::vector<std::vector<int>> loadMap(const char *fname, int& width, int& height, bool cost_map=false) {

    std::vector<std::vector<int>> map;
    FILE *f;
    f = fopen(fname, "r");

    if (f) {
        if (fscanf(f, "type octile\nheight %d\nwidth %d\nmap\n", &height, &width)) {
            map.resize(width, std::vector<int>(height));

            for (int y = 0; y < height; y++) {
                for (int x = 0; x < width; x++) {
                    if (cost_map) {
                        int d;
                        fscanf(f, "%d", &d);
                        map[x][y] = d;
                    }
                    else {
                        char c;
                        do {
                            fscanf(f, "%c", &c);
                        } while (isspace(c));

                        map[x][y] = (c == '.' || c == 'G' || c == 'S' || c == 'T') ? 0 : 100;
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

void map_to_image(cv::Mat *img, const std::vector<std::vector<int>> &map, int height, int width, 
                  int scale, bool cost_map=false, int threshold=500) {
    assert(scale == 1);
    
    std::vector<std::vector<int>> scaled_map;
    int scaled_height = scale*height;
    int scaled_width = scale*width;
    scaled_map.resize(scaled_width, std::vector<int>(scaled_height));

    for (int y = 0; y < scaled_height; y++) {
        for (int x = 0; x < scaled_width; x++) {
            scaled_map[x][y] = map[x/scale][y/scale];
        }
    }

    *img = cv::Mat(scaled_height, scaled_width, CV_8UC3);

    for (int y = 0; y < scaled_height; y++) {
        for (int x = 0; x < scaled_width; x++) {
            if (cost_map) {
                int gray = 255 - (scaled_map[x][y] * 255 / threshold);
                (*img).at<cv::Vec3b>(y,x) = (scaled_map[x][y] >= threshold) ? cv::Vec3b(0,0,0) : cv::Vec3b(gray,gray,gray);
            }
            else {
                (*img).at<cv::Vec3b>(y,x) = (scaled_map[x][y] > 0) ? cv::Vec3b(0,0,0) : cv::Vec3b(255,255,255);
            }
        }
    }
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

int find_plan(std::shared_ptr<actionSpace2dRob> ActionSpace, ims::Planner *planner, 
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

void draw_paths(cv::Mat &img, const std::vector<double> &start, 
                const std::vector<double> &goal, const std::vector<StateType> &path_) {
    // draw the start in red and goal in green
    img.at<cv::Vec3b>((int)start[1], (int)start[0]) = cv::Vec3b(0,0,255);
    img.at<cv::Vec3b>((int)goal[1], (int)goal[0]) = cv::Vec3b(0,255,0);
    cv::circle(img, cv::Point((int)start[0], (int)start[1]), 2, cv::Scalar(0,0,255), 1);
    cv::circle(img, cv::Point((int)goal[0], (int)goal[1]), 2, cv::Scalar(0,255,0), 1);

    // draw the path in blue but skip the start and goal
    for (int j {1}; j < path_.size()-1; j++) {
        img.at<cv::Vec3b>((int)path_[j][1], (int)path_[j][0]) = cv::Vec3b(255,0,0);
    }
}

void display_image(const cv::Mat &img) {
    cv::namedWindow("Map", cv::WINDOW_NORMAL);
    cv::imshow("Map", img);
    cv::waitKey(0);
}

#endif //SEARCH_SEARCH_DOMAINS_2D_ROBOT_NAV_UTILS_HPP_
