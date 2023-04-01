/*
 * Copyright (C) 2023, Itamar Mishani
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
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   3/28/23
*/


#include <vector>
#include <memory>
#include <iostream>
#include <fstream>
#include <string>
#include <utility>
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


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

    std::ifstream file(fname);
    std::string str;
    std::vector<std::vector<int>> map;
    std::vector<int> row;
    int i = 0;
    while (std::getline(file, str)) {
        if (i == 0) {
            /// the line should be "type octile" so we want type = octile
            type = str.substr(5);
        } else if (i == 1) {
            /// the line should be "height 100" so we want height = 100
            width = std::stoi(str.substr(7));
        } else if (i == 2) {
            /// the line should be "width 100" so we want width = 100
            height = std::stoi(str.substr(6));
        } else if (i == 3) {
            /// the line should be "map" so we want to skip it but first, we resize the map
            map.resize(width, std::vector<int>(height));
            continue;
        } else {
            /// For now we assume the map is of type "octile".
            for (int x = 0; x < width; x++)
            {
                char c;
                do {
                    // fscanf(f, "%c", &c);
                    c = str[x];
                } while (isspace(c));
                map[x][i] = (c == '.' || c == 'G' || c == 'S' || c == 'T') ? 0 : 100;
            }
        }
        i++;
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
        std::cout << "Usage: " << argv[0] << " <map_file> <num_runs> <scale> <path>" << std::endl;
        return 0;
    }
    std::vector<std::string> maps;
    // get current directory using boost

    maps.emplace_back("/home/itamar/work/code/algorithms/search/domains/2d_robot_nav/data/hrt201n/hrt201n.map");
    maps.emplace_back("/home/itamar/work/code/algorithms/search/domains/2d_robot_nav/data/den501d/den501d.map");
    maps.emplace_back("/home/itamar/work/code/algorithms/search/domains/2d_robot_nav/data/den520d/den520d.map");
    maps.emplace_back("/home/itamar/work/code/algorithms/search/domains/2d_robot_nav/data/ht_chantry/ht_chantry.map");
    maps.emplace_back("/home/itamar/work/code/algorithms/search/domains/2d_robot_nav/data/brc203d/brc203d.map");

    std::vector<std::string> starts_goals_path = {"/home/itamar/work/code/algorithms/search/domains/2d_robot_nav/data/hrt201n/",
                                        "/home/itamar/work/code/algorithms/search/domains/2d_robot_nav/data/den501d/",
                                        "/home/itamar/work/code/algorithms/search/domains/2d_robot_nav/data/den520d/",
                                        "/home/itamar/work/code/algorithms/search/domains/2d_robot_nav/data/ht_chantry/",
                                        "/home/itamar/work/code/algorithms/search/domains/2d_robot_nav/data/brc203d/",
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

    std::cout << "Map: " << img << std::endl;

    cv::namedWindow("Map", cv::WINDOW_NORMAL);
    cv::imshow("Map", img);
    cv::waitKey(0);

    return 0;
}

