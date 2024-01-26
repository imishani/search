/*
 * Copyright (C) 2023, Rishi V.
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
 * \file   instance.hpp
 * \author Rishi V. (...@cmu.edu)
 * \date   Oct 04 2023
 */

#pragma once

#include<boost/tokenizer.hpp> // For parsing
#include <memory>
#include <fstream>
#include <vector>
#include <string>
using std::string;
using std::vector;
#include <search/common/types.hpp>
#include "action_space_2d_rob.hpp"
#include "utils.hpp"

/// @brief Helper class that loads MAPF instances from files.
/// @note This class is used for loading the map and agents from a file.
/// @note Handles parsing custom and benchmark instances.
class ROBOTNAVInstance {
private:
    std::vector<std::vector<int>> map_;
    std::vector<std::vector<double>> starts_;
    std::vector<std::vector<double>> goals_;
    cv::Mat img_;
public:
    string map_file_;
    void loadBenchmarkInstance(int map_index, int num_runs, int scale, int threshold);
    std::vector<std::vector<int>>* getMap() {return &map_;}
    std::vector<std::vector<double>> getRawStarts() {return starts_;}
    std::vector<std::vector<double>> getRawGoals() {return goals_;}
    cv::Mat getImage() {return img_;}
};

////////////////// Implementations Below //////////////////////

const vector<string> idxToMapName = {
    "/../domains/2d_robot_nav/data/hrt201n/hrt201n.map",
    "/../domains/2d_robot_nav/data/den501d/den501d.map",
    "/../domains/2d_robot_nav/data/den520d/den520d.map",
    "/../domains/2d_robot_nav/data/ht_chantry/ht_chantry.map",
    "/../domains/2d_robot_nav/data/brc203d/brc203d.map",
    "/../domains/2d_robot_nav/data/costmap1/costmap1.map",
    "/../domains/2d_robot_nav/data/costmap2/costmap2.map"
};
const vector<string> idxToStartGoal = {
    "/../domains/2d_robot_nav/data/hrt201n/",
    "/../domains/2d_robot_nav/data/den501d/",
    "/../domains/2d_robot_nav/data/den520d/",
    "/../domains/2d_robot_nav/data/ht_chantry/",
    "/../domains/2d_robot_nav/data/brc203d/",
    "/../domains/2d_robot_nav/data/costmap1/",
    "/../domains/2d_robot_nav/data/costmap2/"
};

void ROBOTNAVInstance::loadBenchmarkInstance(int map_index, int num_runs, int scale, int threshold=500) {
    boost::filesystem::path full_path( boost::filesystem::current_path() );
    std::cout << "Current path is : " << full_path.string() << std::endl;

    std::string starts_goals_path = full_path.string() + idxToStartGoal[map_index];
    map_file_ = full_path.string() + idxToMapName[map_index];

    int width, height;
    map_ = loadMap(map_file_.c_str(), width, height, map_index>4);
    map_to_image(&img_, map_, height, width, scale, map_index>4, threshold);

    loadStartsGoalsFromFile(starts_, goals_, scale, num_runs, starts_goals_path);
}

