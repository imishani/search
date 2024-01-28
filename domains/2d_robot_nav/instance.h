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
 * \author Michelle L. (mmliu@andrew.cmu.edu)
 * \date   Dec 20 2023
 */

#pragma once

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp> // Optional arguments
#include <memory>
#include <fstream>
#include <vector>
#include <string>
using std::string;
using std::vector;
#include <search/common/types.hpp>
#include "action_space_2d_rob.hpp"
#include "utils.hpp"

/// @brief Helper class that loads ROBOTNAV instances from files.
/// @note Handles parsing custom and benchmark instances.
class ROBOTNAVInstance {
private:
    std::vector<std::vector<int>> map_;
    std::vector<std::vector<double>> starts_;
    std::vector<std::vector<double>> goals_;
    int mapIdx_;
    int scale_;
    int threshold_;
public:
    string map_file_;
    void loadBenchmarkInstance(int argc, char** argv);
    std::vector<std::vector<int>>* getMap() {return &map_;}
    std::vector<std::vector<double>> getRawStarts() {return starts_;}
    std::vector<std::vector<double>> getRawGoals() {return goals_;}
    int getMapIdx() {return mapIdx_;}
    int getScale() {return scale_;}
    int getThreshold() {return threshold_;}
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

void ROBOTNAVInstance::loadBenchmarkInstance(int argc, char** argv) {

    namespace po = boost::program_options;
    po::options_description desc("Options");
    desc.add_options()
        ("map_idx", po::value<int>()->default_value(0), "")
        ("num_runs", po::value<int>()->default_value(1), "")
        ("scale", po::value<int>()->default_value(1), "")
        ("threshold", po::value<int>()->default_value(500), "threshold > 0");

    po::variables_map vm;
    po::store(po::parse_command_line(argc,argv,desc),vm);
    po::notify(vm);

    mapIdx_ = vm["map_idx"].as<int>();
    int num_runs = vm["num_runs"].as<int>();
    scale_ = vm["scale"].as<int>();
    threshold_ = vm["threshold"].as<int>();

    // try to check argv[4] to check if the user wants to save the path for experience
    // bool cache = false;
    // try {
    //     cache = std::stoi(argv[4]);
    // }
    // catch (std::exception& e) {
    //     std::cout << YELLOW << "Didn't specify whether to save the path or not. Default is not to save." << RESET << std::endl;
    // }

    boost::filesystem::path full_path( boost::filesystem::current_path() );
    std::cout << "Current path is : " << full_path.string() << std::endl;

    string filename = idxToMapName[mapIdx_];
    boost::filesystem::path filePath(filename);
    string directory = filePath.parent_path().string() + "/";
    string starts_goals_path = full_path.string() + directory;
    map_file_ = full_path.string() + idxToMapName[mapIdx_];

    map_ = loadMap(map_file_.c_str());
    loadStartsGoalsFromFile(starts_, goals_, scale_, num_runs, starts_goals_path);
}
