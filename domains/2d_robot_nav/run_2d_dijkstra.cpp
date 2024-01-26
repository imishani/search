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
 * \file   run_2d_dijkstra.cpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   3/28/23
*/


#include <boost/filesystem.hpp>
#include <vector>
#include <memory>
#include <iostream>
#include <string>
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp> // Get the cv circle.

// project includes
#include <search/planners/dijkstra.hpp>
#include <search/heuristics/standard_heuristics.hpp>
#include "action_space_2d_rob.hpp"
#include "utils.hpp"
#include "instance.h"


int main(int argc, char** argv) {

    if (argc < 2) {
        std::cout << RED << "Usage: " << argv[0] << " <map_file> <num_runs> <scale> <path>" << RESET << std::endl;
        return 0;
    }
    int map_index = std::stoi(argv[1]);
    int num_runs = std::stoi(argv[2]);
    int scale = std::stoi(argv[3]);

    ROBOTNAVInstance instance;
    instance.loadBenchmarkInstance(map_index, num_runs, scale);
    std::vector<std::vector<double>> starts = instance.getRawStarts();
    std::vector<std::vector<double>> goals = instance.getRawGoals();
    cv::Mat img = instance.getImage();

    // construct the planner
    std::cout << "Constructing planner..." << std::endl;
    // construct planner params
    ims::DijkstraParams params;
    
    // construct the scene and the action space
    Scene2DRob scene (instance.getMap());
    ActionType2dRob action_type;
    std::shared_ptr<actionSpace2dRob> ActionSpace = std::make_shared<actionSpace2dRob>(scene, action_type);

    for (int i {0}; i < starts.size(); i++){
        process_start_goal(*(instance.getMap()), starts[i], goals[i]);
        // construct planner
        ims::Dijkstra planner(params);
        std::vector<StateType> path_;
        if (find_plan(ActionSpace, &planner, starts[i], goals[i], &path_) != 0) {
            continue;
        }
        draw_paths(img, starts[i], goals[i], path_);
    }
    display_image(img);

    return 0;
}

