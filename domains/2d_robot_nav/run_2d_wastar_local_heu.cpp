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
 * \file   run_2d_wastar_local_heu.cpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   8/17/23
*/


#include <boost/filesystem.hpp>
#include <vector>
#include <memory>
#include <iostream>
#include <string>
#include <cmath>

// project includes
#include <search/planners/wastar.hpp>
#include <search/heuristics/standard_heuristics.hpp>
#include "action_space_2d_rob.hpp"
#include "utils.hpp"
#include "instance.h"


int main(int argc, char** argv) {
    ROBOTNAVInstance instance;
    instance.loadBenchmarkInstance(argc, argv);

    // construct planner params
    std::cout << "Constructing planner..." << std::endl;
    auto* heuristic = new ims::EuclideanHeuristic();
    double epsilon = 10.0;

    // construct the scene and the action space
    Scene2DRob scene (instance.getMap());
    ActionType2dRob action_type;
    std::shared_ptr<actionSpace2dRob> ActionSpace = std::make_shared<actionSpace2dRob>(scene, action_type);
    std::shared_ptr<actionSpace2dRob> ActionSpace2 = std::make_shared<actionSpace2dRob>(scene, action_type);
    auto* local_heuristic = new ims::LocalHeuristic(ActionSpace2, heuristic, 4);
    ims::wAStarParams params (local_heuristic, epsilon);

    ims::wAStar planner(params);
    findAllPlans(ActionSpace, &planner, instance.getRawStarts(), instance.getRawGoals(),
                 instance.getMap(), instance.getMapIdx(), instance.getScale(), instance.getThreshold());

    return 0;
}
