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
 * \file   run_2d_wastar.cpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   3/28/23
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
#include "action_space_2d_arm.hpp"
#include "utils.hpp"
#include "instance.h"
#include <box2d/box2d.h>

void construct_obstacles(vector<b2PolygonShape> &obstacle_polygons,
                         vector<b2Transform> &obstacle_transforms) {
    b2PolygonShape obst_polygon1;
    b2Transform obst_transform1;
    obst_polygon1.SetAsBox(5.0f, 5.0f, b2Vec2 (0.0f, 0.0f), 0.0f);
    obst_transform1.Set(b2Vec2(10.0f, 15.0f), 0.0f);
    obstacle_polygons.push_back(obst_polygon1);
    obstacle_transforms.push_back(obst_transform1);
    obst_polygon1.SetAsBox(15.0f, 1.0f, b2Vec2 (15.0f, 0.0f), 0.0f);
    obst_transform1.Set(b2Vec2(-15.0f, -2.0f), 0.0f);
    obstacle_polygons.push_back(obst_polygon1);
    obstacle_transforms.push_back(obst_transform1);
}

int main(int argc, char** argv) {
    // ROBOTNAVInstance instance;
    // instance.loadBenchmarkInstance(argc, argv);

    // construct planner params
    std::cout << "Constructing planner..." << std::endl;
    auto* heuristic = new ims::EuclideanHeuristic();
    double epsilon = 10.0;
    ims::wAStarParams params (heuristic, epsilon);

    // construct the scene and the action space
    std::vector<float> link_lengths = {10.0f, 10.0f, 10.0f}; //
    float angle_incr = 2.0f; //

    vector<b2PolygonShape> obstacle_polygons;
    vector<b2Transform> obstacle_transforms;
    construct_obstacles(obstacle_polygons, obstacle_transforms); //

    Scene2DArm scene (link_lengths, obstacle_polygons, obstacle_transforms);
    ActionType2dArm action_type (link_lengths.size(), angle_incr);
    std::shared_ptr<actionSpace2dArm> ActionSpace = std::make_shared<actionSpace2dArm>(scene, action_type);

    ims::wAStar planner(params);

    std::vector<std::vector<double>> starts = {{0.0f, 0.0f, 0.0f}}; //
    std::vector<std::vector<double>> goals = {{90.0f, 0.0f, 0.0f}}; //
    findAllPlansArm(ActionSpace, &planner, starts, goals);

    return 0;
}

