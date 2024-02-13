//  * Copyright (c) 2024, Ramkumar Natarajan
//  * All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without
//  * modification, are permitted provided that the following conditions are met:
//  *
//  *     * Redistributions of source code must retain the above copyright
//  *       notice, this list of conditions and the following disclaimer.
//  *     * Redistributions in binary form must reproduce the above copyright
//  *       notice, this list of conditions and the following disclaimer in the
//  *       documentation and/or other materials provided with the distribution.
//  *     * Neither the name of the Carnegie Mellon University nor the names of its
//  *       contributors may be used to endorse or promote products derived from
//  *       this software without specific prior written permission.
//  *
//  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
//  * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  * POSSIBILITY OF SUCH DAMAGE.
//

/*!
 * \file run_sliding_tiles_mpwa.cpp
 * \author Ram Natarajan (rnataraj@cs.cmu.edu)
 * \date 2/13/24
*/

#include <boost/filesystem.hpp>
#include <vector>
#include <memory>
#include <iostream>
#include <string>
#include <cmath>

// project includes
#include <search/planners/multi_path_wastar.hpp>
#include "action_space_sliding_puzzle.hpp"
#include "heuristics.hpp"
#include "utils.hpp"


int main(int argc, char** argv) {
//  srand(time(NULL));
  srand(12);

  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " <puzzle_size> <num_runs>" << std::endl;
    return 0;
  }
  std::string planner_name = "mpwa";
  std::string domain_name = "sliding_tiles";
  int puzzle_size = std::stoi(argv[1]);
  int num_runs = std::stoi(argv[2]);

  StateType goal;
  for (int i=1; i<puzzle_size*puzzle_size; ++i) {
    goal.push_back(i);
  }
  goal.push_back(0);
  std::string type;

  // construct the planner
  std::cout << "Constructing planner..." << std::endl;
  // construct planner params
  ManhattanPlusLinearConflictHeuristic* heuristic = new ManhattanPlusLinearConflictHeuristic();
  double epsilon = 10.0;
  int num_jumble = 750;
  ims::MultiPathwAStarParams params (heuristic, epsilon);
  // construct the scene and the action space
  SceneSlidingTiles scene (puzzle_size);
  ActionTypeSlidingTiles action_type;

  // log the results
  std::unordered_map<int, PlannerStats> logs;
  std::unordered_map<int, PathType> paths;
  for (int i {0}; i < num_runs; i++){
    // generate a random start
    StateType start = generateSolvableStartState(goal, num_jumble, puzzle_size);

    MatDf start_mat = Eigen::Map<const MatDf>(start.data(), puzzle_size, puzzle_size);
    MatDf goal_mat = Eigen::Map<const MatDf>(goal.data(), puzzle_size, puzzle_size);

    // round the start and goal to the nearest integer
    std::cout << "Start: \n" << start_mat << std::endl;
    std::cout << "Goal: \n" << goal_mat << std::endl;

    std::shared_ptr<actionSpaceSlidingTiles> ActionSpace = std::make_shared<actionSpaceSlidingTiles>(scene, action_type);
    // construct planner
    ims::MultiPathwAStar planner(params);

    // catch the exception if the start or goal is not valid
    try {
      planner.initializePlanner(ActionSpace, start, goal);
    }
    catch (std::exception& e) {
      std::cout << RED << "Start or goal is not valid!" <<RESET << std::endl;
      continue;
    }
    // plan
    std::cout << "Planning..." << std::endl;
    std::vector<StateType> path_;
    if (!planner.plan(path_)) {
      std::cout << RED << "No path found!" << RESET << std::endl;
    }
    else
      std::cout << GREEN << "Path found!" << RESET << std::endl;

//    std::cout << "path found is " << std::endl;
//    for (auto& state : path_)
//      std::cout << Eigen::Map<const MatDf>(state.data(), puzzle_size, puzzle_size) << std::endl << "-----------" << std::endl;

    PlannerStats stats = planner.reportStats();
    std::cout << GREEN << "Planning time: " << stats.time << " sec" << std::endl;
    std::cout << "cost: " << stats.cost << std::endl;
    std::cout << "Path length: " << path_.size() << std::endl;
    std::cout << "Number of nodes expanded: " << stats.num_expanded << std::endl;
    std::cout << "Number of nodes generated: " << stats.num_generated << std::endl;
    std::cout << "suboptimality: " << stats.suboptimality << RESET << std::endl;
    logs[i] = stats; // log the stats
    paths[i] = path_; // log the path

    auto sid_to_fval = planner.reconstructFValue();

    ///// DATA FORMAT /////
    /// planner
    /// map ID
    /// start
    /// goal
    /// epsilon (w)
    /// g h f state

    std::string fpath = "../domains/" + domain_name + "/data/phai/" + std::to_string(i) + ".txt";
    std::string map_id = "NA";
    planner.writeDataToFile(fpath, planner_name, map_id,
                            start, goal, epsilon, sid_to_fval);
  }

  return 0;
}