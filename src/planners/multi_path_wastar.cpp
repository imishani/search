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
 * \file multi_path_wastar.cpp
 * \author Ram Natarajan (rnataraj@cs.cmu.edu)
 * \date 2/12/24
*/

#include "search/planners/multi_path_wastar.hpp"
#include <queue>
#include <fstream>

ims::MultiPathwAStar::MultiPathwAStar(const ims::MultiPathwAStarParams &params) : params_(params), wAStar(params) {}

ims::MultiPathwAStar::~MultiPathwAStar() {

}

void ims::MultiPathwAStar::expand(int state_id){

  auto state_ = getSearchState(state_id);
  std::vector<int> successors;
  std::vector<double> costs;
  action_space_ptr_->getSuccessors(state_->state_id, successors, costs);
  for (size_t i {0} ; i < successors.size() ; ++i){
    int successor_id = successors[i];
    double cost = costs[i];
    auto successor = getOrCreateSearchState(successor_id);
    assert(state_id>=0);
    successor->parent_ids_.insert(state_id); /// Keeping track of visitation by every state
    if (successor->in_closed){
      continue;
    }
    if (isGoalState(successor_id) && params_.verbose ){
      std::cout << "Added Goal to open list" << std::endl;
    }
    if (successor->in_open){
      if (successor->g > state_->g + cost){
        successor->parent_id = state_->state_id;
        successor->g = state_->g + cost;
        successor->f = successor->g + params_.epsilon*successor->h;
        open_.update(successor);
      }
    } else {
      setStateVals(successor->state_id, state_->state_id, cost);
      open_.push(successor);
      successor->setOpen();
    }
  }
  stats_.num_expanded++;
}


std::unordered_map<int, double> ims::MultiPathwAStar::Dijkstra(std::unordered_map<int, std::vector<int>>& graph,
                                                               std::unordered_map<int, std::vector<double>>& costs,
                                                               int start_id) {

  std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> pq;
  std::unordered_map<int, double> fval;

  fval[start_id] = 0.0;
  pq.emplace(0.0, start_id);

  int i = 0;
  while (!pq.empty()) {
    int u = pq.top().second;
    int dist_u = pq.top().first;
    pq.pop();

    if (dist_u > fval.at(u)) {
      continue; // Skip outdated entries in priority queue
    }

    for (int i=0; i<graph.at(u).size(); ++i) {
      int v = graph.at(u)[i];
      double weight = costs.at(u)[i];

      if (fval.find(v) == fval.end()) {
        fval[v] = fval.at(u) + weight;
        pq.emplace(fval.at(v), v);
      } else if (fval.at(u) + weight < fval.at(v)) {
        fval.at(v) = fval.at(u) + weight;
        pq.emplace(fval.at(v), v);
      }
    }
  }

  for (const auto id : fval) {
    fval.at(id.first) += getSearchState(id.first)->g;
  }

  return fval;
}


std::unordered_map<int, double> ims::MultiPathwAStar::reconstructFValue() {
  /// Construct the backward graph
  std::unordered_map<int, std::vector<int>> bwg;
  std::unordered_map<int, std::vector<double>> bwg_costs;
  for (auto s : states_) {
    SearchState* sid = getSearchState(s->state_id);
    for (const auto bp : sid->parent_ids_) {
      bwg[s->state_id].push_back(bp);
      double c = fabs(s->g - getSearchState(bp)->g);
      bwg_costs[s->state_id].push_back(c);
    }
  }
  auto id_to_fval = Dijkstra(bwg, bwg_costs, goal_);
  return id_to_fval;

//  std::unordered_map<ims::wAStar::SearchState*, double> state_to_fval;
//  for (const auto id : id_to_fval) {
//    state_to_fval[getSearchState(id.first)] = id.second;
//  }
//  return state_to_fval;
}

double ims::MultiPathwAStar::getGValue(int state_id) {
  return wAStar::getSearchState(state_id)->g;
}

double ims::MultiPathwAStar::getHValue(int state_id) {
  return wAStar::getSearchState(state_id)->h;
}

StateType ims::MultiPathwAStar::getState(int state_id) {
  return action_space_ptr_->getRobotState(state_id)->state;
}

void ims::MultiPathwAStar::writeDataToFile(std::string fpath, std::string planner_name,
                                           std::string map_id, std::vector<double> start, std::vector<double> goal,
                                           double epsilon, std::unordered_map<int, double>& sid_to_fval) {

  std::ofstream opfile(fpath);

  // Write each variable on a new line
  if (opfile.is_open()) {
    opfile << planner_name << std::endl;
    opfile << map_id << std::endl;
    for (const auto& val : start)
      opfile << val << " ";
    opfile << std::endl;
    for (const auto& val : goal)
      opfile << val << " ";
    opfile << std::endl;
    opfile << epsilon << std::endl;
    for (auto sidtof : sid_to_fval) {
      opfile << getGValue(sidtof.first) << " "
             << getHValue(sidtof.first) << " "
             << sid_to_fval[sidtof.first] << " ";
      auto state = getState(sidtof.first);
      for (auto s : state) {
        opfile << s << " ";
      }
      opfile << std::endl;
    }

    // Close the file
    opfile.close();
    std::cout << "Data has been written to data.txt" << std::endl;
  } else {
    std::cerr << "Unable to open file!" << std::endl;
  }
}
