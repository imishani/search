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
    if (successor->in_closed){
      continue;
    }
    if (isGoalState(successor_id) && params_.verbose ){
      std::cout << "Added Goal to open list" << std::endl;
    }
    if (successor->in_open){
      if (successor->g > state_->g + cost){
        successor->parent_id = state_->state_id;
        successor->parent_ids_.insert(state_id);
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

// Breadth-First Search (BFS) function
void ims::MultiPathwAStar::BFSUtil(const std::unordered_map<int, std::unordered_set<int>>& graph, int startVertex) {
  std::unordered_set<int> visited;
  std::queue<int> queue;

  queue.push(startVertex);

  while (!queue.empty()) {
    int currentVertex = queue.front();
    queue.pop();

    if (visited.find(currentVertex) != visited.end()) {
      continue; // Skip if vertex is already visited
    }

    visited.insert(currentVertex);

    // Iterate over adjacent vertices
    const auto& adjacentVertices = graph.at(currentVertex);
    for (int adjacentVertex : adjacentVertices) {
      if (visited.find(adjacentVertex) == visited.end()) {
        queue.push(adjacentVertex);
      }
    }
  }
}


void ims::MultiPathwAStar::reconstructPath(std::vector<StateType>& path, std::vector<double>& costs) {
  path.clear();
  costs.clear();

  /// Construct the backward graph
  std::unordered_map<int, std::vector<int>> bwg;
  std::unordered_map<int, std::vector<double>> bwg_costs;
  for (auto s : states_) {
    SearchState* sid = getSearchState(s->state_id);
    for (const auto bp : sid->parent_ids_) {
      bwg[s->state_id].push_back(bp);
      double c = s->g - getSearchState(s->parent_id)->g;
      bwg_costs[s->state_id].push_back(c);
    }
  }


  costs.push_back(0); // The goal state gets a transition cost of 0.
  SearchState* state_ = getSearchState(goal_);
  while (state_->parent_id != -1){
    path.push_back(action_space_ptr_->getRobotState(state_->state_id)->state);

    // Get the transition cost. This is the difference between the g values of the current state and its parent.
    double transition_cost = state_->g - getSearchState(state_->parent_id)->g;
    costs.push_back(transition_cost);

    state_ = getSearchState(state_->parent_id);
  }
  path.push_back(action_space_ptr_->getRobotState(state_->state_id)->state);

  std::reverse(path.begin(), path.end());
  std::reverse(costs.begin(), costs.end());
}

void ims::MultiPathwAStar::reconstructPath(std::vector<StateType>& path) {
  SearchState* state_ = getSearchState(goal_);
  while (state_->parent_id != -1){
    path.push_back(action_space_ptr_->getRobotState(state_->state_id)->state);
    state_ = getSearchState(state_->parent_id);
  }
  path.push_back(action_space_ptr_->getRobotState(state_->state_id)->state);
  std::reverse(path.begin(), path.end());
}


