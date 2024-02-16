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
    for (auto state_ : states_){
        state_->heap_data.~HeapData();
        delete state_;
    }
}


void ims::MultiPathwAStar::initializePlanner(const std::shared_ptr<ActionSpace> &action_space_ptr,
                                    const std::vector<StateType> &starts,
                                    const std::vector<StateType> &goals) {
    // space pointer
    action_space_ptr_ = action_space_ptr;
    // Clear both.
    action_space_ptr_->resetPlanningData();
    resetPlanningData();

    if (goals.empty() || starts.empty()) {
        throw std::runtime_error("Starts or goals are empty");
    }

    if (goals.size() > 1) {
        throw std::runtime_error("Currently, only one goal is supported");
    }
    // check if goal is valid
    if (!action_space_ptr_->isStateValid(goals[0])){
        throw std::runtime_error("Goal state is not valid");
    }
    int goal_ind_ = action_space_ptr_->getOrCreateRobotState(goals[0]);
    auto goal_ = getOrCreateSearchState(goal_ind_);
    goals_.push_back(goal_ind_);

    // Evaluate the goal state
    goal_->heap_data.parent_id = PARENT_TYPE(GOAL);
    heuristic_->setGoal(const_cast<StateType &>(goals[0]));
    goal_->heap_data.h = 0;

    for (auto &start : starts) {
        // check if start is valid
        if (!action_space_ptr_->isStateValid(start)){
            throw std::runtime_error("Start state is not valid");
        }
        // Evaluate the start state
        int start_ind_ = action_space_ptr_->getOrCreateRobotState(start);
        auto start_ = getOrCreateSearchState(start_ind_);
        start_->heap_data.parent_id = PARENT_TYPE(START);
        heuristic_->setStart(const_cast<StateType &>(start));
        start_->heap_data.g = 0;
        start_->heap_data.h = computeHeuristic(start_ind_);
        start_->heap_data.f = start_->heap_data.g + params_.epsilon*start_->heap_data.h;
        open_.push(&start_->heap_data);
        start_->heap_data.setOpen();
    }
    stats_.suboptimality = params_.epsilon;
}

void ims::MultiPathwAStar::initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
                                    const StateType& start, const StateType& goal) {
    // Space pointer.
    action_space_ptr_ = action_space_ptr;

    // Clear both.
    action_space_ptr_->resetPlanningData();
    resetPlanningData();

    // check if start is valid
    if (!action_space_ptr_->isStateValid(start)){
        throw std::runtime_error("Start state is not valid");
    }
    // check if goal is valid
    if (!action_space_ptr_->isStateValid(goal)){
        throw std::runtime_error("Goal state is not valid");
    }
    int start_ind_ = action_space_ptr_->getOrCreateRobotState(start);
    auto start_ = getOrCreateSearchState(start_ind_);

    int goal_ind_ = action_space_ptr_->getOrCreateRobotState(goal);
    auto goal_ = getOrCreateSearchState(goal_ind_);
    goals_.push_back(goal_ind_);

    start_->heap_data.parent_id = PARENT_TYPE(START);
    heuristic_->setStart(const_cast<StateType &>(start));
    // Evaluate the goal state
    goal_->heap_data.parent_id = PARENT_TYPE(GOAL);
    heuristic_->setGoal(const_cast<StateType &>(goal));
    goal_->heap_data.h = 0;
    // Evaluate the start state
    start_->heap_data.g = 0;
    start_->heap_data.h = computeHeuristic(start_ind_);
    start_->heap_data.f = start_->heap_data.g + params_.epsilon*start_->heap_data.h;
    start_->heap_data.setOpen();

    open_.push(&start_->heap_data);
    // update stats suboptimality
    stats_.suboptimality = params_.epsilon;
}

auto ims::MultiPathwAStar::getSearchState(int state_id) -> ims::MultiPathwAStar::SearchState * {
    assert(state_id < states_.size() && state_id >= 0);
    return states_[state_id];
}

auto ims::MultiPathwAStar::getOrCreateSearchState(int state_id) -> ims::MultiPathwAStar::SearchState * {
    if (state_id >= states_.size()){
        states_.resize(state_id + 1, nullptr);
    }
    if (states_[state_id] == nullptr){
        assert(state_id < states_.size() && state_id >= 0);
        states_[state_id] = new SearchState;
        states_[state_id]->heap_data.state_id = state_id;
    }
    return states_[state_id];
}

bool ims::MultiPathwAStar::plan(std::vector<StateType>& path) {
    startTimer();
    int iter {0};
    while (!open_.empty() && !isTimeOut()){
        // report progress every 1000 iterations
        if (iter % 100000 == 0 && params_.verbose){
            std::cout << "Iter: " << iter << " open size: " << open_.size() << std::endl;
        }
        auto state  = open_.min();
        open_.pop();
        state->setClosed();
        if (isGoalState(state->state_id)){
            goal_ = state->state_id;
            getTimeFromStart(stats_.time);
            reconstructPath(path, stats_.transition_costs);
            stats_.cost = state->g;
            stats_.path_length = (int)path.size();
            stats_.num_generated = (int)action_space_ptr_->states_.size();
            return true;
        }
        expand(state->state_id);
        ++iter;
    }
    getTimeFromStart(stats_.time);
    return false;
}

void ims::MultiPathwAStar::expand(int state_id){

  auto state_ = getSearchState(state_id);
  std::vector<int> successors;
  std::vector<double> costs;
  action_space_ptr_->getSuccessors(state_->heap_data.state_id, successors, costs);
  for (size_t i {0} ; i < successors.size() ; ++i){
    int successor_id = successors[i];
    double cost = costs[i];
    auto successor = getOrCreateSearchState(successor_id);
    assert(state_id>=0);
    successor->parent_ids_.insert(state_id); /// Keeping track of visitation by every state
    if (successor->heap_data.in_closed){
      continue;
    }
    if (isGoalState(successor_id) && params_.verbose ){
      std::cout << "Added Goal to open list" << std::endl;
    }
    if (successor->heap_data.in_open){
      if (successor->heap_data.g > state_->heap_data.g + cost){
        successor->heap_data.parent_id = state_->heap_data.state_id;
        successor->heap_data.g = state_->heap_data.g + cost;
        successor->heap_data.f = successor->heap_data.g + params_.epsilon*successor->heap_data.h;
        open_.update(&successor->heap_data);
      }
    } else {
      setStateVals(successor->heap_data.state_id, state_->heap_data.state_id, cost);
      open_.push(&successor->heap_data);
      successor->heap_data.setOpen();
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
    fval.at(id.first) += getSearchState(id.first)->heap_data.g;
  }

  return fval;
}


std::unordered_map<int, double> ims::MultiPathwAStar::reconstructFValue() {
  /// Construct the backward graph
  std::unordered_map<int, std::vector<int>> bwg;
  std::unordered_map<int, std::vector<double>> bwg_costs;
  for (auto s : states_) {
    SearchState* sid = getSearchState(s->heap_data.state_id);
    for (const auto bp : sid->parent_ids_) {
      bwg[s->heap_data.state_id].push_back(bp);
      double c = fabs(s->heap_data.g - getSearchState(bp)->heap_data.g);
      bwg_costs[s->heap_data.state_id].push_back(c);
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
  return getSearchState(state_id)->heap_data.g;
}

double ims::MultiPathwAStar::getHValue(int state_id) {
  return getSearchState(state_id)->heap_data.h;
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


void ims::MultiPathwAStar::setStateVals(int state_id, int parent_id, double cost)
{
    auto state_ = getSearchState(state_id);
    auto parent = getSearchState(parent_id);
    state_->heap_data.parent_id = parent_id;
    state_->heap_data.g = parent->heap_data.g + cost;
    state_->heap_data.h = computeHeuristic(state_id);
    state_->heap_data.f = state_->heap_data.g + params_.epsilon*state_->heap_data.h;
}


void ims::MultiPathwAStar::reconstructPath(std::vector<StateType>& path, std::vector<double>& costs) {
    path.clear();
    costs.clear();

    costs.push_back(0); // The goal state gets a transition cost of 0.
    SearchState* state_ = getSearchState(goal_);
    while (state_->heap_data.parent_id != -1){
        path.push_back(action_space_ptr_->getRobotState(state_->heap_data.state_id)->state);

        // Get the transition cost. This is the difference between the g values of the current state and its parent.
        double transition_cost = state_->heap_data.g - getSearchState(state_->heap_data.parent_id)->heap_data.g;
        costs.push_back(transition_cost);

        state_ = getSearchState(state_->heap_data.parent_id);
    }
    path.push_back(action_space_ptr_->getRobotState(state_->heap_data.state_id)->state);

    std::reverse(path.begin(), path.end());
    std::reverse(costs.begin(), costs.end());
}

void ims::MultiPathwAStar::reconstructPath(std::vector<StateType>& path) {
    SearchState* state_ = getSearchState(goal_);
    while (state_->heap_data.parent_id != -1){
        path.push_back(action_space_ptr_->getRobotState(state_->heap_data.state_id)->state);
        state_ = getSearchState(state_->heap_data.parent_id);
    }
    path.push_back(action_space_ptr_->getRobotState(state_->heap_data.state_id)->state);
    std::reverse(path.begin(), path.end());
}

void ims::MultiPathwAStar::resetPlanningData(){
    for (auto state_ : states_){
        delete state_;
    }
    states_.clear();
    open_.clear();
    goals_.clear();
    goal_ = -1;
    stats_ = PlannerStats();
}

auto ims::MultiPathwAStar::getAllSearchStates() -> std::vector<ims::MultiPathwAStar::SearchState*> {
    return states_;
}