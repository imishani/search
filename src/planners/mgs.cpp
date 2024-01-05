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
 * \file   mgs.cpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   Oct 05 2023
 */

#include "search/planners/mgs.hpp"
#include <queue>

ims::MGS::MGS(const MGSParams &params) : Planner(params), params_(params) {
    heuristic_ = params.heuristic_;
//    opens_ = new OpenList;
//    opens_.reserve(params.g_num_);
    max_h_states_.reserve(params.g_num_);
    roots_.reserve(params.g_num_);
    hl_graph_.reserve(params.g_num_);
    for (int i {0}; i < params.g_num_; ++i){
//        opens_[i] = OpenList {};
        hl_graph_[i] = std::vector<int>();
    }
}

ims::MGS::~MGS() {
    for (auto &state : states_) {
        delete state;
    }
//    delete[] opens_;
}

void ims::MGS::initializePlanner(const std::shared_ptr<ActionSpaceMGS>& action_space_ptr,
                                 const StateType& start, const StateType& goal, int g_num) {
    // Action space pointer.
    action_space_ptr_ = action_space_ptr;

    action_space_ptr_->resetPlanningData();
    resetPlanningData();

    // Check if start is valid and add it to the action space.
    int start_ind_ = action_space_ptr_->getOrCreateRobotState(start);
    start_ = getOrCreateSearchState(start_ind_);
    start_->use_graph_ = GRAPH_START;

    int goal_ind_ = action_space_ptr_->getOrCreateRobotState(goal);
    auto goal_ = getOrCreateSearchState(goal_ind_);
    goal_->use_graph_ = GRAPH_GOAL;
    goals_.push_back(goal_ind_);

    // Evaluate the start state
    start_->parent_id.at(GRAPH_START) = PARENT_TYPE(START);
    heuristic_->setStart(const_cast<StateType &>(start));
    // Evaluate the goal state
    goal_->parent_id.at(GRAPH_GOAL) = PARENT_TYPE(GOAL);
    heuristic_->setGoal(const_cast<StateType &>(goal));

    // generate params_.g_num_ - 2 random states
    std::vector<SearchState*> random_states;
    for (int i {0}; i < params_.g_num_ - 2; ++i){
        int rand_ind {-1};
        do {rand_ind = generateRandomState(start, goal);}
        while (rand_ind == -1 || std::any_of(random_states.begin(),
                                             random_states.end(),
                                             [&rand_ind](SearchState* s){
                                                 return s->state_id == rand_ind;}));
        auto rand_state = getOrCreateSearchState(rand_ind);
        rand_state->parent_id.at(i + 2) = PARENT_TYPE(UNSET);
        rand_state->g.at(i + 2) = 0; rand_state->h_self.at(i + 2) = 0;
        max_h_states_.emplace(i + 2, rand_state);
        roots_.emplace(i + 2, rand_state);
        random_states.push_back(rand_state);
    }


    roots_.emplace(GRAPH_START, start_);
    roots_.emplace(GRAPH_GOAL, goal_);
    // initialize the max_h_states_ map
    max_h_states_.emplace(GRAPH_START, start_);
    max_h_states_.emplace(GRAPH_GOAL, goal_);

    // create two open lists one forward and one backward
    start_->g.at(GRAPH_START) = 0; start_->h_self.at(GRAPH_START) = 0;
    start_->h.at(GRAPH_START) = computeHeuristic(start_->state_id);
    start_->f.at(GRAPH_START) = start_->g.at(GRAPH_START) + start_->h.at(GRAPH_START);
    opens_[GRAPH_START].push(start_);
    start_->setOpen(GRAPH_START);
    goal_->g.at(GRAPH_GOAL) = 0; goal_->h_self.at(GRAPH_GOAL) = 0;
    goal_->h.at(GRAPH_GOAL) = computeHeuristic(goal_->state_id);
    goal_->f.at(GRAPH_GOAL) = goal_->g.at(GRAPH_GOAL) + goal_->h.at(GRAPH_GOAL);
    opens_[GRAPH_GOAL].push(goal_);
    goal_->setOpen(GRAPH_GOAL);


    // initialize the open lists
    for (int i {2}; i < params_.g_num_; ++i){
        random_states[i - 2]->h.at(i) = computeHeuristic(random_states[i - 2]->state_id);
        random_states[i - 2]->f.at(i) = random_states[i - 2]->g.at(i) + random_states[i - 2]->h.at(i);
        opens_[i].push(random_states[i - 2]);
        random_states[i - 2]->setOpen(i);
    }
}

auto ims::MGS::getSearchState(int state_id) -> ims::MGS::SearchState*{
    assert(state_id < states_.size() && state_id >= 0);
    return states_[state_id];
}

auto ims::MGS::getOrCreateSearchState(int state_id) -> ims::MGS::SearchState * {
    if (state_id >= states_.size()){
        states_.resize(state_id + 1, nullptr);
    }
    if (states_[state_id] == nullptr){
        assert(state_id < states_.size() && state_id >= 0);
        states_[state_id] = new SearchState(params_.g_num_);
        states_[state_id]->state_id = state_id;
    }
    return states_[state_id];
}

double ims::MGS::computeHeuristic(int state_id) { // TODO: Something feels off here. Check this.
    computeHeuristics(state_id);
    auto ss = getSearchState(state_id);
    double h {0};
    for (int i {0}; i < params_.g_num_; ++i){
        if (std::find(closed_graphs_.begin(), closed_graphs_.end(), i) != closed_graphs_.end()){
            continue;
        }
        h += std::max(ss->h_map[i] - max_h_states_[i]->h_self.at(i), 0.0);
    }
    h /= (params_.g_num_ - (int)closed_graphs_.size()); // TODO: should it be the minimum rather than the average?
    return h;
}

double ims::MGS::computeHeuristic(int s1_id, int s2_id) {
    double dist;
    auto s1 = action_space_ptr_->getRobotState(s1_id);
    auto s2 = action_space_ptr_->getRobotState(s2_id);
    if (!heuristic_->getHeuristic(s1->state, s2->state, dist))
        throw std::runtime_error("Heuristic function failed");
    else
        return dist;
}

void ims::MGS::computeHeuristics(int s_id) {
    auto ss = getSearchState(s_id);
    for (int i {0}; i < params_.g_num_; ++i){
        double dist = computeHeuristic(s_id, roots_[i]->state_id);
        ss->h_map[i] = dist;
    }
}


bool ims::MGS::plan(std::vector<StateType>& path) {
    startTimer();
    int iter {0};
    // while all open lists are not empty
    bool opens_empty{true};
    for (int i {0}; i < params_.g_num_; ++i){
        if (!opens_[i].empty()){
            opens_empty = false;
            break;
        }
    }
    while (!opens_empty && !isTimeOut()){
        // We start by expanding sequentially 1 state from each open list. TODO: make this synchronous in parallel.
        for (int i {0}; i < params_.g_num_; ++i){
            // if the graph is closed, skip it
            if (std::find(closed_graphs_.begin(), closed_graphs_.end(), i) != closed_graphs_.end()){
                continue;
            }
            if (!opens_[i].empty()){
                // report progress every 1000 iterations
                if (iter % 100000 == 0 && params_.verbose){
                    std::cout << "Iter: " << iter << " open size: " << opens_[i].size() << std::endl;
                }
                if (isGoalConditionSatisfied()){
                    getTimeFromStart(stats_.time);
                    reconstructPath(path, stats_.transition_costs);
                    stats_.cost = 0; // TODO: fix this
                    stats_.path_length = (int)path.size();
                    stats_.num_generated = (int)action_space_ptr_->states_.size();
                    return true;
                }
                auto state  = opens_[i].min();
                opens_[i].pop();
                // check if the state is in the closed list of the other open lists
                if (state->in_closed){
                    if (state->graph_closed == i){
                        continue;
                    }
                    else {
                        std::cout << "Iter: " << iter << " open size: " << opens_[i].size() << std::endl;
                        // we need to connect the graphs!
                        connectGraphs(state->graph_closed, i, state->state_id);
                        ++iter;
                        continue;
                    }
                }
                state->setClosed(i);
                expand(state->state_id, i);
                ++iter;
            }
        }
        opens_empty = true;
        for (int i {0}; i < params_.g_num_; ++i){
            if (!opens_[i].empty()){
                opens_empty = false;
                break;
            }
        }
    }

    getTimeFromStart(stats_.time);
    return false;
}

void ims::MGS::expand(int state_id, int g_num){
    auto state = getSearchState(state_id);
    state->use_graph_ = g_num;
    std::vector<int> successors;
    std::vector<double> costs;
    action_space_ptr_->getSuccessors(state->state_id,
                                     successors, costs);

    for (size_t i {0} ; i < successors.size() ; ++i){
        int successor_id = successors[i];
        double cost = costs[i];
        auto successor = getOrCreateSearchState(successor_id);
        if (successor->in_closed_map.at(g_num)){
            continue;
        }
        if (successor->in_open_map.at(g_num)){ // TODO: check if this is the correct condition, since the states are shared between all open lists
            if (successor->f.at(g_num) > state->g.at(g_num) + cost){
                state->edges.at(g_num).emplace_back(successor_id, cost);
                successor->edges.at(g_num).emplace_back(state_id, cost);
                successor->use_graph_ = g_num;
                successor->parent_id.at(g_num) = state->state_id;
                successor->g.at(g_num) = state->g.at(g_num) + cost;
                successor->f.at(g_num) = successor->g.at(g_num) + computeHeuristic(successor->state_id);
//                opens_.at(g_num).update(successor); // TODO: There is a hack here. In the compare we use the graph number from this state alone!
                opens_[g_num].update(successor); // TODO: There is a hack here. In the compare we use the graph number from this state alone!
            }
        } else {
            setStateVals(successor->state_id,
                         state->state_id,
                         cost, g_num);
            state->edges.at(g_num).emplace_back(successor_id, cost);
            successor->edges.at(g_num).emplace_back(state_id, cost);
//            opens_.at(g_num).push(successor); // the use_graph_ is set in the setStateVals function
            opens_[g_num].push(successor);
            successor->setOpen(g_num);
        }
    }
    stats_.num_expanded++;
}

void ims::MGS::connectGraphs(int graph_id1, int graph_id2, int state_id){
    // Use the root of the graph that the heuristic value between its root and the start state is the lowest.
    // Then, combine both open lists into one, which is the open belongs to the graph with the lowest heuristic value.
    SearchState* root1 = roots_[graph_id1];
    SearchState* root2 = roots_[graph_id2];
    double h1 = computeHeuristic(start_->state_id, root1->state_id);
    double h2 = computeHeuristic(start_->state_id, root2->state_id);
    int min_h_graph_id = h1 < h2 ? graph_id1 : graph_id2;
    int max_h_graph_id = h1 < h2 ? graph_id2 : graph_id1;
    closed_graphs_.push_back(max_h_graph_id);
    // connect the graphs
    hl_graph_[min_h_graph_id].push_back(max_h_graph_id);
    hl_graph_[max_h_graph_id].push_back(min_h_graph_id);
    // Update the values of the states that where expanded from the root of the graph with the higher heuristic value.
    // first, back propagate the values from the state to the root
    SearchState* state = getSearchState(state_id); // the state that was expanded
    // make sure that the values of the state are updated correctly,
    // based on the graph with the lower heuristic value
    if (state->graph_closed == max_h_graph_id) {
        state->graph_closed = min_h_graph_id;
        state->h_self.at(min_h_graph_id) = computeHeuristic(state->state_id, roots_[min_h_graph_id]->state_id);
        if (state->h_self.at(min_h_graph_id) > max_h_states_[min_h_graph_id]->h_self.at(min_h_graph_id)) {
            max_h_states_[min_h_graph_id] = state;
        }
        state->h.at(min_h_graph_id) = computeHeuristic(state->state_id);
        state->f.at(min_h_graph_id) = state->g.at(min_h_graph_id) + state->h.at(min_h_graph_id);
    }
    std::queue<SearchState*> local_open_list;
    local_open_list.push(state);
    std::vector<int> in_open_list;
    in_open_list.push_back(state->state_id);
    while (!local_open_list.empty()) {
        // FIFO
        std::cout << "local open list size: " << local_open_list.size() << std::endl;
        SearchState* curr_state = local_open_list.front();
        local_open_list.pop();
        curr_state->use_graph_ = min_h_graph_id;
        if (curr_state->in_closed) {
            curr_state->graph_closed = min_h_graph_id;
        }
        curr_state->in_closed_map.at(min_h_graph_id) = true;
        // look at all the successors of the state based on the edges in the graph
        for (auto& succ_id : curr_state->edges.at(max_h_graph_id)){
            auto succ = getSearchState(succ_id.first);
            if (succ->in_closed_map.at(min_h_graph_id)){
                continue;
            }
            // check if was already in the open list
            if (std::find(in_open_list.begin(), in_open_list.end(), succ->state_id) != in_open_list.end()){
                continue;
            }
            succ->parent_id.at(min_h_graph_id) = curr_state->state_id;
            succ->g.at(min_h_graph_id) = curr_state->g.at(min_h_graph_id) + succ_id.second;
            succ->h.at(min_h_graph_id) = computeHeuristic(succ->state_id);
            succ->f.at(min_h_graph_id) = succ->g.at(min_h_graph_id) + succ->h.at(min_h_graph_id);
            local_open_list.push(succ);
            in_open_list.push_back(succ->state_id);

//            if (succ->in_open_map.at(min_h_graph_id) && (succ->use_graph_ == min_h_graph_id)){
//                if (succ->f.at(min_h_graph_id) > curr_state->g.at(min_h_graph_id) + succ_id.second){
//                    succ->parent_id.at(min_h_graph_id) = curr_state->state_id;
//                    succ->g.at(min_h_graph_id) = curr_state->g.at(min_h_graph_id) + succ_id.second;
//                    succ->h.at(min_h_graph_id) = computeHeuristic(succ->state_id);
//                    succ->f.at(min_h_graph_id) = succ->g.at(min_h_graph_id) + succ->h.at(min_h_graph_id);
//                    local_open_list.push(succ);
//                }
//            } else {
//                setStateVals(succ->state_id,
//                             curr_state->state_id,
//                             succ_id.second, min_h_graph_id);
//                local_open_list.push(succ);
//                succ->setOpen(min_h_graph_id);
//            }
        }
    }
    // Move all states from the open list of the graph with the higher heuristic value to the open list of the graph with the lower heuristic value.
    while (!opens_[max_h_graph_id].empty()){
        auto state_ = opens_[max_h_graph_id].min();
        opens_[max_h_graph_id].pop();
        state_->setOpen(min_h_graph_id);
        // assert that the state was updated in previous steps
//        if (std::find(in_open_list.begin(), in_open_list.end(), state_->state_id) == in_open_list.end()){
//            continue;
//        }
//        assert(state_->use_graph_ == min_h_graph_id); // TODO: Check what is going on here
        opens_[min_h_graph_id].push(state_);
    }
    // update the roots
    roots_[max_h_graph_id] = roots_[min_h_graph_id];

}

void ims::MGS::setStateVals(int state_id, int parent_id, double cost, int g_num) {
    auto state_ = getSearchState(state_id);
    state_->use_graph_ = g_num;
    state_->parent_id.at(g_num) = parent_id;
    state_->g.at(g_num) = getSearchState(parent_id)->g.at(g_num) + cost;
    state_->h_self.at(g_num) = computeHeuristic(state_id, roots_[g_num]->state_id);
    if (state_->h_self.at(g_num) > max_h_states_[g_num]->h_self.at(g_num)){
        max_h_states_[g_num] = state_;
    }
    state_->h.at(g_num) = computeHeuristic(state_id);
    state_->f.at(g_num) = state_->g.at(g_num) + state_->h.at(g_num);
}


void ims::MGS::reconstructPath(std::vector<StateType>& path) { // TODO: implement from scratch
    SearchState* state = getSearchState(goal_);
    while (state->parent_id.at(0) != -1){
        path.push_back(action_space_ptr_->getRobotState(state->state_id)->state);
        state = getSearchState(state->parent_id.at(0));
    }
    path.push_back(action_space_ptr_->getRobotState(state->state_id)->state);
    std::reverse(path.begin(), path.end());
}


void ims::MGS::reconstructPath(std::vector<StateType>& path, std::vector<double>& costs) {
    path.clear();
    costs.clear();

    costs.push_back(0); // The goal state gets a transition cost of 0.
    SearchState* state_ = getSearchState(goal_);
    while (state_->parent_id.at(0) != -1){
        path.push_back(action_space_ptr_->getRobotState(state_->state_id)->state);

        // Get the transition cost. This is the difference between the g values of the current state and its parent.
        double transition_cost = state_->g.at(0) - getSearchState(state_->parent_id.at(0))->g.at(0);
        costs.push_back(transition_cost);

        state_ = getSearchState(state_->parent_id.at(0));
    }
    path.push_back(action_space_ptr_->getRobotState(state_->state_id)->state);

    std::reverse(path.begin(), path.end());
    std::reverse(costs.begin(), costs.end());
}


bool ims::MGS::isGoalState(int s_id) {
    if (std::any_of(goals_.begin(), goals_.end(), [&s_id](int goal_ind) {return s_id == goal_ind;})){
        return true;
    }

    // Also ask the action space if this state id is a goal state. Sometimes, states need to be determined as goal in real time.
    // This is EXTREMELY bug prone. Commenting this out for now until we have a proper GoalCondition object.
    // return action_space_ptr_->isGoalState(s_id, goals_);
    return false;
}


bool ims::MGS::isGoalConditionSatisfied() {
    // check if the hl_graph_ is connected from start (vertex 0) to goal (vertex 1)
    std::vector<bool> visited(params_.g_num_, false);
    std::queue<int> q;
    q.push(0);
    visited[0] = true;
    while (!q.empty()){
        int curr = q.front();
        q.pop();
        for (auto& v : hl_graph_[curr]){
            if (!visited[v]){
                visited[v] = true;
                q.push(v);
            }
        }
    }
    if (visited[1]){
        return true;
    }
    return visited[1];
}


void ims::MGS::resetPlanningData() {
    for (auto state : states_){
        delete state;
    }
    states_ = std::vector<SearchState*>();
//    for (auto &open : opens_){
//        open.second.clear();
//    }
    for (int i {0}; i < params_.g_num_; ++i){
        opens_[i].clear();
    }
    max_h_states_.clear(); max_h_states_.reserve(params_.g_num_);
    roots_.clear(); roots_.reserve(params_.g_num_);
    hl_graph_.clear(); hl_graph_.reserve(params_.g_num_);
    for (int i {0}; i < params_.g_num_; ++i){
        hl_graph_[i] = std::vector<int>();
    }
    goals_.clear();
    goal_ = -1;
    stats_ = MGSPlannerStats();
}

int ims::MGS::generateRandomState(const StateType& start, const StateType& goal) {
    // generate a random state within the ellipsoid defined based on the heuristic between start and goal
    double dist;
    if (!heuristic_->getHeuristic(start, goal, dist)) {
        std::cout << RED << "Heuristic function failed" << RESET << std::endl;
        return -1;
    } else {
        StateType random_state;
        if (!action_space_ptr_->generateRandomState(start, goal, dist, random_state)) {
            std::cout << RED << "Failed to generate random state" << RESET << std::endl;
            return -1;
        } else {
            int rand_ind = action_space_ptr_->getOrCreateRobotState(random_state);
            return rand_ind;
        }
    }
}

