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
 * \date   Jan 01 2024
 */

#include "search/planners/mgs.hpp"


ims::MGS::MGS(const MGSParams &params) : Planner(params), params_(params) {
    heuristic_ = params.heuristic_;
}

ims::MGS::~MGS() {
    std::cout << "MGS destructor" << std::endl;
    for (auto &state : states_) {
        // free the memory of the state
        for (int i {0}; i < params_.g_num_; ++i){
            state->data_[i].~HeapData();
        }
        free(state);
    }
    delete[] opens_;
}

void ims::MGS::initializePlanner(const std::shared_ptr<ActionSpaceMGS>& action_space_ptr,
                                 std::shared_ptr<std::vector<Controller>>& controllers,
                                 const StateType& start, const StateType& goal) {
    // Action space pointer.
    action_space_ptr_ = action_space_ptr;
    action_space_ptr_->resetPlanningData();
    resetPlanningData();
    // Set the controllers
    controllers_ = controllers;
    params_.g_num_ = 2;
    std::vector<ActionSequence> experiences;
    for (auto& controller : *controllers_) {
        std::vector<ActionSequence> local_mosaic = controller.solve();
        for (auto& action_seq : local_mosaic) {
            experiences.push_back(action_seq);
            params_.g_num_++;
        }
    }

    opens_ = new OpenList[params_.g_num_];
    max_h_states_.reserve(params_.g_num_);
    roots_.reserve(params_.g_num_);
    hl_graph_.reserve(params_.g_num_);
    for (int i {0}; i < params_.g_num_; ++i){
        hl_graph_[i] = std::vector<int>();
    }

    // Check if start is valid and add it to the action space.
    int start_ind_ = action_space_ptr_->getOrCreateRobotState(start);
    start_ = getOrCreateSearchState(start_ind_);
    start_->use_graph_ = GRAPH_START;

    int goal_ind_ = action_space_ptr_->getOrCreateRobotState(goal);
    auto goal_ = getOrCreateSearchState(goal_ind_);
    goal_->use_graph_ = GRAPH_GOAL;
    goals_.push_back(goal_ind_);

    // Evaluate the goal state
    goal_constraint_.type = SINGLE_SEARCH_STATE_GOAL;
    goal_constraint_.check_goal = &singleGoalConstraint;
    goal_constraint_.check_goal_user = &goal_->state_id;
    goal_constraint_.action_space_ptr = action_space_ptr_;

    // Evaluate the start state
    start_->data_[GRAPH_START].parent_id = std::make_shared<std::vector<int>>(1, PARENT_TYPE(START));
    heuristic_->setStart(start);
    // Evaluate the goal state
    goal_->data_[GRAPH_GOAL].parent_id = std::make_shared<std::vector<int>>(1, PARENT_TYPE(GOAL));

    heuristic_->setGoalConstraint(goal_constraint_);

    
    int graph_id = 2;
    std::vector<SearchState*> added_roots;
    std::vector<std::vector<SearchState*>> experiences_states;
    for (auto& experience : experiences) {
        int parent_id = UNSET;
        std::vector<SearchState*> experience_s;
        for (size_t i {0}; i < experience.size(); ++i) {
            int state_id = action_space_ptr_->getOrCreateRobotState(experience[i]);
            auto state = getOrCreateSearchState(state_id);
            state->data_[graph_id].parent_id->resize(1);
            state->data_[graph_id].parent_id->at(0) = parent_id;
            state->data_[graph_id].g = 0;
            state->data_[graph_id].h_self = 0;
            if (parent_id == UNSET){
                roots_[graph_id] = state;
            } else {
                state->data_[graph_id].edges->emplace_back(parent_id, 1);
                // add the edge in the opposite direction
                auto parent = getSearchState(parent_id);
                parent->data_[graph_id].edges->emplace_back(state_id, 1);
            }
            if (i == experience.size() - 1){
                max_h_states_[graph_id] = state;
            }
            parent_id = state_id;
            state->use_graph_ = graph_id;
            experience_s.emplace_back(state);
        }
        experiences_states.emplace_back(experience_s);
        graph_id++;
    }

    roots_.emplace(GRAPH_START, start_);
    roots_.emplace(GRAPH_GOAL, goal_);
    // initialize the max_h_states_ map
    max_h_states_.emplace(GRAPH_START, start_);
    max_h_states_.emplace(GRAPH_GOAL, goal_);

    // create two open lists one forward and one backward
    start_->data_[GRAPH_START].g = 0; start_->data_[GRAPH_START].h_self = 0;
    start_->data_[GRAPH_START].h = computeHeuristic(start_->state_id, (size_t)GRAPH_START);
    start_->data_[GRAPH_START].f = start_->data_[GRAPH_START].g + params_.w_ * start_->data_[GRAPH_START].h;
    start_->data_[GRAPH_START].setOpen();
    opens_[GRAPH_START].push(&start_->data_[GRAPH_START]);

    goal_->data_[GRAPH_GOAL].g = 0; goal_->data_[GRAPH_GOAL].h_self = 0;
    goal_->data_[GRAPH_GOAL].h = computeHeuristic(goal_->state_id, (size_t)GRAPH_GOAL);
    goal_->data_[GRAPH_GOAL].f = goal_->data_[GRAPH_GOAL].g + goal_->data_[GRAPH_GOAL].h;
    goal_->data_[GRAPH_GOAL].setOpen();
    opens_[GRAPH_GOAL].push(&goal_->data_[GRAPH_GOAL]);

    // reserve recently_expanded_ vector to the size of the number of graphs
    recently_expended_ = std::make_shared<std::unordered_map<int, std::vector<int>>>();
    for (int i {0}; i < params_.g_num_; ++i){
        recently_expended_->emplace(i, std::vector<int>());
    }

    for (int i {2}; i < graph_id; ++i){
        for (auto& state : experiences_states[i - 2]){
            state->data_[i].h = computeHeuristic(state->state_id, static_cast<size_t>(i));
            state->data_[i].f = state->data_[i].g + state->data_[i].h;
            state->data_[i].setOpen();
            opens_[i].push(&state->data_[i]);
        }
    }
}

auto ims::MGS::getSearchState(int state_id) -> ims::MGS::SearchState*{
    assert(state_id < states_.size() && state_id >= 0);
    return states_[state_id];
}

auto ims::MGS::getOrCreateSearchState(int state_id) -> ims::MGS::SearchState * {
    if (state_id >= states_.size()) {
        assert(state_id <= states_.size() && state_id >= 0);
        // overallocate the state for appropriate data
        size_t state_size =
            sizeof(SearchState) +
            sizeof(SearchState::HeapData) * (params_.g_num_);
        auto s = static_cast<SearchState*>(malloc(state_size));
        new (s) SearchState;
        for (int i = 0; i < params_.g_num_; ++i) {
            new (&s->data_[i]) SearchState::HeapData;
            s->data_[i].graph_id = i;
            s->data_[i].me = s;
            s->data_[i].edges = std::make_shared<std::vector<std::pair<int, double>>>();
            s->data_[i].parent_id = std::make_shared<std::vector<int>>();
        }
        s->state_id = state_id;
        s->h_map = std::make_shared<std::vector<double>>(params_.g_num_, INF_DOUBLE);
        states_.push_back(s);
    }
    return states_[state_id];
}

double ims::MGS::computeHeuristic(int state_id, size_t g_num) { // TODO: Something feels off here. Check this.
    computeHeuristics(state_id);
    auto ss = getSearchState(state_id);
    if (static_cast<int>(g_num) == GRAPH_START) { // GRAPH_GOAL (// testing heuristics
        return ss->h_map->at(goal_in_graph_);
    } else if (static_cast<int>(g_num) == goal_in_graph_) { // GRAPH_START
        return ss->h_map->at(GRAPH_START);
    }
    else {
        double h {0};
        double h_min {INF_DOUBLE};
        for (int i {0}; i < params_.g_num_; ++i){
            if (std::find(closed_graphs_.begin(), closed_graphs_.end(), i) != closed_graphs_.end()){
                continue;
            }
            else if ((i != GRAPH_START) && (i != goal_in_graph_))
                continue;

//            h += std::max(ss->h_map->at(i) - max_h_states_[i]->data_[i].h_self, 0.0);
            h += ss->h_map->at(i);
            if ((ss->h_map->at(i) < h_min) && i != g_num){
                h_min = ss->h_map->at(i);
//                h_min = std::max(ss->h_map->at(i) - max_h_states_[i]->data_[i].h_self, 0.0);
            }
        }
        h /= static_cast<double>(params_.g_num_ - static_cast<int>(closed_graphs_.size())); // TODO: should it be the minimum rather than the average?

//        // randomly return 0
//        std::random_device rd;
//        std::mt19937 gen(rd());
//        std::bernoulli_distribution dis0(0.5);
//        if (dis0(gen)){
//            return 0;
//        }
        // return h;
        return h_min;
    }
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
        ss->h_map->at(i) = dist;
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
            // if the graph is closed or empty, skip it
            if ((std::find(closed_graphs_.begin(), closed_graphs_.end(), i) != closed_graphs_.end()) ||
                (opens_[i].empty())){
                continue;
            }
            // report progress every 1000 iterations
            if (iter % 10000 == 0 && params_.verbose){
                std::cout << "Iter: " << iter << " open sizes: " <<  std::endl;
                for (int j {0}; j < params_.g_num_; ++j){
                    std::cout << "Open " << j << ": " << opens_[j].size() << "\n";
                }
            }
            if (isGoalConditionSatisfied()){
                getTimeFromStart(stats_.time);
                std::cout << "Goal found!" << std::endl;
                reconstructPath(path, stats_.transition_costs);
                stats_.cost = 0;
                for (auto& cost : stats_.transition_costs){
                    stats_.cost += cost;
                }
                stats_.path_length = static_cast<int>(path.size());
                stats_.num_generated = static_cast<int>(action_space_ptr_->states_.size());
                if (runAnchorGraphSearch(path)){
                    std::cout << GREEN << "Anchor graph search succeeded!" << RESET << std::endl;
                } else {
                    std::cout << RED << "Anchor graph search failed!" << RESET << std::endl;
                }
                stats_.suboptimality = params_.w_;
                saveData();
                return true;
            }
            auto state  = opens_[i].min();
            opens_[i].pop();
            // check if the state is in the closed list of the other open lists
            if (state->me->in_closed){
                if (state->me->graph_closed == i){
                    continue;
                }
                if (std::find(closed_graphs_.begin(), closed_graphs_.end(),
                              state->me->graph_closed) == closed_graphs_.end()) {
                    if (params_.verbose) {
                        std::cout << "Iter: " << iter << " open sizes: " <<  std::endl;
                        for (int j {0}; j < params_.g_num_; ++j){
                            std::cout << "Open " << j << ": " << opens_[j].size() << "\n";
                        }
                    }
                    // we need to connect the graphs!
                    connectGraphs(state->me->graph_closed, i,
                                  state->me->state_id);
                    if (params_.verbose) {
                        std::cout << "Iter: " << iter << " open sizes: " <<  std::endl;
                        for (int j {0}; j < params_.g_num_; ++j){
                            std::cout << "Open " << j << ": " << opens_[j].size() << "\n";
                        }
                    }
                    ++iter;
                    continue;
                }
            }
            state->setClosed();
            state->me->in_closed = true;
            state->me->graph_closed = i;
            expand(state->me->state_id, i);
            recently_expended_->at(i).push_back(state->me->state_id);
            //check if its parent is in recently_expanded_, and if so, remove it
            if (state->me->data_[i].parent_id->at(0) != START &&
                state->me->data_[i].parent_id->at(0) != GOAL &&
                state->me->data_[i].parent_id->at(0) != UNSET){
                const auto parent = getSearchState(state->me->data_[i].parent_id->at(0));
                auto it = std::find(recently_expended_->at(i).begin(),
                                    recently_expended_->at(i).end(),
                                    parent->state_id);
                if (it != recently_expended_->at(i).end()){
                    recently_expended_->at(i).erase(it);
                }
            }
            ////////////////////// CONNECT MANIPULATION WORKS NOT 2D ////////////////
            int curr_graph = i;
            // try to connect to other graphs
            for (int k{0} ; k < params_.g_num_ ; ++k) {
                if (k == i || std::find(closed_graphs_.begin(), closed_graphs_.end(), k) != closed_graphs_.end()) {
                    continue;
                }
                std::vector<int> path_id; std::vector<double> costs;
                bool connected = connect(curr_graph, k, state->me->state_id, path_id, costs);
                if (connected) {
                    std::cout << "Found connection: " << curr_graph << ", " << k << std::endl;
                    // check if the last state is in closed
                    double w = 1;
                    if (curr_graph == GRAPH_START) {
                        w = params_.w_;
                    }
                    auto curr_state = state->me;
                    for (int l {0}; l < path_id.size() ; ++l) {
                        int state_in_path = path_id[l];
                        // check if the last state is in closed
                        auto last_state = getOrCreateSearchState(state_in_path);
                        // update the state's values in the current graph
                        last_state->data_[curr_graph].parent_id->resize(1); // TODO: I assume it is a new state with no parents assigned
                        last_state->data_[curr_graph].parent_id->at(0) = curr_state->state_id;
                        last_state->data_[curr_graph].edges->emplace_back(curr_state->state_id, costs.at(l));
                        last_state->data_[curr_graph].g = curr_state->data_[curr_graph].g + costs.at(l);
                        last_state->data_[curr_graph].h_self = computeHeuristic(last_state->state_id,
                                                                                roots_[curr_graph]->state_id);
//                        if (last_state->data_[curr_graph].h_self > max_h_states_[curr_graph]->data_[curr_graph].h_self){
//                            max_h_states_[curr_graph] = last_state;
//                        }
                        last_state->data_[curr_graph].h = computeHeuristic(last_state->state_id, static_cast<size_t>(curr_graph));
                        last_state->data_[curr_graph].f = last_state->data_[curr_graph].g + w * last_state->data_[curr_graph].h;

                        ///////// check////////
                        if (l < path_id.size() - 1){
                            last_state->use_graph_ = curr_graph;
                            last_state->data_[curr_graph].setClosed();
                            last_state->graph_closed = curr_graph;
                        }
                        ///////////////////////

                        curr_state = last_state;
                    }
                    // for (int l {0}; l < path_id.size()-1 ; ++l) {
                    //     int state_in_path = path_id[l];
                    //     auto last_state = getOrCreateSearchState(state_in_path);
                    //     expand(last_state->state_id, curr_graph);
                    // }
                    // if (!curr_state->data_[k].is_closed) {
                    //     curr_state->data_[k].setClosed();
                    //     curr_state->in_closed = true;
                    //     curr_state->graph_closed = k;
                    //     expand(curr_state->state_id, k);
                    // }
                    // connect the graphs
                    int max_graph = connectGraphs(k, curr_graph, curr_state->state_id);
                    if (max_graph == curr_graph) {
                        curr_graph = k;
                    }
                }
            }
            ////////////////////// CONNECT MANIPULATION WORKS NOT 2D ////////////////
            ++iter;
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

    double w;
    if (g_num == GRAPH_START) {
        w = params_.w_;
    } else {
        w = 1;
    }
    for (size_t i {0} ; i < successors.size() ; ++i){
        int successor_id = successors[i];
        double cost = costs[i];
        auto successor = getOrCreateSearchState(successor_id);
        if (successor->data_[g_num].is_closed){
            continue;
        }
//        if (successor->data_[g_num].is_open){
        if (opens_[g_num].contains(&successor->data_[g_num])){ // really not efficient TODO: change this
            if (successor->data_[g_num].g > state->data_[g_num].g + cost){
                state->data_[g_num].edges->emplace_back(successor_id, cost);
                successor->data_[g_num].edges->emplace_back(state_id, cost);
                successor->use_graph_ = g_num;
                successor->data_[g_num].parent_id->resize(1); // TODO: I assume it is a new state with no parents assigned
                successor->data_[g_num].parent_id->at(0) = state_id;
                successor->data_[g_num].g = state->data_[g_num].g + cost;
                successor->data_[g_num].h = computeHeuristic(successor->state_id, static_cast<size_t>(g_num));
                successor->data_[g_num].f = successor->data_[g_num].g + w * successor->data_[g_num].h;
                opens_[g_num].update(&successor->data_[g_num]);
            }
        } else {
            setStateVals(successor->state_id,
                         state->state_id,
                         cost, g_num);
            state->data_[g_num].edges->emplace_back(successor->state_id, cost);
            successor->data_[g_num].edges->emplace_back(state->state_id, cost);
            opens_[g_num].push(&successor->data_[g_num]);
            successor->data_[g_num].setOpen();
        }
    }
    stats_.num_expanded++;
}

int ims::MGS::connectGraphs(int graph_id1, int graph_id2, int state_id){
    // Use the root of the graph that the heuristic value between its root and the start state is the lowest.
    // Then, combine both open lists into one, which is the open belongs to the graph with the lowest heuristic value.

    SearchState* root1 = roots_[graph_id1];
    SearchState* root2 = roots_[graph_id2];
    double h1 = computeHeuristic(start_->state_id, root1->state_id);
    double h2 = computeHeuristic(start_->state_id, root2->state_id);
    int min_h_graph_id = h1 < h2 ? graph_id1 : graph_id2;
    double w;
    if (min_h_graph_id == GRAPH_START) {
        w = params_.w_;
    } else {
        w = 1;
    }
    int max_h_graph_id = h1 < h2 ? graph_id2 : graph_id1;

    if (params_.verbose) {
        std::cout << "Connecting graphs (min, max): " << min_h_graph_id << ", " << max_h_graph_id << std::endl;
        std::cout << "Based on state: " << state_id << std::endl;
    }
    closed_graphs_.push_back(max_h_graph_id);
    // connect the graphs
    hl_graph_[min_h_graph_id].push_back(max_h_graph_id);
    hl_graph_[max_h_graph_id].push_back(min_h_graph_id);

    if (max_h_graph_id == goal_in_graph_) {
        goal_in_graph_ = min_h_graph_id;
    }

    // Update the values of the states that where expanded from the root of the graph with the higher heuristic value.
    // first, back propagate the values from the state to the root
    SearchState* state = getSearchState(state_id); // the state that was expanded
    // make sure that the values of the state are updated correctly,
    // based on the graph with the lower heuristic value
    if (state->graph_closed != min_h_graph_id) {
        // state->graph_closed = min_h_graph_id;
        state->data_[min_h_graph_id].h_self = computeHeuristic(state->state_id,
                                                               roots_[min_h_graph_id]->state_id);
        if (state->data_[min_h_graph_id].h_self > max_h_states_[min_h_graph_id]->data_[min_h_graph_id].h_self) {
            max_h_states_[min_h_graph_id] = state;
        }
        state->data_[min_h_graph_id].h = computeHeuristic(state->state_id, static_cast<size_t>(min_h_graph_id));
        state->data_[min_h_graph_id].f = state->data_[min_h_graph_id].g + w * state->data_[min_h_graph_id].h;
        state->data_[min_h_graph_id].setClosed();
    }
    // add all edges TODO: Is this necessary?
    for (auto& succ_id : *state->data_[max_h_graph_id].edges){
        if (std::find(state->data_[min_h_graph_id].edges->begin(),
                      state->data_[min_h_graph_id].edges->end(),
                      succ_id) == state->data_[min_h_graph_id].edges->end()){
            state->data_[min_h_graph_id].edges->emplace_back(succ_id);
        }
    }

    struct ConnectState {
        ConnectState(int state_id, SearchState::HeapData* state) : state_id(state_id), state(state), expanded(false) {}
        int state_id;
        SearchState::HeapData* state;
        bool expanded;
    };

    std::unordered_map<int, std::shared_ptr<ConnectState>> states_map;

    struct CompareConnectState {
        bool operator()(const std::shared_ptr<ConnectState>& a, const std::shared_ptr<ConnectState>& b) const {
            return a->state->g > b->state->g;
        }
    };

    std::priority_queue<std::shared_ptr<ConnectState>, std::vector<std::shared_ptr<ConnectState>>, CompareConnectState> open_list;
    auto state_init = std::make_shared<ConnectState>(state_id, &state->data_[min_h_graph_id]);
    open_list.push(state_init);
    states_map.emplace(state_id, state_init);

    while (!open_list.empty()) {
        auto curr_state = open_list.top();
        open_list.pop();
        curr_state->expanded = true;
        curr_state->state->me->use_graph_ = min_h_graph_id;
        // if (curr_state->state->me->in_closed) {
        //     curr_state->state->me->graph_closed = min_h_graph_id;
        // }
        for (auto &[fst, snd] : *curr_state->state->me->data_[max_h_graph_id].edges) {
            const auto succ = getSearchState(fst);
            // check if was already in the open list
            if (states_map.find(fst) != states_map.end()) {
                const auto& succ_connect_state = states_map.at(fst);
                if (succ_connect_state->expanded) {
                    continue;
                }
                if (succ_connect_state->state->g > curr_state->state->g + snd) {
                    succ_connect_state->state->parent_id->resize(1); // TODO: I assume it is a new state with no parents assigned
                    succ_connect_state->state->parent_id->at(0) = curr_state->state_id;
                    succ_connect_state->state->g = curr_state->state->g + snd;
                    succ_connect_state->state->h_self = computeHeuristic(succ_connect_state->state_id,
                                                                         roots_[min_h_graph_id]->state_id);
                    if (succ_connect_state->state->h_self
                        > max_h_states_[min_h_graph_id]->data_[min_h_graph_id].h_self) {
                        max_h_states_[min_h_graph_id] = succ_connect_state->state->me;
                    } // I think can be removed
                    succ_connect_state->state->h =
                        computeHeuristic(succ_connect_state->state_id, static_cast<size_t>(min_h_graph_id));
                    succ_connect_state->state->f = succ_connect_state->state->g + w * succ_connect_state->state->h;
                    // update the open list
                    open_list.push(succ_connect_state); // TODO: not efficient, can have duplicates
                }
            } else {
                auto state_from_max = &succ->data_[max_h_graph_id];
                auto state_from_min = &succ->data_[min_h_graph_id];
                if (state_from_min->is_closed) {
                    continue;
                }
                if (state_from_min->g > curr_state->state->g + snd) {
                    state_from_min->parent_id->resize(1); // TODO: I assume it is a new state with no parents assigned
                    state_from_min->parent_id->at(0) = curr_state->state_id;
                    state_from_min->g = curr_state->state->g + snd;
                    if (!state_from_min->is_open) {
                        state_from_min->setOpen();
                    } else {
                        // delete edges
                        state_from_min->edges->clear();
                    }
                }

                state_from_min->h_self = computeHeuristic(succ->state_id,
                                                          roots_[min_h_graph_id]->state_id);
                if (state_from_min->h_self > max_h_states_[min_h_graph_id]->data_[min_h_graph_id].h_self) {
                    max_h_states_[min_h_graph_id] = succ->data_[min_h_graph_id].me;
                }
                state_from_min->h = computeHeuristic(succ->state_id, static_cast<size_t>(min_h_graph_id));
                state_from_min->f = state_from_min->g + w * state_from_min->h;
                // add edges
                for (auto& edges : *state_from_max->edges){
                    if (std::find(state_from_min->edges->begin(),
                                  state_from_min->edges->end(),
                                  edges) == state_from_min->edges->end()){
                        state_from_min->edges->emplace_back(edges);
                    }
                }
                // do i need to set it to closed if it is in closed of max_h_graph_id?
                // if (state_from_max->is_closed){
                //     state_from_min->setClosed();
                // }
                auto succ_connect_state = std::make_shared<ConnectState>(succ->state_id, state_from_min);
                open_list.push(succ_connect_state);
                states_map.emplace(succ->state_id, succ_connect_state);
            }
        }
    }
    // Move all states from the open list of the graph with the higher heuristic value to the open list of the graph with the lower heuristic value.
    while (!opens_[max_h_graph_id].empty()){
        auto state_heap_max = opens_[max_h_graph_id].min();
        opens_[max_h_graph_id].pop();
        auto state_ = state_heap_max->me;
        auto state_heap_min = &state_->data_[min_h_graph_id];
        if (state_heap_min->is_closed && !state_heap_min->is_open){
            continue;
        } else if (state_heap_min->is_open && !state_heap_min->is_closed) {
            continue;
        }
        else {
            state_heap_min->setOpen();
            opens_[min_h_graph_id].push(state_heap_min);
        }
    }
    // update the roots
    roots_[max_h_graph_id] = roots_[min_h_graph_id];
    return max_h_graph_id;
}

void ims::MGS::setStateVals(int state_id, int parent_id, double cost, int g_num) {
    auto state_ = getSearchState(state_id);
    state_->use_graph_ = g_num;
    state_->data_[g_num].parent_id->resize(1); // TODO: I assume it is a new state with no parents assigned
    state_->data_[g_num].parent_id->at(0) = parent_id;
//    if (g_num == GRAPH_START || g_num == goal_in_graph_){
//        state_->data_[g_num].g = getSearchState(parent_id)->data_[g_num].g + cost;
//    } else {
//        state_->data_[g_num].g = 0;
//    }
    state_->data_[g_num].g = getSearchState(parent_id)->data_[g_num].g + cost;
    state_->data_[g_num].h_self = computeHeuristic(state_id, roots_[g_num]->state_id);
    if (state_->data_[g_num].h_self > max_h_states_[g_num]->data_[g_num].h_self){
        max_h_states_[g_num] = state_;
    }
    state_->data_[g_num].h = computeHeuristic(state_id, (size_t)g_num);
    double w {1};
    if (g_num == GRAPH_START) {
        w = params_.w_;
    }
    state_->data_[g_num].f = state_->data_[g_num].g + w * state_->data_[g_num].h;
}


void ims::MGS::reconstructPath(std::vector<StateType>& path) { // TODO: implement from scratch
    goal_ = goals_[0]; // TODO: fix this
    SearchState* state = getSearchState(goal_);
    while (state->data_[GRAPH_START].parent_id->at(0) != -1){
        if (state->data_[GRAPH_START].parent_id->size() > 1){
            for (int p_id : *state->data_[GRAPH_START].parent_id){
                path.push_back(action_space_ptr_->getRobotState(p_id)->state);
            }
        } else {
            path.push_back(action_space_ptr_->getRobotState(state->state_id)->state);
        }
        state = getSearchState(state->data_[GRAPH_START].parent_id->at(0));
    }
    path.push_back(action_space_ptr_->getRobotState(state->state_id)->state);
    std::reverse(path.begin(), path.end());
}


void ims::MGS::reconstructPath(std::vector<StateType>& path, std::vector<double>& costs) {
    path.clear();
    costs.clear();
    goal_ = goals_[0]; // TODO: fix this
    costs.push_back(0); // The goal state gets a transition cost of 0.
    SearchState* state_ = getSearchState(goal_);
    // if (state_->data_[GRAPH_START].parent_id->at(0) == -3){
    //     bool bp = true;
    // }
    while (state_->data_[GRAPH_START].parent_id->at(0) != -1){
        if ((std::find(path.begin(), path.end(), action_space_ptr_->getRobotState(state_->state_id)->state) != path.end()) &&
            (state_->data_[GRAPH_START].parent_id->at(0) != -1)){
            std::cout << "Loop in the path" << std::endl;
        }
        if (state_->data_[GRAPH_START].parent_id->size() > 1){
            for (int p_id : *state_->data_[GRAPH_START].parent_id){
                path.push_back(action_space_ptr_->getRobotState(p_id)->state);
            }
        } else {
            path.push_back(action_space_ptr_->getRobotState(state_->state_id)->state);
        }
        // Get the transition cost. This is the difference between the g values of the current state and its parent.
        double transition_cost = state_->data_[GRAPH_START].g -
            getSearchState(state_->data_[GRAPH_START].parent_id->at(0))->data_[GRAPH_START].g;
        costs.push_back(transition_cost);
        state_ = getSearchState(state_->data_[GRAPH_START].parent_id->at(0));
    }
    path.push_back(action_space_ptr_->getRobotState(state_->state_id)->state);

    std::reverse(path.begin(), path.end());
    std::reverse(costs.begin(), costs.end());
}


bool ims::MGS::isGoalState(int s_id) {
    if (std::any_of(goals_.begin(), goals_.end(), [&s_id](int goal_ind) {return s_id == goal_ind;})){
        return true;
    }
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
    if (visited[1]) {
        // save the graph_hl_
        std::ofstream hl_graph_file;
        hl_graph_file.open("hl_graph_mgs.txt");
        // delete the file if it exists
        if (hl_graph_file.is_open()){
            hl_graph_file.close();
            std::remove("hl_graph_mgs.txt");
        }
        hl_graph_file.open("hl_graph_mgs.txt");
        for (int ind {0} ; ind < params_.g_num_ ; ++ind){
            for (int ind_2 {0}; ind_2 < params_.g_num_; ++ind_2){
                if (ind_2 != 0)
                    hl_graph_file << ",";
                if (std::find(hl_graph_[ind].begin(), hl_graph_[ind].end(), ind_2) != hl_graph_[ind].end()){
                    hl_graph_file << "1";
                } else {
                    hl_graph_file << "0";
                }
            }
            hl_graph_file << "\n";
        }
        hl_graph_file.close();
        std::cout << "hl_graph_mgs.txt saved" << std::endl;
    }
    return visited[1];
    // TODO: Do i need all of that? isn;t it enough to check the forward and backward graphs? (i.e., 0 and 1)
}


void ims::MGS::resetPlanningData() {
    for (auto state : states_){
        delete state;
    }
    states_ = std::vector<SearchState*>();
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


bool ims::MGS::runAnchorGraphSearch(std::vector<StateType>& path) {
    // time the search
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    // add goal state to the open list
    auto goal_state = getSearchState(goal_);
    opens_[GRAPH_START].push(&goal_state->data_[GRAPH_START]);
    int iter {0};
    while (!opens_[GRAPH_START].empty() && !isTimeOut()) {
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        // if (elapsed_seconds.count() > 5.0){
        //     return false;
        // }
        // report progress every 1000 iterations
        if (iter % 100000 == 0 && params_.verbose){
            std::cout << "Anchor search Iter: " << iter << " open size: " << opens_[GRAPH_START].size() << std::endl;
        }
        auto state = opens_[GRAPH_START].min();
        opens_[GRAPH_START].pop();
        if (state->me->in_closed && state->me->graph_closed == GRAPH_START){
            continue;
        }
        state->setClosed();
        state->me->graph_closed = GRAPH_START;

        if (isGoalState(state->me->state_id)) {
            getTimeFromStart(stats_.time);
            std::cout << "Goal found!" << std::endl;
            reconstructPath(path, stats_.transition_costs);
            stats_.cost = 0;
            for (auto& cost : stats_.transition_costs){
                stats_.cost += cost;
            }
            stats_.path_length = static_cast<int>(path.size());
            stats_.num_generated = static_cast<int>(action_space_ptr_->states_.size());
            return true;
        }
        std::vector<int> successors;
        std::vector<double> costs;
        action_space_ptr_->getSuccessors(state->me->state_id,
                                         successors, costs);
        for (size_t i {0} ; i < successors.size() ; ++i){
            int successor_id = successors[i];
            double cost = costs[i];
            auto successor = getOrCreateSearchState(successor_id);
            if (successor->data_[GRAPH_START].is_closed && successor->graph_closed == GRAPH_START){
                continue;
            }
            if (opens_[GRAPH_START].contains(&successor->data_[GRAPH_START])) {
                if (successor->data_[GRAPH_START].g > state->g + cost) {
                    state->edges->emplace_back(successor_id, cost);
                    successor->data_[GRAPH_START].edges->emplace_back(state->me->state_id, cost);
                    successor->data_[GRAPH_START].parent_id->resize(1); // TODO: I assume it is a new state with no parents assigned
                    successor->data_[GRAPH_START].parent_id->at(0) = state->me->state_id;
                    successor->data_[GRAPH_START].g = state->g + cost;
                    // successor->data_[GRAPH_START].h_self = computeHeuristic(successor->state_id,
                    //                                                        roots_[GRAPH_START]->state_id);
                    // successor->data_[GRAPH_START].h = computeHeuristic(successor->state_id, static_cast<size_t>(GRAPH_START));
                    successor->data_[GRAPH_START].f = successor->data_[GRAPH_START].g + params_.w_ * successor->data_[GRAPH_START].h;
                    opens_[GRAPH_START].update(&successor->data_[GRAPH_START]);
                }
            } else {
                setStateVals(successor->state_id,
                             state->me->state_id,
                             cost, GRAPH_START);
                state->edges->emplace_back(successor->state_id, cost);
                successor->data_[GRAPH_START].edges->emplace_back(state->me->state_id, cost);
                opens_[GRAPH_START].push(&successor->data_[GRAPH_START]);
                successor->data_[GRAPH_START].setOpen();
            }
        }
        stats_.num_expanded++;
        ++iter;
    }
    getTimeFromStart(stats_.time);
    return false;
}

void ims::MGS::saveData() const {
    // save all states to a file
    std::ofstream states_file;
    states_file.open("states_mgs.txt");
    // delete the file if it exists
    if (states_file.is_open()){
        states_file.close();
        std::remove("states_mgs.txt");
    }
    states_file.open("states_mgs.txt");
    for (auto& state : states_) {
        states_file << state->state_id;
        auto state_val = action_space_ptr_->getRobotState(state->state_id)->state;
        for (auto& val : state_val) {
            states_file << "," << val;
        }
        states_file << "\n";
    }
}


bool ims::MGS::connect(int graph_curr, int graph_other,
                       int state_id, std::vector<int> &connecting_path,
                       std::vector<double>& costs) {
    // find the nearest state in the other graph
    double min_dist = INF_DOUBLE;
    int min_state_id = -1;
    for (auto& s : states_){ // TODO: this is not efficient, it loops over all states and not only the states in the other graph
    // for (const int recent_expanded : recently_expended_->at(graph_other)){
        // if (const auto s = getSearchState(recent_expanded); s->data_[graph_other].g != INF_DOUBLE){
        if (s->data_[graph_other].g != INF_DOUBLE){
            double dist = computeHeuristic(state_id, s->state_id);
            if (dist < min_dist){
                min_dist = dist;
                min_state_id = s->state_id;
            }
        }
    }
    if (min_state_id == -1){
        return false;
    }
    // find the path between the two states
    return action_space_ptr_->connectStates(state_id, min_state_id, connecting_path, costs);
}

