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
#include <fstream>

ims::MGS::MGS(const MGSParams &params) : Planner(params), params_(params) {
    heuristic_ = params.heuristic_;
    opens_ = new OpenList[params.g_num_];
    max_h_states_.reserve(params.g_num_);
    roots_.reserve(params.g_num_);
    hl_graph_.reserve(params.g_num_);
    for (int i {0}; i < params.g_num_; ++i){
        hl_graph_[i] = std::vector<int>();
    }
}

ims::MGS::~MGS() {
    std::cout << "MGS destructor" << std::endl;
    for (auto &state : states_) {
        // free the memory of the state
        for (int i {0}; i < params_.g_num_; ++i){
            state->data_[i].~HeapData();
        }
        free(state);
//        delete state;
    }
    delete[] opens_;
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
    start_->data_[GRAPH_START].parent_id = PARENT_TYPE(START);
    heuristic_->setStart(const_cast<StateType &>(start));
    // Evaluate the goal state
    goal_->data_[GRAPH_GOAL].parent_id = PARENT_TYPE(GOAL);
    heuristic_->setGoal(const_cast<StateType &>(goal));

    // generate params_.g_num_ - 2 random states
    std::vector<SearchState*> random_states;
    for (int i {2}; i < params_.g_num_; ++i){
        int rand_ind {-1};
        do {rand_ind = generateRandomState(start, goal);}
        while (rand_ind == -1 || std::any_of(random_states.begin(),
                                             random_states.end(),
                                             [&rand_ind](SearchState* s){
                                                 return s->state_id == rand_ind;}) ||
                                                 rand_ind == start_ind_ || rand_ind == goal_ind_);
        auto rand_state = getOrCreateSearchState(rand_ind);
        rand_state->data_[i].parent_id = PARENT_TYPE(UNSET);
        rand_state->data_[i].g = 0; rand_state->data_[i].h_self = 0;
        max_h_states_.emplace(i, rand_state);
        roots_.emplace(i, rand_state);
        random_states.push_back(rand_state);
        stats_.root_states.push_back(action_space_ptr_->getRobotState(rand_ind)->state);
    }

    roots_.emplace(GRAPH_START, start_);
    roots_.emplace(GRAPH_GOAL, goal_);
    // initialize the max_h_states_ map
    max_h_states_.emplace(GRAPH_START, start_);
    max_h_states_.emplace(GRAPH_GOAL, goal_);

    // create two open lists one forward and one backward
    start_->data_[GRAPH_START].g = 0; start_->data_[GRAPH_START].h_self = 0;
    start_->data_[GRAPH_START].h = computeHeuristic(start_->state_id, (size_t)GRAPH_START);
    start_->data_[GRAPH_START].f = start_->data_[GRAPH_START].g + start_->data_[GRAPH_START].h;
    start_->data_[GRAPH_START].setOpen();
    opens_[GRAPH_START].push(&start_->data_[GRAPH_START]);

    goal_->data_[GRAPH_GOAL].g = 0; goal_->data_[GRAPH_GOAL].h_self = 0;
    goal_->data_[GRAPH_GOAL].h = computeHeuristic(goal_->state_id, (size_t)GRAPH_GOAL);
    goal_->data_[GRAPH_GOAL].f = goal_->data_[GRAPH_GOAL].g + goal_->data_[GRAPH_GOAL].h;
    goal_->data_[GRAPH_GOAL].setOpen();
    opens_[GRAPH_GOAL].push(&goal_->data_[GRAPH_GOAL]);

    // initialize the open lists
    for (int i {2}; i < params_.g_num_; ++i){
        random_states[i - 2]->data_[i].h = computeHeuristic(random_states[i - 2]->state_id, (size_t)i);
        random_states[i - 2]->data_[i].f = random_states[i - 2]->data_[i].g + random_states[i - 2]->data_[i].h;
        random_states[i - 2]->data_->setOpen();
        opens_[i].push(&random_states[i - 2]->data_[i]);
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
    if ((int)g_num == GRAPH_START) { // GRAPH_GOAL (// testing heuristics
        return ss->h_map->at(goal_in_graph_);
    } else if ((int)g_num == goal_in_graph_) { // GRAPH_START
        return ss->h_map->at(GRAPH_START);
    }
    else {
        double h {0};
        double h_min {INF_DOUBLE};
        for (int i {0}; i < params_.g_num_; ++i){
            if (std::find(closed_graphs_.begin(), closed_graphs_.end(), i) != closed_graphs_.end()){
                continue;
            }
            h += std::max(ss->h_map->at(i) - max_h_states_[i]->data_[i].h_self, 0.0);
            if ((ss->h_map->at(i) < h_min) && i != g_num){
                h_min = ss->h_map->at(i);
            }
        }
        h /= (params_.g_num_ - (int)closed_graphs_.size()); // TODO: should it be the minimum rather than the average?
//        return h;
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
                stats_.path_length = (int)path.size();
                stats_.num_generated = (int)action_space_ptr_->states_.size();
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
                else if (std::find(closed_graphs_.begin(), closed_graphs_.end(),
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

    for (size_t i {0} ; i < successors.size() ; ++i){
        int successor_id = successors[i];
        double cost = costs[i];
        auto successor = getOrCreateSearchState(successor_id);
        if (successor->data_[g_num].is_closed){
            continue;
        }
//        if (successor->data_[g_num].is_open){
        if (opens_[g_num].contains(&successor->data_[g_num])){ // really not efficient TODO: change this
            if (successor->data_[g_num].f > state->data_[g_num].g + cost){
                state->data_[g_num].edges->emplace_back(successor_id, cost);
                successor->data_[g_num].edges->emplace_back(state_id, cost);
                successor->use_graph_ = g_num;
                successor->data_[g_num].parent_id = state->state_id;
                successor->data_[g_num].g = state->data_[g_num].g + cost;
                successor->data_[g_num].h = computeHeuristic(successor->state_id, (size_t)g_num);
                successor->data_[g_num].f = successor->data_[g_num].g + successor->data_[g_num].h;
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

void ims::MGS::connectGraphs(int graph_id1, int graph_id2, int state_id){
    // Use the root of the graph that the heuristic value between its root and the start state is the lowest.
    // Then, combine both open lists into one, which is the open belongs to the graph with the lowest heuristic value.
    SearchState* root1 = roots_[graph_id1];
    SearchState* root2 = roots_[graph_id2];
    double h1 = computeHeuristic(start_->state_id, root1->state_id);
    double h2 = computeHeuristic(start_->state_id, root2->state_id);
    int min_h_graph_id = h1 < h2 ? graph_id1 : graph_id2;
    int max_h_graph_id = h1 < h2 ? graph_id2 : graph_id1;

    if (params_.verbose)
        std::cout << "Connecting graphs (min, max): " << min_h_graph_id << ", " << max_h_graph_id << std::endl;

    closed_graphs_.push_back(max_h_graph_id);
    // connect the graphs
    hl_graph_[min_h_graph_id].push_back(max_h_graph_id);
    hl_graph_[max_h_graph_id].push_back(min_h_graph_id);

    if (max_h_graph_id == goal_in_graph_) {
        goal_in_graph_ = min_h_graph_id;
    }
    ////////// DEBUG /////////// TODO: delete this debug block
//    static int goal_graph_id = GRAPH_GOAL;
//    bool goal_graph = false; bool goal_expanded = false;
//    if (max_h_graph_id == goal_graph_id){
//        goal_graph = true;
//        goal_graph_id = min_h_graph_id;
//    }
    ////////////////////////////

    // Update the values of the states that where expanded from the root of the graph with the higher heuristic value.
    // first, back propagate the values from the state to the root
    SearchState* state = getSearchState(state_id); // the state that was expanded
    // make sure that the values of the state are updated correctly,
    // based on the graph with the lower heuristic value
    if (state->graph_closed == max_h_graph_id) {
        state->graph_closed = min_h_graph_id;
        state->data_[min_h_graph_id].h_self = computeHeuristic(state->state_id,
                                                               roots_[min_h_graph_id]->state_id);
        if (state->data_[min_h_graph_id].h_self > max_h_states_[min_h_graph_id]->data_[min_h_graph_id].h_self) {
            max_h_states_[min_h_graph_id] = state;
        }
        state->data_[min_h_graph_id].h = computeHeuristic(state->state_id, (size_t)min_h_graph_id);
        state->data_[min_h_graph_id].f = state->data_[min_h_graph_id].g + state->data_[min_h_graph_id].h;
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
        if (curr_state->state->me->in_closed) {
            curr_state->state->me->graph_closed = min_h_graph_id;
        }
        ////////// debug ///////// TODO: delete this debug block
//        if (curr_state->state_id == 1) {
//            goal_expanded = true;
//        }
        /////////////////////////
        for (auto &succ_id : *curr_state->state->me->data_[max_h_graph_id].edges) {
            auto succ = getSearchState(succ_id.first);
            // check if was already in the open list
            if (states_map.find(succ_id.first) != states_map.end()) {
                auto succ_connect_state = states_map.at(succ_id.first);
                if (succ_connect_state->expanded) {
                    continue;
                }
                if (succ_connect_state->state->g > curr_state->state->g + succ_id.second) {
                    succ_connect_state->state->parent_id = curr_state->state_id;
                    succ_connect_state->state->g = curr_state->state->g + succ_id.second;
                    succ_connect_state->state->h_self = computeHeuristic(succ_connect_state->state_id,
                                                                         roots_[min_h_graph_id]->state_id);
                    if (succ_connect_state->state->h_self
                        > max_h_states_[min_h_graph_id]->data_[min_h_graph_id].h_self) {
                        max_h_states_[min_h_graph_id] = succ_connect_state->state->me;
                    } // I think can be removed
                    succ_connect_state->state->h =
                        computeHeuristic(succ_connect_state->state_id, (size_t) min_h_graph_id);
                    succ_connect_state->state->f = succ_connect_state->state->g + succ_connect_state->state->h;
                    // update the open list
                    open_list.push(succ_connect_state); // TODO: not efficient, can have duplicates
                }
            } else {
                auto state_from_max = &succ->data_[max_h_graph_id];
                auto state_from_min = &succ->data_[min_h_graph_id];
                if (state_from_min->is_closed) {
                    continue;
                }
                if (state_from_min->g > curr_state->state->g + succ_id.second) {
                    state_from_min->parent_id = curr_state->state_id;
                    state_from_min->g = curr_state->state->g + succ_id.second;
                    if (!state_from_min->is_open) {
                        state_from_min->setOpen();
                    } else {
                        // delete edges
                        state_from_min->edges->clear();
                    }
                }

                /////// debug ////////// // TODO: delete this debug block
//                if ((state_from_min->me->state_id == curr_state->state->parent_id) &&
//                    (curr_state->state->me->data_[min_h_graph_id].parent_id == state_from_min->parent_id))
//                    std::cout << "Loop" << std::endl;
                ////////////////////////

                state_from_min->h_self = computeHeuristic(succ->state_id,
                                                          roots_[min_h_graph_id]->state_id);
                if (state_from_min->h_self > max_h_states_[min_h_graph_id]->data_[min_h_graph_id].h_self) {
                    max_h_states_[min_h_graph_id] = succ->data_[min_h_graph_id].me;
                }
                state_from_min->h = computeHeuristic(succ->state_id, (size_t) min_h_graph_id);
                state_from_min->f = state_from_min->g + state_from_min->h;
                // add edges
                for (auto& edges : *state_from_max->edges){
                    if (std::find(state_from_min->edges->begin(),
                                  state_from_min->edges->end(),
                                  edges) == state_from_min->edges->end()){
                        state_from_min->edges->emplace_back(edges);
                    }
                }
                // do i need to set it to closed if it is in closed of max_h_graph_id?
                if (state_from_max->is_closed){
                    state_from_min->setClosed();
                }
                auto succ_connect_state = std::make_shared<ConnectState>(succ->state_id, state_from_min);
                open_list.push(succ_connect_state);
                states_map.emplace(succ->state_id, succ_connect_state);
            }
        }
    }


//    //////////// test/////////////
//    std::vector<int> states_in_max_h_graph;
//    // loop through the states_ and if it is in max_h_graph_id count it
//    for (auto& s : states_){
//        if (s->data_[max_h_graph_id].g != INF_DOUBLE){
//            states_in_max_h_graph.push_back(s->state_id);
//        }
//    }
//
//    bool goal_graph = false;
//    if (max_h_graph_id == 1) {
//        std::cout << RED << "Connecting goal graph " << RESET << std::endl;
//        goal_graph = true;
//    }
//    /////////////////////////////
//    bool goal_expanded = false;
//
//    std::queue<SearchState*> local_open_list;
//    local_open_list.push(state);
//    std::vector<int> in_open_list;
//    in_open_list.push_back(state->state_id);
//    std::vector<int> updates ;
//    while (!local_open_list.empty()) {
//        // FIFO
//        SearchState* curr_state = local_open_list.front();
//        local_open_list.pop();
//        curr_state->use_graph_ = min_h_graph_id;
//        if (curr_state->in_closed) {
//            curr_state->graph_closed = min_h_graph_id;
//        }
////        curr_state->data_[min_h_graph_id].is_closed = true;)
//        curr_state->data_[min_h_graph_id].is_closed = true;
//        updates.push_back(curr_state->state_id);
//        if (curr_state->state_id == 1) {
//            std::cout << RED << "Goal State Expanded" << RESET << std::endl;
//            goal_expanded = true;
//        }
//
//        // look at all the successors of the state based on the edges in the graph
//        for (auto& succ_id : *curr_state->data_[max_h_graph_id].edges){
//            if (succ_id.first == 1)
//                std::cout << RED << "Goal State" << RESET << std::endl;
//            curr_state->data_[min_h_graph_id].edges->emplace_back(succ_id);
//            auto succ = getSearchState(succ_id.first);
//            if (succ->data_[min_h_graph_id].is_closed){
//                continue;
//            }
//            // check if was already in the open list
//            if (std::find(in_open_list.begin(), in_open_list.end(), succ->state_id) != in_open_list.end()){
//                if (succ->data_[min_h_graph_id].g > curr_state->data_[min_h_graph_id].g + succ_id.second){
//                    succ->data_[min_h_graph_id].parent_id = curr_state->state_id;
//                    succ->data_[min_h_graph_id].g = curr_state->data_[min_h_graph_id].g + succ_id.second;
//                    succ->data_[min_h_graph_id].h_self = computeHeuristic(succ->state_id,
//                                                                          roots_[min_h_graph_id]->state_id);
//                    if (succ->data_[min_h_graph_id].h_self > max_h_states_[min_h_graph_id]->data_[min_h_graph_id].h_self){
//                        max_h_states_[min_h_graph_id] = succ;
//                    } // I think can be removed
//                    succ->data_[min_h_graph_id].h = computeHeuristic(succ->state_id, (size_t)min_h_graph_id);
//                    succ->data_[min_h_graph_id].f = succ->data_[min_h_graph_id].g + succ->data_[min_h_graph_id].h;
//                }
//                continue;
//            }
//            succ->data_[min_h_graph_id].parent_id = curr_state->state_id;
//            succ->data_[min_h_graph_id].g = curr_state->data_[min_h_graph_id].g + succ_id.second;
//            succ->data_[min_h_graph_id].h_self = computeHeuristic(succ->state_id,
//                                                                  roots_[min_h_graph_id]->state_id);
//            if (succ->data_[min_h_graph_id].h_self > max_h_states_[min_h_graph_id]->data_[min_h_graph_id].h_self){
//                max_h_states_[min_h_graph_id] = succ;
//            }
//            succ->data_[min_h_graph_id].h = computeHeuristic(succ->state_id, (size_t)min_h_graph_id);
//            succ->data_[min_h_graph_id].f = succ->data_[min_h_graph_id].g + succ->data_[min_h_graph_id].h;
//
//            local_open_list.push(succ);
//            in_open_list.push_back(succ->state_id);
//        }
//    }
//    ////////// test //////////////
//    // check that all the states in the max_h_graph_id are in the updates
//    for (auto& s : states_in_max_h_graph){
//        if (std::find(updates.begin(), updates.end(), s) == updates.end()){
//            auto ss = getSearchState(s);
//            std::cout << "state " << s << " is not in the updates" << std::endl;
//        }
//    }
//    /////////////////////////////
//
//    if (goal_graph && !goal_expanded){
//        std::cout << "Goal graph was not expanded" << std::endl;
//    }

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
}

void ims::MGS::setStateVals(int state_id, int parent_id, double cost, int g_num) {
    auto state_ = getSearchState(state_id);
    state_->use_graph_ = g_num;
    state_->data_[g_num].parent_id = parent_id;
    state_->data_[g_num].g = getSearchState(parent_id)->data_[g_num].g + cost;
    state_->data_[g_num].h_self = computeHeuristic(state_id, roots_[g_num]->state_id);
    if (state_->data_[g_num].h_self > max_h_states_[g_num]->data_[g_num].h_self){
        max_h_states_[g_num] = state_;
    }
    state_->data_[g_num].h = computeHeuristic(state_id, (size_t)g_num);
    state_->data_[g_num].f = state_->data_[g_num].g + state_->data_[g_num].h;
}


void ims::MGS::reconstructPath(std::vector<StateType>& path) { // TODO: implement from scratch
    goal_ = goals_[0]; // TODO: fix this
    SearchState* state = getSearchState(goal_);
    while (state->data_[GRAPH_START].parent_id != -1){
        path.push_back(action_space_ptr_->getRobotState(state->state_id)->state);
        state = getSearchState(state->data_[GRAPH_START].parent_id);
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
    if (state_->data_[GRAPH_START].parent_id == -3){
        bool bp = true;
    }
    while (state_->data_[GRAPH_START].parent_id != -1){
        if ((std::find(path.begin(), path.end(), action_space_ptr_->getRobotState(state_->state_id)->state) != path.end()) &&
            (state_->data_[GRAPH_START].parent_id != -1)){
            std::cout << "Loop in the path" << std::endl;
        }
        path.push_back(action_space_ptr_->getRobotState(state_->state_id)->state);

        // Get the transition cost. This is the difference between the g values of the current state and its parent.
        double transition_cost = state_->data_[GRAPH_START].g -
            getSearchState(state_->data_[GRAPH_START].parent_id)->data_[GRAPH_START].g;
        costs.push_back(transition_cost);

        state_ = getSearchState(state_->data_[GRAPH_START].parent_id);
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

void ims::MGS::saveData() {
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

