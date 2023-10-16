/*
 * Copyright (C) 2023, Yorai Shaoul
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
 * \file   drrt.cpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   OCtober 15 2023
 */

#include <search/planners/sampling_based/drrt.hpp>
#include <chrono>


ims::dRRT::dRRT(const ims::dRRTParams& params) : params_(params), MultiAgentPlanner(params){}


void ims::dRRT::initializePlanner(std::vector<std::shared_ptr<RoadmapActionSpace>>& action_space_ptrs,
                                 const std::vector<StateType>& starts, const std::vector<StateType>& goals) {

    
    // Store the action spaces. This must happen before checking for the validity of the start and end states.
    agent_action_space_ptrs_ = action_space_ptrs;

    // Check if the inputs are valid.
    if (starts.size() != agent_action_space_ptrs_.size() || goals.size() != agent_action_space_ptrs_.size()) {
        throw std::runtime_error("Start state vector size (" + std::to_string(starts.size()) + ") does not match the number of agents (" + std::to_string(agent_action_space_ptrs_.size()) + ")");
    }

    // Check start and end states for validity. Individually and between agents.
    verifyStartAndGoalInputStates(starts, goals);

    // Store the starts and goals.
    starts_ = starts;
    goals_ = goals;

    // Set the number of agents.
    num_agents_ = (int)starts.size();

    // Measure the time it takes to create all the roadmaps.
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    // Check if all the action spaces have a roadmap. If not, then create one for each.
    for (auto& action_space_ptr : agent_action_space_ptrs_) {
        if (!action_space_ptr->hasNonEmptyRoadmap()) {
            action_space_ptr->createRoadmap(1);
        }
    }
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Time to create roadmaps: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;

}

void ims::dRRT::initializePlanner(std::vector<std::shared_ptr<RoadmapActionSpace>>& action_space_ptrs, const std::vector<std::string> & agent_names, const std::vector<StateType>& starts, const std::vector<StateType>& goals){
                        agent_names_ = agent_names;
                        initializePlanner(action_space_ptrs, starts, goals);
                        }

auto ims::dRRT::getSearchState(int state_id) -> ims::dRRT::SearchState* {
    assert(state_id < states_.size() && state_id >= 0);
    return states_[state_id];
}

auto ims::dRRT::getOrCreateSearchState(int state_id) -> ims::dRRT::SearchState* {
    if (state_id >= states_.size()) {
        states_.resize(state_id + 1, nullptr);
    }
    if (states_[state_id] == nullptr) {
        assert(state_id < states_.size() && state_id >= 0);
        states_[state_id] = new SearchState;
        states_[state_id]->state_id = state_id;
    }
    return states_[state_id];
}

bool ims::dRRT::plan(MultiAgentPaths& paths) {
    startTimer();
    

    // getTimeFromStart(stats_.time);
    return false;
}

// void ims::dRRT::setStateVals(int state_id, int parent_id, double cost) {
//     auto state = getSearchState(state_id);
//     auto parent = getSearchState(parent_id);
//     state->parent_id = parent_id;
// }

void ims::dRRT::verifyStartAndGoalInputStates(const std::vector<StateType>& starts, const std::vector<StateType>& goals) {
    // Check all goals have starts.
    if (starts.size() != goals.size()){
        throw std::runtime_error("Start state vector size (" + std::to_string(starts.size()) + ") does not match the goal state vector size (" + std::to_string(goals.size()) + ")");
    }
    // Check if the start and goal states are valid w.r.t time. All starts are t=0 and all goals are t=-1.
    for (size_t i{0}; i < starts.size(); ++i) {
        if (starts[i].back() != 0) {
            throw std::runtime_error("Start state for agent " + std::to_string(i) + " is not at time 0");
        }
        if (goals[i].back() != -1) {
            throw std::runtime_error("Goal state for agent " + std::to_string(i) + " is not at time -1");
        }
    }

    // Check if the start and goal states are valid. For each agent.
    for (size_t i{0}; i < starts.size(); ++i) {
        if (!agent_action_space_ptrs_[i]->isStateValid(starts[i])) {
            throw std::runtime_error("Start state for agent " + std::to_string(i) + " is not valid");
        }
    }

    for (size_t i{0}; i < goals.size(); ++i) {
        if (!agent_action_space_ptrs_[i]->isStateValid(goals[i])) {
            throw std::runtime_error("Goal state for agent " + std::to_string(i) + " is not valid");
        }
    }
}

void ims::dRRT::resetPlanningData(){

}
