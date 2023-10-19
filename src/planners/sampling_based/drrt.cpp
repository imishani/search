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
    for (int agent_id{0}; agent_id < num_agents_; ++agent_id) {
        auto action_space_ptr = agent_action_space_ptrs_[agent_id];
        if (!action_space_ptr->hasNonEmptyRoadmap()) {
            action_space_ptr->createRoadmap(starts_[agent_id], goals_[agent_id], 500);
        }
    }
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Time to create roadmaps: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;

    // Create a root node in the search tree.
    SearchState* root = getOrCreateSearchState(0);
    root->parent_id = PARENT_TYPE(START);

    // The root node is a composite state of all the individual agents' start states. Get the state id for each agent on their individual roadmap.
    std::vector<int> agent_state_ids(num_agents_, 0);
    for (int agent_id{0}; agent_id < num_agents_; ++agent_id) {
        agent_state_ids[agent_id] = agent_action_space_ptrs_[agent_id]->getRobotStateId(starts_[agent_id]);
    }
    root->agent_state_ids = agent_state_ids;
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

void ims::dRRT::sampleCompositeState(MultiAgentStateType& sampled_composite_state){
    // Sample a state from each agent's action space and stack them all, in order.
    for (int agent_id{0}; agent_id < num_agents_; ++agent_id) {
        StateType sampled_state;
        agent_action_space_ptrs_[agent_id]->sampleState(sampled_state);
        sampled_composite_state[agent_id] = sampled_state;
    }
}

double ims::dRRT::distanceCompositeStates(const MultiAgentStateType& composite_state_1, const MultiAgentStateType& composite_state_2){
    // Compute the distance between two composite states.
    double distance = 0.0;

    for (auto agent_id_and_state : composite_state_1) {
        int agent_id = agent_id_and_state.first;   
        for (int dim{0}; dim < composite_state_1.at(agent_id).size(); ++dim) {
            distance += std::pow(composite_state_1.at(agent_id)[dim] - composite_state_2.at(agent_id)[dim], 2);
        }
    }
    distance = std::sqrt(distance);
    return distance;
}

void ims::dRRT::getNearestSearchState(const MultiAgentStateType& composite_state, SearchState*& nearest_state, double& nearest_state_distance){
    // Get the nearest state to the sampled state from the current search tree.
    nearest_state = nullptr;
    nearest_state_distance = std::numeric_limits<double>::max();

    for (auto& state : states_) {
        // Compute the distance between the sampled state and the state in the search tree.
        MultiAgentStateType tree_composite_state;
        for (int agent_id{0}; agent_id < num_agents_; ++agent_id) {
            // Get the state id of the agent on the roadmap.
            int agent_state_id = state->agent_state_ids[agent_id];

            // Get the state of the agent on the roadmap.
            StateType agent_state = agent_action_space_ptrs_[agent_id]->getRobotState(agent_state_id)->state;

            // Add to the composite state.
            tree_composite_state[agent_id] = agent_state;
        }

        // Compute the distance.
        double distance = distanceCompositeStates(composite_state, tree_composite_state);

        // Check if this is the nearest state.
        if (distance < nearest_state_distance) {
            nearest_state = state;
            nearest_state_distance = distance;
        }
    }
}

void ims::dRRT::getNearestSearchStates(const MultiAgentStateType& composite_state, std::vector<SearchState*>& nearest_states, std::vector<double>& nearest_state_distances, int num_nearest_states){
    // Get the nearest state to the sampled state from the current search tree.
    double nearest_state_distance = std::numeric_limits<double>::max();
    std::deque<SearchState*> nearest_states_hist;
    std::deque<double> nearest_state_distances_hist;

    for (auto& state : states_) {
        // Compute the distance between the sampled state and the state in the search tree.
        MultiAgentStateType tree_composite_state;
        for (int agent_id{0}; agent_id < num_agents_; ++agent_id) {
            // Get the state id of the agent on the roadmap.
            int agent_state_id = state->agent_state_ids[agent_id];

            // Get the state of the agent on the roadmap.
            StateType agent_state = agent_action_space_ptrs_[agent_id]->getRobotState(agent_state_id)->state;

            // Add to the composite state.
            tree_composite_state[agent_id] = agent_state;
        }

        // Compute the distance.
        double distance = distanceCompositeStates(composite_state, tree_composite_state);

        // Check if this is the nearest state.
        if (distance < nearest_state_distance) {
            nearest_state_distance = distance;

            // Add the nearest state to the history.
            nearest_states_hist.push_back(state);
            nearest_state_distances_hist.push_back(distance);

            // Keep the history size bounded.
            if (nearest_states_hist.size() > num_nearest_states) {
                nearest_states_hist.pop_front();
                nearest_state_distances_hist.pop_front();
            }
        }
    }

    // Copy the nearest states and distances to the output.
    nearest_states.clear();
    nearest_state_distances.clear();
    for (int i{0}; i < nearest_states_hist.size(); ++i) {
        nearest_states.push_back(nearest_states_hist[i]);
        nearest_state_distances.push_back(nearest_state_distances_hist[i]);
    }
}

void ims::dRRT::agentStateIdsToCompositeState(const std::vector<int>& agent_state_ids, MultiAgentStateType& composite_state){
    // Convert a vector of agent state ids to a composite state.
    for (int agent_id{0}; agent_id < num_agents_; ++agent_id) {
        // Get the state of the agent on the roadmap.
        StateType agent_state = agent_action_space_ptrs_[agent_id]->getRobotState(agent_state_ids[agent_id])->state;

        // Add to the composite state.
        composite_state[agent_id] = agent_state;
    }
}

void ims::dRRT::reconstructPath(SearchState* goal_state, MultiAgentPaths& paths){
    // Reconstruct a path from a state to the root, and return the reversed.
    std::vector<int> path_state_ids;
    SearchState* current_state = goal_state;
    while (current_state->parent_id != PARENT_TYPE(START)) {
        path_state_ids.push_back(current_state->state_id);
        current_state = getSearchState(current_state->parent_id);
    }
    path_state_ids.push_back(current_state->state_id);

    // Reverse the path.
    std::reverse(path_state_ids.begin(), path_state_ids.end());

    // Convert the path to a MultiAgentPath.
    for (int agent_id{0}; agent_id < num_agents_; ++agent_id) {
        // Get the agent's path.
        for (auto step_state_ids : path_state_ids) {
            int agent_state_id = states_.at(step_state_ids)->agent_state_ids[agent_id];
            StateType agent_state = agent_action_space_ptrs_[agent_id]->getRobotState(agent_state_id)->state;
            paths[agent_id].push_back(agent_state); 
        }
    }
}

bool ims::dRRT::plan(MultiAgentPaths& paths) {
    startTimer();
    int iter = 0;
    while(true || !isTimeOut()){
        if (iter % 100 == 0){
            std::cout << "Iteration: " << iter << std::endl;
        }
            

        // Check if should try to connect to goal.
        if (iter % 10 == 9){
            
            // Set the next state to be the goal state.
            std::vector<int> next_agent_state_ids(num_agents_, 0);
            for (int agent_id{0}; agent_id < num_agents_; ++agent_id) {
                next_agent_state_ids[agent_id] = agent_action_space_ptrs_[agent_id]->getRobotStateId(goals_[agent_id]);
            }
            MultiAgentStateType next_composite_state;
            agentStateIdsToCompositeState(next_agent_state_ids, next_composite_state);

            // Get the nearest composite state to the goal state from the current search tree.
            std::vector<SearchState*> nearest_states;
            std::vector<double> nearest_state_distances;
            getNearestSearchStates(next_composite_state, nearest_states, nearest_state_distances, 15);

            for (int i{0}; i < nearest_states.size(); ++i) {
                // Get the nearest composite state.
                MultiAgentStateType nearest_composite_state;

                SearchState* nearest_state = nearest_states[i];
                agentStateIdsToCompositeState(nearest_state->agent_state_ids, nearest_composite_state);

                // Verify that this transition is valid and add it to the search tree if it is.
                MultiAgentPaths transition_paths;
                bool is_valid_transition = agent_action_space_ptrs_[0]->multiAgentStateToStateConnector(nearest_composite_state, next_composite_state, transition_paths, agent_names_);
                if (is_valid_transition) {
                    // Add a new composite state to the tree.
                    // NOTE: there is currently no use of the paths returned by the connector.
                    SearchState* new_state = getOrCreateSearchState(states_.size());
                    new_state->agent_state_ids = next_agent_state_ids;
                    new_state->parent_id = nearest_state->state_id;
                    new_state->g = nearest_state->g + 1.0;

                    // Reconstruct path, this is a goal state.
                    reconstructPath(new_state, paths);

                    // Update stats.
                    getTimeFromStart(stats_.time);
                    return true;
                }
            }
        }
        else{
            // Sample a composite state.
            MultiAgentStateType sampled_composite_state;
            sampleCompositeState(sampled_composite_state);

            // Get the nearest composite state to the sampled state from the current search tree.
            double nearest_state_distance = std::numeric_limits<double>::max();
            SearchState* nearest_state = nullptr;
            getNearestSearchState(sampled_composite_state, nearest_state, nearest_state_distance);

            // From the nearest composite search state, get the individual agent state ids.
            std::vector<int> nearest_agent_state_ids = nearest_state->agent_state_ids;
            MultiAgentStateType nearest_composite_state;
            agentStateIdsToCompositeState(nearest_agent_state_ids, nearest_composite_state);

            // For each agent, ask the action space to get its neighbor states. Among those, ask for their angles to the sampled state. The agent with the smallest angle is the one that will be used in the new composite state.
            std::vector<int> next_agent_state_ids(num_agents_, 0);
            for (int agent_id{0}; agent_id < num_agents_; agent_id++){
                agent_action_space_ptrs_[agent_id]->getSuccessorInDirection(nearest_agent_state_ids[agent_id],  // Nearest on the single-agent roadmap. 
                                                                            next_agent_state_ids[agent_id],     // The next state id on the single-agent roadmap. To be populated.
                                                                            sampled_composite_state[agent_id]); // The sampled state for the agent.
            }


            // Create the tentative new composite state.
            MultiAgentStateType next_composite_state;
            agentStateIdsToCompositeState(next_agent_state_ids, next_composite_state);

            // Verify that this transition is valid and add it to the search tree if it is.
            MultiAgentPaths transition_paths;
            bool is_valid_transition = agent_action_space_ptrs_[0]->multiAgentStateToStateConnector(nearest_composite_state, next_composite_state, transition_paths, agent_names_);
            if (is_valid_transition) {
                // Add a new composite state to the tree.
                // NOTE(yoraish): there is currently no use of the paths returned by the connector.
                SearchState* new_state = getOrCreateSearchState(states_.size());
                new_state->agent_state_ids = next_agent_state_ids;
                new_state->parent_id = nearest_state->state_id;
                new_state->g = nearest_state->g + 1.0;            
            }
        }

    
    iter++;
    }

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
