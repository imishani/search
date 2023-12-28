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
 * \file   roadmap_action_space.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   October 15 2023
 */
#pragma once

// standard includes
#include <functional>
#include <algorithm>
#include <utility>
#include <unordered_set>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

// project includes
#include "action_space.hpp"
#include <search/heuristics/base_heuristic.hpp>
#include <search/planners/planner.hpp>
#include <search/common/distance.hpp>

namespace ims {

/// @brief Base class for ActionSpaces with constraints.
/// @details This is an actions space extended to be "Constrainable" using a mixin.
class RoadmapActionSpace : virtual public ActionSpace{
private:
public:
    /// @brief Constructor
    explicit RoadmapActionSpace(): ActionSpace() {
        std::cout << "RoadmapActionSpace: Constructor" << std::endl;

        // Set the state limits.
        // getStateLimits(state_limits_);
    }

    /// @brief Destructor
    ~RoadmapActionSpace() = default;

    /// @brief Get the actions available from a given state. 
    // virtual StateType

    /// @brief If the action space has a nonempty roadmap, return true.
    inline bool hasNonEmptyRoadmap() const {
        return !adjacency_mat_.empty();
    }

    /// @brief Create a roadmap with a given timeout in seconds.
    inline void createRoadmap(const StateType& start_state, const StateType& goal_state, int num_samples,
                              int num_neighbors = 10, double radius = INF_DOUBLE) {
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        // Clear the roadmap.
        adjacency_mat_.clear();
        sampled_states_.clear();

        // Create robot states for the start and goal.
        int start_state_id = getOrCreateRobotState(start_state);
        int goal_state_id =  getOrCreateRobotState(goal_state);

        // Add the start and goal.
        sampled_states_[start_state_id] = start_state;
        sampled_states_[goal_state_id] = goal_state;
        adjacency_mat_[start_state_id] = std::unordered_set<int>();
        adjacency_mat_[goal_state_id] = std::unordered_set<int>();

        // Create the roadmap.
        while (true) {
            // Check if enough states were sampled.
            if (sampled_states_.size() >= num_samples) {
                break;
            }

            // Sample a state.
            StateType state;
            sampleState(state);
            int new_state_id = getOrCreateRobotState(state);
            sampled_states_[new_state_id] = state;
            adjacency_mat_[new_state_id] = std::unordered_set<int>();
        }

        // Create the roadmap adjacency list.
        int dim_state = sampled_states_[0].size();

        // Create a roadmap adjacency list for the roadmap alongside populating the states_ vector in the action space.
        int num_edges = 0;
        for (auto state_id_and_state_val : sampled_states_) {
            int i = state_id_and_state_val.first;
            StateType state = state_id_and_state_val.second;
            int num_edges_added = addStateToRoadmap(sampled_states_[i]);

            if (num_edges_added  > 0) {
                num_edges += num_edges_added;
            }
        }

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "Created a roadmap with " << sampled_states_.size() << " states and " << num_edges << " edges in " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() / 1000.0 << " seconds." << std::endl;

        // Verify that the start state has at least one neighbor.
        if (adjacency_mat_[start_state_id].size() == 0) {
            std::cout << RED << "Start state has no neighbors. Scraping the roadmap and starting over with more samples." << RESET << std::endl;

            // Scrap the roadmap and start over with more samples.
            adjacency_mat_.clear();
            sampled_states_.clear();
            for (auto state : states_) {
                state_to_id_.erase(state);
                delete state;
            }
            states_.clear();

            createRoadmap(start_state, goal_state, num_samples + 1000);
        }

    }

    /// @brief Sample a valid state from the action space.
    virtual void sampleState(StateType& state) = 0;

    /// @brief Get the state limits.
    virtual void getStateLimits(std::vector<std::pair<double, double>>& state_limits) const = 0;
    virtual int getStateDimensionality() const = 0;

    /// @brief Check if a state transition is valid.
    virtual bool isStateToStateValid(const StateType& state_from, const StateType& state_to) = 0;

    /// @brief Check if a multiagent state transition is valid, and if it is also yield connecting path segments.
    /// @param state_from The state from which to connect.
    /// @param state_to The state to which to connect.
    /// @param paths The paths to populate with connecting path segments.
    /// @return True if the transition is valid, false otherwise.
    virtual bool multiAgentStateToStateConnector(const MultiAgentStateType& state_from, const MultiAgentStateType& state_to, MultiAgentPaths& paths, std::vector<std::string> agent_names) = 0;

    // inline double euclideanDistance(const StateType& point1, const StateType& point2){
    //     double sum = 0;
    //     for (int i = 0; i < point1.size(); i++){
    //         sum += pow(point1[i] - point2[i], 2);
    //     }
    //     return sqrt(sum);
    // }


    /// @brief Add a state to the roadmap. Returns the number of edges added.
    int addStateToRoadmap(const StateType& state, int max_neighbors = 10, double radius = 2.0){

        // Create a robot state for the state.
        int state_id = getOrCreateRobotState(state);

        // Find any neighbor state.
        // Loop over all sampled states and find the nearest neighbor.
        double nearest_state_dist = std::numeric_limits<double>::infinity();

        // Keep a history of the nearest state ids.
        int num_nearest_states_to_keep = max_neighbors;
        std::deque<int> nearest_state_id_hist;
        std::deque<StateType> nearest_state_hist;


        for(auto other_state_id_and_state_val : sampled_states_){
            int other_state_id = other_state_id_and_state_val.first;
            StateType other_state_val = other_state_id_and_state_val.second;

            // Nearest is not self.
            if (state_id == other_state_id) {
                continue;
            }

            double dist = euclideanDistance(state, other_state_val);

            if (dist < nearest_state_dist && dist > 0.0 && dist < radius) {
                nearest_state_dist = dist;

                // Add the nearest state to the history.
                nearest_state_id_hist.push_back(other_state_id);
                nearest_state_hist.push_back(other_state_val);

                // Keep the history size bounded.
                if (nearest_state_id_hist.size() > num_nearest_states_to_keep) {
                    nearest_state_id_hist.pop_front();
                    nearest_state_hist.pop_front();
                }

            }
        }
        
        for (int i = 0; i < nearest_state_id_hist.size(); i++){
            int nearest_state_id = nearest_state_id_hist[i];
            StateType nearest_state = nearest_state_hist[i];

            // Check if the transition is valid w.r.t. static obstacles.            
            if (isStateToStateValid(state, nearest_state)) {

                // NOTE(yoraish): add a potential check for the branching of the nodes is already too high.

                // Add the state to the adjacency list.
                adjacency_mat_.at(nearest_state_id).insert(state_id);
                adjacency_mat_.at(state_id).insert(nearest_state_id);
            }
        }
        if (adjacency_mat_.at(state_id).size() > 0){
            return nearest_state_id_hist.size();
        }

        return 0;
    }

    /// @brief Get the successor of a state that moves in a direction of a target state.
    /// @param state_id 
    /// @param state_id_succ 
    /// @param state_target 
    virtual void getSuccessorInDirection(int state_id, int& state_id_succ, StateType state_target) = 0;

    /// @brief Reset the planning data. In the roadmap case, this means that states_ is not cleared completely, but rather keeps the roadmap.
    virtual void resetPlanningData() override {

        // Create a copy of all the states that are in the roadmap.
        std::vector<RobotState*> states_copy;
        hash_map<StateKey*, int, StateHash, StateEqual> state_to_id_copy;

        for (auto state :states_) {
            if (sampled_states_.find(state_to_id_[state]) != sampled_states_.end()) {
                states_copy.push_back(state);
                state_to_id_copy[state] = state_to_id_[state];
            }
        }

        // Clear the states_ vector.
        states_.clear();
        state_to_id_.clear();

        // Copy the states back.
        for (int i = 0; i < states_copy.size(); i++) {
            states_.push_back(states_copy[i]);
            state_to_id_[states_copy[i]] = state_to_id_copy[states_copy[i]];
        }
    }

    // Members.
    /// @brief The roadmap vertices. Store it for efficient nearest neighbor lookup as a KD-tree.
    std::unordered_map<int, StateType>sampled_states_; 

    /// @brief The roadmap topology (connectivity, edges). Represented as an adjacency list, with each state_id mapping to a vector of state_ids.
    std::unordered_map<int, std::unordered_set<int>> adjacency_mat_;
};

}  // namespace ims

