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
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/datastructures/NearestNeighbors.h>

// project includes
#include "action_space.hpp"
#include <search/heuristics/base_heuristic.hpp>
#include <search/planners/planner.hpp>
#include <search/common/kdtree.hpp>

namespace ims {

/// @brief Base class for ActionSpaces with constraints.
/// @details This is an actions space extended to be "Constrainable" using a mixin.
class RoadmapActionSpace : virtual public ActionSpace{
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
    inline void createRoadmap(const double timeout){
        std::cout << "RoadmapActionSpace: createRoadmap" << std::endl;
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        // Clear the roadmap.
        adjacency_mat_.clear();
        kdtree_ptr_.reset();

        // The sampled states.  
        std::vector<StateType> sampled_states;

        // Create the roadmap.
        while (true) {
            // Check if the timeout has been reached.
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            double elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() / 1000.0;
            if (elapsed_time > timeout) {
                break;
            }

            // Sample a state.
            StateType state;
            sampleState(state);
            sampled_states.push_back(state);
            
        }

        // Create a kd-tree for the roadmap and initialize the roadmap adjacency list.
        int dim_state = sampled_states[0].size();
        kdtree_ptr_ = std::make_shared<KDTree<StateType>>(sampled_states, dim_state);
        adjacency_mat_.resize(sampled_states.size());

        // Create a roadmap adjacency list for the roadmap alongside populating the states_ vector in the action space.
        int num_edges = 0;
        for (int i = 0; i < sampled_states.size(); i++) {
            bool is_edge_added = addStateToRoadmap(sampled_states[i]);

            if (is_edge_added) {
                num_edges++;
            }
        }

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        num_edges /= 2;
        std::cout << "Created a roadmap with " << sampled_states.size() << " states and " << num_edges << " edges in " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() / 1000.0 << " seconds." << std::endl; 
    }

    /// @brief Sample a valid state from the action space.
    virtual void sampleState(StateType& state) const = 0;

    /// @brief Check if a state transition is valid.
    virtual bool isStateToStateValid(const StateType& state_from, const StateType& state_to) = 0;

    /// @brief Add a state to the roadmap.
    bool addStateToRoadmap(const StateType& state){

        // Create a robot state for the state.
        int state_id = getOrCreateRobotState(state);

        // Find any neighbor state.
        StateType nearest = kdtree_ptr_->nearestNeighbor(state);
        double dist = kdtree_ptr_->euclidean_distance(state, nearest);

        // Get the state id of the nearest neighbor.
        int nearest_id = getOrCreateRobotState(nearest);

        // If the nearest neighbor is within the radius, then add it to the adjacency list.
        if (dist < 0.1) {
            // Check if the transition is valid w.r.t. static obstacles.            
            if (!isStateToStateValid(state, nearest)) {
                return false;
            }

            // TODO(yoraish): check if the branching of the nodes is already too high.

            // Add the state to the adjacency list.
            adjacency_mat_.at(nearest_id).insert(state_id);
            adjacency_mat_.at(state_id).insert(nearest_id);
            return true;
        }

        return false;
    }

private:

    /// @brief The roadmap vertices. Store it for efficient nearest neighbor lookup as a KD-tree.
    std::shared_ptr<KDTree<StateType>> kdtree_ptr_;

    /// @brief The roadmap topology (connectivity, edges). Represented as an adjacency list, with each state_id mapping to a vector of state_ids.
    std::vector<std::unordered_set<int>> adjacency_mat_;
};

}  // namespace ims

