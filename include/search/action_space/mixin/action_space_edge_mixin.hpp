/*
 * Copyright (C) 2024, Hanlan Yang
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
 * \file   action_space_edge_mixin.hpp
 * \author Hanlan Yang (yanghanlan666@gmail.com)
 * \date   July 19 2024
 */

#pragma once

// standard includes
#include <algorithm>
#include <functional>
#include <utility>

// project includes
#include <search/action_space/action_space.hpp>
#include <search/heuristics/base_heuristic.hpp>
#include <search/planners/planner.hpp>

namespace ims {

///@brief RobotEdge base class
struct RobotEdge {
    StateType state_mapped;
    StateType state;
    ActionSequence action;

    enum VALID { valid,
                 invalid,
                 unknown };
    VALID validity = unknown;
};

/// @brief equality operator for RobotEdge
inline bool operator==(const RobotEdge& lhs, const RobotEdge& rhs) {
    return lhs.state == rhs.state && lhs.action == rhs.action;
}
}  // namespace ims

namespace std {
template <>
struct hash<ims::RobotEdge> {
    using argument_type = ims::RobotEdge;
    using result_type = std::size_t;

    result_type operator()(const argument_type& s) const {
        // Setting a different seed to prevent hash collision with RobotState
        size_t seed = 9;
        boost::hash_combine(seed, boost::hash_range(s.state.begin(), s.state.end()));
        boost::hash_combine(seed, boost::hash_range(s.action.begin(), s.action.end()));
        return seed;
    }
};
}  // namespace std

namespace ims {

/// @brief A trait class (mixin) for allowing an ActionSpace to use the "edge" representation, where an edge is a pair of state action (s,a).
class ActionSpaceEdgeMixin {
   public:
    // Members
    using EdgeKey = RobotEdge;
    using EdgeHash = SharedPointerValueHash<EdgeKey>;
    using EdgeEqual = SharedPointerValueEqual<EdgeKey>;
    hash_map<std::shared_ptr<EdgeKey>, int, EdgeHash, EdgeEqual> edge_to_id_;
    /// @brief The edges
    std::vector<std::shared_ptr<RobotEdge>> edges_;

    /// @brief Constructor
    explicit ActionSpaceEdgeMixin() = default;

    /// @brief Destructor
    ~ActionSpaceEdgeMixin() = default;

    /// @brief Get a edge by id
    /// @param id The id of the edge
    /// @return The edge
    /// @note The id is assumed to be valid - meaning that the edge exists in edges_
    virtual auto getRobotEdge(size_t id) -> std::shared_ptr<RobotEdge> {
        lock_e_.lock();
        assert(id < edges_.size() && id >= 0);
        lock_e_.unlock();
        return edges_[id];
    }

    /// @brief Get a edge by state value and action pair
    /// @param  edge_pair = std::pair<StateType, ActionSequence> The state value and action pair
    /// @return The robot edge
    virtual auto getRobotEdge(const std::pair<StateType, ActionSequence>& edge_pair) -> std::shared_ptr<RobotEdge> {
        lock_e_.lock();
        auto curr_edge = std::make_shared<RobotEdge>();
        curr_edge->state = edge_pair.first;
        curr_edge->action = edge_pair.second;
        auto it = edge_to_id_.find(curr_edge);
        if (it != edge_to_id_.end()) {
            lock_e_.unlock();
            return edges_[it->second];
        } else {
            // Raise an error.
            std::cout << "getRobotEdge via edge_pair failed. Edge not found." << std::endl;
            throw std::runtime_error("getRobotEdge via edge_pair failed. Edge not found.");
            lock_e_.unlock();
            return nullptr;
        }
    }

    /// @brief Get a edge id by edge_pair (state value and action pair)
    /// @param  edge_pair = std::pair<StateType, ActionSequence> The state value and action pair
    /// @return The robot edge id
    /// @note The id is assumed to be valid - meaning that the edge exists in edges_
    virtual int getRobotEdgeId(const std::pair<StateType, ActionSequence>& edge_pair) {
        lock_e_.lock();
        auto curr_edge = std::make_shared<RobotEdge>();
        curr_edge->state = edge_pair.first;
        curr_edge->action = edge_pair.second;
        auto it = edge_to_id_.find(curr_edge);
        if (it != edge_to_id_.end()) {
            lock_e_.unlock();
            return it->second;
        } else {
            // Raise an error.
            std::cout << "getRobotEdge via edge_pair failed. Edge not found." << std::endl;
            throw std::runtime_error("getRobotEdge via edge_pair failed. Edge not found.");
            lock_e_.unlock();
            return -1;
        }
    }

    /// @brief Get or create edge by edge pair
    /// @param  edge_pair = std::pair<StateType, ActionSequence> The state value and action pair
    /// @return The edge id
    virtual int getOrCreateRobotEdge(const std::pair<StateType, ActionSequence>& edge_pair) {
        // check if the edge exists
        lock_e_.lock();
        auto curr_edge = std::make_shared<RobotEdge>();
        curr_edge->state = edge_pair.first;
        curr_edge->action = edge_pair.second;
        auto it = edge_to_id_.find(curr_edge);
        if (it != edge_to_id_.end()) {
            lock_e_.unlock();
            return it->second;
        } else {
            edges_.push_back(curr_edge);
            int edge_id = (int)edges_.size() - 1;
            edge_to_id_[curr_edge] = edge_id;
            lock_e_.unlock();
            return edge_id;
        }
    }

    /// @brief Create robot edges from a proxy edge
    /// @param  proxy_edge_id The id of the proxy edge
    /// @param  edges_ind& The index of created real-edges
    /// @return Success bool
    /// @note The id is assumed to be valid - meaning that the state exists in states_
    virtual bool createRobotEdgesFromState(int state_id, std::vector<int>& edges_ind) = 0;
    
    /// @brief Get single Successor by edge id
    /// @param curr_egde_ind The current state index
    /// @param successors The successor state
    /// @return Success bool
    /// @note Beware the you should make sure that the state is discretized! (see ActionType::Discretization)
    /// If you are using an implicit graph where the state space is not discrete then define the discretization
    /// based on the tolerance for comparison between states.
    /// @attention You should use getOrCreateRobotState() and getRobotState() when generating the successors!
    virtual bool getSuccessor(int curr_edge_ind,
                              int& successor,
                              double& cost) = 0;

    /// @brief Clear all the cached edges data in the action space.
    virtual void resetPlanningData() {
        edges_.clear();
        edge_to_id_.clear();
    }

   protected:
    /// @brief  Lock for the action space to make it multi-thread safe
    LockType lock_e_;
};

}  // namespace ims
