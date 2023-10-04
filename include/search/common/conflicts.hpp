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
 * \file   conflicts.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   July 15 2023
*/

#ifndef SEARCH_COMMON_CONFLICTS_HPP
#define SEARCH_COMMON_CONFLICTS_HPP

// standard includes
#include <functional>
#include <algorithm>
#include <utility>
#include <vector>
#include <memory>
#include <eigen3/Eigen/Dense>   

// Project includes.
#include <search/common/types.hpp>

namespace ims {

enum class ConflictType {
    UNSET = -1,
    VERTEX = 0,
    EDGE = 1,
    POINT3D = 4,
};

/// @brief Base class for all search conflicts.
struct Conflict {
    /// @brief Constructor
    explicit Conflict() = default;

    /// @brief Virtual destructor.
    virtual ~Conflict() = default;

    /// @brief Getter method for all the member variables.
    // virtual std::vector<int> getMemberVariables() const = 0;

    /// @brief Get a string describing the conflict.
    virtual std::string toString() const = 0;

    /// @brief The type of the conflict.
    ConflictType type;
};

// ==========================
// Conflicts for CBS. 
// In these, each agent is specified its own state (or state transition), so these conflicts are valid for private grids as well.
// ==========================
/// @brief A struct for storing a vertex conflict on private grids.
struct VertexConflict : public Conflict {
    /// @brief The state vector. Could be a robot configuration.
    // We specify the states directly since their ID may change in future low-level plan iterations.
    std::vector<StateType> states;

    // The agent IDs.
    std::vector<int> agent_ids;

    /// @brief Constructor, allowing to set the state, time, and type.
    /// @param state The state vector.
    explicit VertexConflict(std::vector<StateType> states, std::vector<int> agent_ids) : states(std::move(states)), agent_ids(std::move(agent_ids)) {
        /// @brief The type of the Conflict.
        type = ConflictType::VERTEX;
    }

    std::string toString() const override {
        std::string str = "Vertex conflict at states: ";
        for (const auto& s : states) {
            for (const auto& ss : s) {
                str += std::to_string(ss) + " ";
            }
            str += "and ";
        }
        str += "for agents: ";
        for (const auto& a : agent_ids) {
            str += std::to_string(a) + " ";
        }
        return str;
    }
};

/// @brief A struct for storing an edge conflict on private grids.
struct EdgeConflict : public Conflict {
    /// @brief The state vector. Could be a robot configuration.
    // We specify the states directly since their ID may change in future low-level plan iterations.
    std::vector<StateType> from_states;
    std::vector<StateType> to_states;

    // The agent IDs.
    std::vector<int> agent_ids;

    /// @brief Constructor, allowing to set the state, time, and type.
    /// @param state The state vector.
    explicit EdgeConflict(std::vector<StateType> from_states, std::vector<StateType> to_states, std::vector<int> agent_ids) : from_states(std::move(from_states)), to_states(std::move(to_states)), agent_ids(std::move(agent_ids)) {
        /// @brief The type of the Conflict.
        type = ConflictType::EDGE;
    }

    std::string toString() const override {
        std::string str = "Edge conflict at states: ";
        for (const auto& s : from_states) {
            for (const auto& ss : s) {
                str += std::to_string(ss) + " ";
            }
            str += "and ";
        }
        str += "and ";
        for (const auto& s : to_states) {
            for (const auto& ss : s) {
                str += std::to_string(ss) + " ";
            }
            str += "and ";
        }
        str += "for agents: ";
        for (const auto& a : agent_ids) {
            str += std::to_string(a) + " ";
        }
        return str;
    }
};

// ==========================
// Conflicts for MRAMP.
// ==========================
/// @brief A struct for storing a vertex conflict on private grids.
struct Point3dConflict : public Conflict {
    /// @brief The state vector. Could be a robot configuration.
    // We specify the states directly since their ID may change in future low-level plan iterations. The collision either happened in a transition between states or in a state itself. If a conflict is specified for a transition, then we specify both the from and to states. If a conflict is specified for a state, then those are going to be the same.
    std::vector<StateType> from_states;
    std::vector<StateType> to_states;

    // The agent IDs.
    std::vector<int> agent_ids;

    // The point of conflict.
    Eigen::Vector3d point;

    /// @brief Constructor, allowing to set the state_from, state_to, ids, and point.
    /// @param state The state vector.
    explicit Point3dConflict(const std::vector<StateType>& from_states,
                             const std::vector<StateType>& to_states, 
                             const std::vector<int> & agent_ids,
                             const Eigen::Vector3d& point) : 
                                from_states(from_states),
                                to_states(to_states), 
                                agent_ids(agent_ids),
                                point(point) {
    
    /// @brief The type of the Conflict.
    type = ConflictType::POINT3D;
    }

    /// @brief Constructor, allowing to set the from state, ids, and point.
    /// @param state The state vector.
    explicit Point3dConflict(const std::vector<StateType>& from_states,
                             const std::vector<int> & agent_ids,
                             const Eigen::Vector3d& point) : 
                                from_states(from_states),
                                agent_ids(agent_ids),
                                point(point) {
    
    /// @brief The type of the Conflict.
    type = ConflictType::POINT3D;

    // Set the to states to be empty.
    to_states = {};
    }

    std::string toString() const override {
        std::string str = "Point3d conflict at states: ";
        for (const auto& s : from_states) {
            for (const auto& ss : s) {
                str += std::to_string(ss) + " ";
            }
            str += "and ";
        }
        str += "and ";
        for (const auto& s : to_states) {
            for (const auto& ss : s) {
                str += std::to_string(ss) + " ";
            }
            str += "and ";
        }
        str += "for agents: ";
        for (const auto& a : agent_ids) {
            str += std::to_string(a) + " ";
        }
        str += "at point: " + std::to_string(point[0]) + " " + std::to_string(point[1]) + " " + std::to_string(point[2]);
        return str;
    }
};

}  // namespace ims

#endif //SEARCH_COMMON_CONFLICTS_HPP
