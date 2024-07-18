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
 * \file   constraints.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   July 13 2023
*/
#pragma once

// standard includes
#include <functional>
#include <algorithm>
#include <utility>
#include <vector>
#include <memory>
#include <unordered_map>
#include <eigen3/Eigen/Dense>

// Project includes.
#include <search/common/types.hpp>

namespace ims {

// Forward declaration for the ConstrainedActionSpace so that we can use it in the ConstraintsContext.
class ConstrainedActionSpace;

enum class ConstraintType {
    UNSET = -1,
    VERTEX = 0, // Do not be at v at time t.
    EDGE = 1, // Do not cross edge (u, v) at time t to t+1.
    VERTEX_SPHERE3D, // Do not be in sphere at time t.
    EDGE_SPHERE3D, // Do not cross edge (u, v) at time t to t+1 if the associated configuration interpolation collides with a sphere.
    VERTEX_PRIORITY, // Do not conflict with agent at time t, setting other agent as specified in its path in the context.
    EDGE_PRIORITY, // Do not conflict with agent at time t_from to t_to, setting other agent as specified in its path in the context.
    VERTEX_STATE_AVOIDANCE, // Do not conflict with agent at a specified configuration at time t.
    EDGE_STATE_AVOIDANCE, // Do not conflict with agent at a specified configuration between time t_from to t_to.
    ALL_EDGE_VERTEX_REQUEST, // Initially used in Generalized-CBS, this is a request to create vertex and edge constraints for all conflicts available. Used in conflictsToConstraints.
    SPHERE3DLARGE, // Do not be in sphere at time t, but with a larger radius.
    SPHERE3DXLARGE, // Do not be in sphere at time t, but with a much larger radius.
    PATH_PRIORITY, // Do not be in sphere at time t, but with a much much larger radius.
    VERTEX_POINT3D, // Do not be at v at time t if the associated configuration collides with a point.
    EDGE_POINT3D, // Do not cross edge (u, v) at time t to t+1 if the associated configuration interpolation collides with a point.
};

// A map from constraint type to whether it is admissible or not.
const std::unordered_map<ConstraintType, bool> constraint_type_admissibility = {
    {ConstraintType::UNSET,                  false},
    {ConstraintType::VERTEX,                 true},
    {ConstraintType::EDGE,                   true},
    {ConstraintType::VERTEX_SPHERE3D,        false},
    {ConstraintType::VERTEX_PRIORITY,        false},
    {ConstraintType::EDGE_PRIORITY,          false},
    {ConstraintType::VERTEX_STATE_AVOIDANCE, false},
    {ConstraintType::EDGE_STATE_AVOIDANCE,   false},
};

/// @brief Base class for all search constraints.
struct Constraint {
    /// @brief Constructor
    explicit Constraint() = default;

    /// @brief Virtual destructor.
    virtual ~Constraint() = default;

    /// @brief Get a string with information about the constraint.
    virtual std::string toString() const = 0;

    /// @brief The time of the constraint.
    /// @note This may be changed to a double for continuous time scenarios.
    virtual std::pair<TimeType, TimeType> getTimeInterval() const = 0;

    /// @brief The type of the constraint.
    ConstraintType type;
};

// ==========================
// Constraints used by CBS.
// ==========================

struct VertexConstraint : public Constraint {
    /// @brief The state vector. Could be a robot configuration.
    // We specify the states directly since their ID may change in future low-level plan iterations.
    StateType state;

    /// @brief Constructor, allowing to set the state, time, and type.
    /// @param state The state vector.
    explicit VertexConstraint(StateType state) : state(std::move(state)) {
        /// @brief The type of the constraint.
        type = ConstraintType::VERTEX;
    }

    /// @brief String representation of the constraint.
    /// @return The string representation.
    std::string toString() const override {
        std::stringstream ss;
        ss << "VertexConstraint: "
           << " (" ;
        for (int i = 0; i < state.size() - 1; i++) {
            ss << state[i] << ", ";
        }
        ss << state.back() << ")";
        return ss.str();
    }

    /// @brief The time interval of the constraint.
    std::pair<int, int> getTimeInterval() const override {
        return std::make_pair(state.back(), state.back());
    }
};

struct EdgeConstraint : public Constraint {
    /// @brief The state vector. Could be a robot configuration.
    // We specify the states directly since their ID may change in future low-level plan iterations.
    StateType state_from;
    StateType state_to;

    /// @brief Constructor, allowing to set the state, time, and type.
    /// @param state The state vector.
    explicit EdgeConstraint(StateType state_from, StateType state_to) : state_from(std::move(state_from)), state_to(std::move(state_to)) {
        /// @brief The type of the constraint.
        type = ConstraintType::EDGE;
    }
    
    std::string toString() const override {
        std::stringstream ss;
        ss << "EdgeConstraint. From: (";
        for (int i = 0; i < state_from.size() - 1; i++) {
            ss << state_from[i] << ", ";
        }
        ss << state_from.back() << ") To: (";
        for (int i = 0; i < state_to.size() - 1; i++) {
            ss << state_to[i] << ", ";
        }
        ss << state_to.back() << ")";
        return ss.str();
    }

    /// @brief The time interval of the constraint.
    std::pair<int, int> getTimeInterval() const override {
        return std::make_pair(state_from.back(), state_to.back());
    }
};

// ==========================
// Constraints used by CBS-MRAMP.
// ==========================
struct VertexSphere3dConstraint : public Constraint {
    /// @brief The center of the constrained sphere.
    Eigen::Vector3d center;

    /// @brief The radius of the sphere.
    double radius;

    /// @brief The time of the constraint.
    TimeType time;

    /// @brief Constructor, allowing to set the state, time, and type.
    /// @param state The state vector.
    explicit VertexSphere3dConstraint(Eigen::Vector3d center, double radius, TimeType time) : center(std::move(center)), radius(radius), time(time) {
        /// @brief The type of the constraint.
        type = ConstraintType::VERTEX_SPHERE3D;
    }
    
    std::string toString() const override {
        std::stringstream ss;
        ss << "VertexSphere3dConstraint. Center: (" << center.x() << ", " << center.y() << ", " << center.z() << ") Radius: " << radius << " Time: " << time;
        return ss.str();
    }

    /// @brief The time interval of the constraint.
    std::pair<int, int> getTimeInterval() const override {
        return std::make_pair(time, time);
    }
};

struct EdgeSphere3dConstraint : public Constraint {
    /// @brief The center of the constrained sphere.
    Eigen::Vector3d center;

    /// @brief The radius of the sphere.
    double radius;

    /// @brief The time of the constraint. Inclusive in both from and to. We do, however, normally assume that time from is valid.
    TimeType time_from;
    TimeType time_to;

    /// @brief Constructor, allowing to set the state, time, and type.
    /// @param state The state vector.
    explicit EdgeSphere3dConstraint(Eigen::Vector3d center, double radius, TimeType time_from, TimeType time_to) :
                                            center(std::move(center)),
                                            radius(radius),
                                            time_from(time_from),
                                            time_to(time_to){
        /// @brief The type of the constraint.
        type = ConstraintType::EDGE_SPHERE3D;
    }

    std::string toString() const override {
        std::stringstream ss;
        ss << "EdgeSphere3dConstraint. Center: (" << center.x() << ", " << center.y() << ", " << center.z() << ") Radius: " << radius << " Time: [" <<  time_from << ", " << time_to << "]";
        return ss.str();
    }

    /// @brief The time interval of the constraint.
    std::pair<int, int> getTimeInterval() const override {
        return std::make_pair(time_from, time_to);
    }
};

struct Sphere3dLargeConstraint : public VertexSphere3dConstraint {

    /// @brief Constructor, allowing to set the state, time, and type.
    /// @param center The center of the constrained sphere.
    /// @param radius The radius of the sphere.
    /// @param time The time of the constraint.
    explicit Sphere3dLargeConstraint(Eigen::Vector3d center, double radius, TimeType time) :
            VertexSphere3dConstraint(std::move(center), radius, time) {
        /// @brief The type of the constraint.
        type = ConstraintType::SPHERE3DLARGE;
    }
};

struct Sphere3dXLargeConstraint : public VertexSphere3dConstraint {

    /// @brief Constructor, allowing to set the state, time, and type.
    /// @param center The center of the constrained sphere.
    /// @param radius The radius of the sphere.
    /// @param time The time of the constraint.
    explicit Sphere3dXLargeConstraint(Eigen::Vector3d center, double radius, TimeType time) :
            VertexSphere3dConstraint(std::move(center), radius, time) {
        /// @brief The type of the constraint.
        type = ConstraintType::SPHERE3DXLARGE;
    }
};

// ==========================
// Constraints used by Prioritized Planning.
// ==========================
struct VertexPriorityConstraint : public Constraint {

    /// @brief The time of the constraint. If the time is -1, then the constraint is applied at all times.
    TimeType time;

    /// @brief the names of the agents to avoid.
    std::vector<std::string> agent_names_to_avoid;
    std::vector<int> agent_ids_to_avoid;

    /// @brief Constructor, allowing to set the agent ids to avoid and when. An overload allows also setting the agent names.
    /// @param agent_ids_to_avoid The agent ids to avoid.
    /// @param time The time of the constraint.
    explicit VertexPriorityConstraint(std::vector<int> agent_ids_to_avoid, TimeType time) :
        agent_ids_to_avoid(agent_ids_to_avoid), 
        time(time) {

        /// @brief The type of the constraint.
        type = ConstraintType::VERTEX_PRIORITY;
    }
    explicit VertexPriorityConstraint(int agent_id_to_avoid, TimeType time) : 
        time(time) {
        agent_ids_to_avoid = {agent_id_to_avoid};

        /// @brief The type of the constraint.
        type = ConstraintType::VERTEX_PRIORITY;
    }

    /// @brief Constructor, allowing to set the agent ids to avoid, time, and agent names.
    /// @param state The state vector.
    explicit VertexPriorityConstraint(std::vector<int> agent_ids_to_avoid, TimeType time, std::vector<std::string> agent_names_to_avoid) :
        agent_ids_to_avoid(agent_ids_to_avoid), 
        time(time), 
        agent_names_to_avoid(agent_names_to_avoid) {

        /// @brief The type of the constraint.
        type = ConstraintType::VERTEX_PRIORITY;
    }
    explicit VertexPriorityConstraint(int agent_id_to_avoid, TimeType time, std::string agent_name_to_avoid) :
        time(time) {
        agent_ids_to_avoid = {agent_id_to_avoid};
        agent_names_to_avoid = {agent_name_to_avoid};
        
        /// @brief The type of the constraint.
        type = ConstraintType::VERTEX_PRIORITY;
    }

    /// @brief String representation of the constraint.
    /// @return The string representation.
    std::string toString() const override {
        std::stringstream ss;
        ss << "VertexPriorityConstraint: avoid agents: (";
        for (int i = 0; i < agent_ids_to_avoid.size() - 1; i++) {
            ss << agent_ids_to_avoid[i];
            if (i < agent_names_to_avoid.size() - 1) {
                ss << " (" << agent_names_to_avoid[i] << "), ";
            }
            else {
                ss << ", ";
            }
        }
        ss << agent_ids_to_avoid.back() << ") at time: " << time;
        return ss.str();
    }

    /// @brief The time interval of the constraint.
    std::pair<TimeType, TimeType> getTimeInterval() const override {
        return std::make_pair(time, time);
    }
};

struct EdgePriorityConstraint : public Constraint {
    /// @brief The time interval of the constraint.
    TimeType t_from;
    TimeType t_to;

    /// @brief the names of the agents to avoid.
    std::vector<std::string> agent_names_to_avoid;

    /// @brief the ids of the agents to avoid.
    std::vector<int> agent_ids_to_avoid;

    /// @brief Constructor, allowing to set the agent ids to avoid and when.
    /// @param agent_ids_to_avoid The agent ids to avoid.
    /// @param t_from The time to start avoiding the agents.
    /// @param t_to T        // TEST TEST TEST!!!
//        EdgePriorityConstraint constraint = EdgePriorityConstraint(agent_ids_to_avoid, -1, -1);
        // END TEST TEST TEST!!!he time to stop avoiding the agents.
    explicit EdgePriorityConstraint(int agent_id_to_avoid, TimeType t_from, TimeType t_to, std::string agent_name_to_avoid = "") :
        t_from(t_from),
        t_to(t_to),
        agent_ids_to_avoid({agent_id_to_avoid}),
        agent_names_to_avoid({agent_name_to_avoid}) {
        type = ConstraintType::EDGE_PRIORITY;

        // Check that there is exactly one timestep between `t_from` and `t_to`.
        if (t_to - t_from != 1 && (t_to != -1 && t_from != -1)) {
            throw std::invalid_argument("EdgePriorityConstraint: t_to - t_from != 1. Currently only single-timestep-edges are supported.");
        }
    }

    /// @brief Constructor, allowing to set the agent ids to avoid and when.
    /// @param agent_ids_to_avoid The agent ids to avoid.
    /// @param t_from The time to start avoiding the agents.
    /// @param t_to The time to stop avoiding the agents.
    explicit EdgePriorityConstraint(std::vector<int> agent_ids_to_avoid, TimeType t_from, TimeType t_to, std::vector<std::string> agent_names_to_avoid = {}) :
        t_from(t_from),
        t_to(t_to),
        agent_ids_to_avoid(agent_ids_to_avoid),
        agent_names_to_avoid(agent_names_to_avoid) {
        type = ConstraintType::EDGE_PRIORITY;

        // Check that there is exactly one timestep between `t_from` and `t_to`.
        if (t_to - t_from != 1 && (t_to != -1 && t_from != -1)) {
            throw std::invalid_argument("EdgePriorityConstraint: t_to - t_from != 1. Currently only single-timestep-edges are supported.");
        }
    }

    /// @brief String representation of the constraint.
    /// @return The string representation.
    std::string toString() const override {
        std::stringstream ss;
        ss << "EdgePriorityConstraint. Avoiding agent ids: (";
        for (int i = 0; i < agent_ids_to_avoid.size() - 1; i++) {
            ss << agent_ids_to_avoid[i];
            if (i < agent_names_to_avoid.size() - 1) {
                ss << " (" << agent_names_to_avoid[i] << "), ";
            }
            else {
                ss << ", ";
            }
        }
        ss << agent_ids_to_avoid.back() << ") from time: " << t_from << " to time: " << t_to;
        return ss.str();
    }

    /// @brief The time interval of the constraint.
    std::pair<int, int> getTimeInterval() const override {
        return std::make_pair(t_from, t_to);
    }
};

struct PathPriorityConstraint : public VertexPriorityConstraint {
    /// @brief Constructor, allowing to set the agent ids to avoid and when.
    /// @param agent_ids_to_avoid The agent ids to avoid.
    /// @param t_from The time to start avoiding the agents.
    /// @param t_to The time to stop avoiding the agents.
    explicit PathPriorityConstraint(int agent_id_to_avoid, std::string agent_name_to_avoid = "") :
        VertexPriorityConstraint(agent_id_to_avoid, -1, agent_name_to_avoid) {
        type = ConstraintType::PATH_PRIORITY;
    }

    /// @brief Constructor, allowing to set the agent ids to avoid and when.
    /// @param agent_ids_to_avoid The agent ids to avoid.
    /// @param t_from The time to start avoiding the agents.
    /// @param t_to The time to stop avoiding the agents.
    explicit PathPriorityConstraint(std::vector<int> agent_ids_to_avoid, std::vector<std::string> agent_names_to_avoid = {}) :
        VertexPriorityConstraint(agent_ids_to_avoid, -1, agent_names_to_avoid) {
        type = ConstraintType::PATH_PRIORITY;
    }
};


// ==========================
// Constraints used by CBS-MP.
// ==========================
struct VertexStateAvoidanceConstraint : public Constraint {

    /// @brief the names of the agents to avoid.
    std::vector<int> agent_ids_to_avoid;
    std::vector<StateType> agent_states_to_avoid;

    /// @brief Constructor, allowing to set the agent ids to avoid and when. An overload allows also setting the agent names.
    /// @param agent_ids_to_avoid The agent ids to avoid.
    explicit VertexStateAvoidanceConstraint(std::vector<int> agent_ids_to_avoid, std::vector<StateType> agent_states_to_avoid) : 
        agent_ids_to_avoid(agent_ids_to_avoid), 
        agent_states_to_avoid(agent_states_to_avoid) {

        TimeType t_from = std::numeric_limits<TimeType>::max();
        TimeType t_to = std::numeric_limits<TimeType>::min();
        for (const auto& state : agent_states_to_avoid) {
            t_from = std::min(t_from, (TimeType)state.back());
            t_to   = std::max(t_to,     (TimeType)state.back());
        }          
        assert (t_from == t_to);  

        /// @brief The type of the constraint.
        type = ConstraintType::VERTEX_STATE_AVOIDANCE;
    }
    explicit VertexStateAvoidanceConstraint(int agent_id_to_avoid, std::vector<StateType> agent_states_to_avoid) : 
        agent_states_to_avoid(agent_states_to_avoid) {
        agent_ids_to_avoid = {agent_id_to_avoid};

        TimeType t_from = std::numeric_limits<TimeType>::max();
        TimeType t_to = std::numeric_limits<TimeType>::min();
        for (const auto& state : agent_states_to_avoid) {
            t_from = std::min(t_from, (TimeType)state.back());
            t_to   = std::max(t_to,     (TimeType)state.back());
        }          
        assert (t_from == t_to);  

        /// @brief The type of the constraint.
        type = ConstraintType::VERTEX_STATE_AVOIDANCE;
    }

    /// @brief String representation of the constraint.
    /// @return The string representation.
    std::string toString() const override {
        std::stringstream ss;
        ss << "VertexStateAvoidanceConstraint: avoid agents: (";
        for (int i = 0; i < agent_ids_to_avoid.size(); i++) {
            ss << agent_ids_to_avoid[i] << " at state ";
            for (int j = 0; j < agent_states_to_avoid[i].size() - 1; j++) {
                ss << agent_states_to_avoid[i][j] << ", ";
            }
            ss << agent_states_to_avoid[i].back() << ") ";
        }
        ss << ")";
            
        return ss.str();
    }

    /// @brief The time interval of the constraint.
    std::pair<TimeType, TimeType> getTimeInterval() const override {
        TimeType t_from = std::numeric_limits<TimeType>::max();
        TimeType t_to = std::numeric_limits<TimeType>::min();
        for (const auto& state : agent_states_to_avoid) {
            t_from = std::min(t_from, (TimeType)state.back());
            t_to   = std::max(t_to,     (TimeType)state.back());
        }    

        return std::make_pair(t_from, t_to);
    }
};

struct EdgeStateAvoidanceConstraint : public Constraint {

    /// @brief the ids of the agents to avoid. The agent names can be passed in the constraints context.
    std::vector<int> agent_ids_to_avoid;

    /// @brief The agent states to avoid.
    std::vector<StateType> agent_states_from;
    std::vector<StateType> agent_states_to;


    /// @brief Constructor, allowing to set the agent ids to avoid and when.
    /// @param agent_ids_to_avoid The agent ids to avoid.
    /// @param t_from The time to start avoiding the agents.
    /// @param t_to The time to stop avoiding the agents.
    explicit EdgeStateAvoidanceConstraint(int agent_id_to_avoid, std::vector<StateType> agent_states_from, std::vector<StateType> agent_states_to) :
        agent_states_from(agent_states_from),
        agent_states_to(agent_states_to),
        agent_ids_to_avoid({agent_id_to_avoid}){
            
        type = ConstraintType::EDGE_STATE_AVOIDANCE;

        TimeType t_from = agent_states_from.back().back();
        TimeType t_to = agent_states_to.back().back();

        // Check that all the times are equal.
        for (const auto& state : agent_states_from) {
            if (state.back() != t_from) {
                throw std::invalid_argument("EdgeStateAvoidanceConstraint: all the states must have the same time.");
            }
        }
        for (const auto& state : agent_states_to) {
            if (state.back() != t_to) {
                throw std::invalid_argument("EdgeStateAvoidanceConstraint: all the states must have the same time.");
            }
        }

        // Check that there is exactly one timestep between `t_from` and `t_to`.
        if (t_to - t_from != 1 && (t_to != -1 && t_from != -1)) {
            throw std::invalid_argument("EdgeStateAvoidanceConstraint: t_to - t_from != 1. Currently only single-timestep-edges are supported.");
        }
    }

    /// @brief Constructor, allowing to set the agent ids to avoid and when.
    /// @param agent_ids_to_avoid The agent ids to avoid.
    /// @param t_from The time to start avoiding the agents.
    /// @param t_to The time to stop avoiding the agents.
    explicit EdgeStateAvoidanceConstraint(std::vector<int> agent_ids_to_avoid, std::vector<StateType> agent_states_from, std::vector<StateType> agent_states_to, std::vector<std::string> agent_names_to_avoid = {}) :
        agent_states_from(agent_states_from),
        agent_states_to(agent_states_to),
        agent_ids_to_avoid(agent_ids_to_avoid) {
        type = ConstraintType::EDGE_STATE_AVOIDANCE;

        TimeType t_from = agent_states_from.back().back();
        TimeType t_to = agent_states_to.back().back();

        // Check that all the times are equal.
        for (const auto& state : agent_states_from) {
            if (state.back() != t_from) {
                throw std::invalid_argument("EdgeStateAvoidanceConstraint: all the states must have the same time.");
            }
        }
        for (const auto& state : agent_states_to) {
            if (state.back() != t_to) {
                throw std::invalid_argument("EdgeStateAvoidanceConstraint: all the states must have the same time.");
            }
        }

        // Check that there is exactly one timestep between `t_from` and `t_to`.
        if (t_to - t_from != 1 && (t_to != -1 && t_from != -1)) {
            throw std::invalid_argument("EdgeStateAvoidanceConstraint: t_to - t_from != 1. Currently only single-timestep-edges are supported.");
        }
    }

    /// @brief String representation of the constraint.
    /// @return The string representation.
    std::string toString() const override {
        std::stringstream ss;
        ss << "EdgeStateAvoidanceConstraint. Avoiding agent ids: (";
        for (int i = 0; i < agent_ids_to_avoid.size(); i++) {
            ss << agent_ids_to_avoid[i] << " between state ";
            for (int j = 0; j < agent_states_from[i].size() - 1; j++) {
                ss << agent_states_from[i][j] << ", ";
            }
            ss << agent_states_from[i].back() << ") and state ";
            for (int j = 0; j < agent_states_to[i].size() - 1; j++) {
                ss << agent_states_to[i][j] << ", ";
            }
            ss << agent_states_to[i].back() << ") ";
        }
        ss << ")";
            
        return ss.str();
    }

    /// @brief The time interval of the constraint.
    std::pair<int, int> getTimeInterval() const override {
        TimeType t_from = agent_states_from.back().back();
        TimeType t_to = agent_states_to.back().back();
        return std::make_pair(t_from, t_to);
    }
};

/// @brief Base class for all search constraint contexts.
/// @details This class is used to store context variables that are required for constraint checking and satisfaction. For example, in the case of PBS, the context for constraint satisfaction is the paths of all the agents whose paths were already found. This class is used to store these paths, and is passed to the ConstrainedActionSpace via the setConstraintsContext method.
struct ConstraintsContext {
    /// @brief Constructor
    ConstraintsContext() = default;
    ConstraintsContext(std::shared_ptr<ConstrainedActionSpace> action_space_ptr) : action_space_ptr(action_space_ptr) { }

    /// @brief Virtual destructor.
    virtual ~ConstraintsContext() = default;

    /// @brief Paths of other agents.
    MultiAgentPaths agent_paths = {};
    std::vector<std::string> agent_names = {};

    /// @brief A pointer to the associated action space.
    std::shared_ptr<ConstrainedActionSpace> action_space_ptr = nullptr;
};

/// @brief Base class for all search constraints.
/// @details This class is used to store a set of constraints, and a context for constraint satisfaction.
struct ConstraintsCollective {
    /// @brief Constructor
    ConstraintsCollective() = default;

    /// @brief Virtual destructor.
    virtual ~ConstraintsCollective() = default;

    // Setters.
    /// @brief Set the constraints.
    /// @param constraints The constraints to set.
    void setConstraints(const std::vector<std::shared_ptr<Constraint>>& constraints);

    void addConstraint(const std::shared_ptr<Constraint>& constraint);

    /// @brief Set the last time that has constraints. This time is often used to check if a planner is allowed to terminate (find a goal). If there are outstanding constraints, then the planner should not terminate. If we impose constraints that are "always active," then we need a way to tell planners "after this time t, keep satisfying the "always" constraints but you are allowed to find the goal." This is one way to do this.
    void setLastConstraintTimeToAtLeast(TimeType t);

    void addConstraints(const std::vector<std::shared_ptr<Constraint>>& constraints);

    /// @brief Set the constraints context.
    /// @param context The constraints context to set.
    void setContext(const std::shared_ptr<ConstraintsContext>& context);

    /// @brief Clear the constraints context.
    void clearContext() ;

    /// @brief Clear the constraints.
    void clear();

    // Getters.
    /// @brief Get the context pointer.
    /// @return The context pointer.
    std::shared_ptr<ConstraintsContext> getConstraintsContext() const ;
    std::shared_ptr<ConstraintsContext> getConstraintsContextNonConst();

    /// @brief Get the constraints.
    /// @return The constraints.
    const std::vector<std::shared_ptr<Constraint>>& getConstraints() const ;

    /// String.
    /// @brief Get a string with information about the constraint.
    std::string toString() const ;

    /// @brief Get the greatest time that is constrained.
    /// @return The greatest time that is constrained.
    int getLastConstraintTime() const;

    /// @brief Create safe intervals for a configuration state.
    /// @param state The state configuration without a time component.
    void createSafeIntervals(const StateType & state);

    /// @brief Update the safe intervals for a given state.
    /// @param state The state configuration.
    void updateSafeIntervals(const std::shared_ptr<Constraint>& constraint);

    /// @brief Get the safe intervals for a given state. If those do not exist then create them.
    /// @param state The state configuration. Should not include a time component.
    /// @return The safe intervals.
    void getOrCreateSafeIntervals(const StateType& state, std::vector<SafeIntervalType>& safe_intervals);

    int getNumSafeIntervals();

private:

    /// @brief Map from a timestep to a set of constraint pointers.
    std::unordered_map<TimeType, std::vector<std::shared_ptr<Constraint>>> time_to_constraints_ptrs_ = {};

    // Member Variables.
    /// @brief The type of the constraint.
    std::shared_ptr<ConstraintsContext> context_ptr_ = std::make_shared<ConstraintsContext>();

    /// @brief The type of the constraint.
    std::vector<std::shared_ptr<Constraint>> constraints_ptrs_ = {};

    /// @brief The last time that has constraints imposed on it.
    TimeType last_constraint_time_ = -1;

    /// @brief The safe intervals of states. We keep track of state configurations and not ids since ids may change between searches.
    std::unordered_map<StateType, std::vector<SafeIntervalType>, StateTypeHash> state_to_safe_intervals_ = {};
};

/// @brief An object for mapping [agent_ids] to a set of constraints collective.
using MultiAgentConstraintsCollective = std::unordered_map<int, ConstraintsCollective>;

}  // namespace ims

