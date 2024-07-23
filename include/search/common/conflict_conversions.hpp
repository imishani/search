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
 * \file   conflict_conversion.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   Sep 1 2023
*/

#ifndef SEARCH_COMMON_CONFLICT_CONVERSIONS_HPP
#define SEARCH_COMMON_CONFLICT_CONVERSIONS_HPP

// General includes.
#include <iostream>

// Project includes.
#include <search/common/conflicts.hpp>
#include <search/common/constraints.hpp>
#include <search/common/utils.hpp>

namespace ims {
namespace conflict_conversions {
/// A container for the conversion context. This holds information that may be needed during the conversion process.
struct ConflictConversionContext {
    /// The agent names.
    std::vector<std::string> agent_names;
    /// The sphere3d constraint radius.
    double sphere3d_constraint_radius = 0.0;

    // Getter methods.
    std::vector<std::string> getAgentNames() const {
        if(agent_names.empty()){
            throw std::runtime_error("Agent names not set.");
        }
        return agent_names;
    }

    double getSphere3dConstraintRadius() const {
        if(sphere3d_constraint_radius <= 0.0){
            throw std::runtime_error("Sphere3d constraint radius not set.");
        }
        return sphere3d_constraint_radius;
    }
};

class ConflictsToConstraintsConverter {
public:
    ConflictsToConstraintsConverter() = default;
    virtual ~ConflictsToConstraintsConverter() = default;

    // Functions.
    /// @brief Convert a conflict to a set of constraints. If a conflict is between agents 2 and 4, then most often these
    /// functions will create one constraint for agent 2 and one constraint for agent 4.
    /// \param conflict_ptr
    /// \param constraint_type
    /// \param agent_constraints
    void convertConflictToConstraints(const std::shared_ptr<Conflict> &  conflict_ptr,
                                      const ims::ConstraintType &constraint_type,
                                      std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints);
    void setContext(const ConflictConversionContext &context) { context_ = context; }

    // Conversion methods. Those are static and can be called without an instance of the class.
    /// @brief Convert a vertex conflict to a vertex constraint.
    /// @param conflict_ptr
    /// @return
    /// @note This function adds pairs of <agent_id, constraints> to the agent_constraints vector.
    static void vertexConflictToVertexConstraints(const VertexConflict * vertex_conflict_ptr,
                                           std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints);

    /// @brief Convert a edge conflict to an edge constraint.
    /// @param conflict_ptr
    /// @return
    /// @note This function adds pairs of <agent_id, constraints> to the agent_constraints vector.
    static void edgeConflictToEdgeConstraints(const EdgeConflict * edge_conflict_ptr,
                                            std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints);

    /// @brief Convert a vertex conflict to a vertex constraint.
    /// @param conflict_ptr
    /// @return
    /// @note This function adds pairs of <agent_id, constraints> to the agent_constraints vector.
    static void vertexConflictToVertexStateAvoidanceConstraints(const VertexConflict * vertex_conflict_ptr,
                                           std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints);

    /// @brief Convert an edge point3d conflict to an edge state-avoidance constraint.
    /// @param point3d_conflict_ptr
    /// @param agent_constraints
    static void point3dEdgeConflictToEdgeStateAvoidanceConstraints(const Point3dEdgeConflict * point3d_conflict_ptr,
                                            std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints);

    /// @brief Convert a vertex point3d conflict to a vertex state-avoidance constraint. "Avoid an agent at this configuration at this time."
    /// @param point3d_conflict_ptr
    /// @param agent_constraints
    static void point3dVertexConflictToVertexStateAvoidanceConstraints(const Point3dVertexConflict * point3d_conflict_ptr,
                                            std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints);

    /// @brief Convert andge point3d conflict to an edge avoidance constraint. "Avoid an agent at this time."
    /// @param point3d_conflict_ptr
    /// @param agent_constraints
    static void point3dEdgeConflictToEdgePriorityConstraints(const Point3dEdgeConflict * point3d_conflict_ptr,
                                            const std::vector<std::string>& agent_names_to_avoid,
                                            std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints);
    static void point3dEdgeConflictToPathPriorityConstraints(const Point3dEdgeConflict * point3d_conflict_ptr,
                                            const std::vector<std::string>& agent_names_to_avoid,
                                            std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints);

    /// @brief Convert a vertex point3d conflict to a vertex avoidance constraint.
    /// @param point3d_conflict_ptr
    /// @param agent_constraints
    static void point3dVertexConflictToVertexPriorityConstraints(const Point3dVertexConflict * point3d_conflict_ptr,
                                            const std::vector<std::string>& agent_names_to_avoid,
                                            std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints);
    static void point3dVertexConflictToPathPriorityConstraints(const Point3dVertexConflict * point3d_conflict_ptr,
                                            const std::vector<std::string>& agent_names_to_avoid,
                                            std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints);

    /// @brief Convert a edge conflict to an edge constraint.
    /// @param conflict_ptr
    /// @return
    /// @note This function adds pairs of <agent_id, constraints> to the agent_constraints vector.
    static void edgeConflictToEdgeStateAvoidanceConstraints(const EdgeConflict * edge_conflict_ptr,
                                            std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints);

    /// @brief Convert a point3d conflict to a sphere3d constraint.
    /// @param point3d_conflict_ptr
    /// @return
    /// @note This function adds pairs of <agent_id, constraints> to the agent_constraints vector.
    static void point3dVertexConflictToSphere3dVertexConstraints(const Point3dVertexConflict * point3d_conflict_ptr,
        std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints, double sphere3d_constraint_radius);
    static void point3dVertexConflictToSphere3dLargeConstraints(const Point3dVertexConflict * point3d_conflict_ptr,
        std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints, double sphere3d_constraint_radius);
    static void point3dVertexConflictToSphere3dXLargeConstraints(const Point3dVertexConflict * point3d_conflict_ptr,
        std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints, double sphere3d_constraint_radius);
    /// @brief Convert a point3d conflict to a sphere3d constraint.
    /// @param point3d_conflict_ptr
    /// @return
    /// @note This function adds pairs of <agent_id, constraints> to the agent_constraints vector.
    static void point3dEdgeConflictToSphere3dEdgeConstraints(const Point3dEdgeConflict * point3d_conflict_ptr,
                                                               std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints, double sphere3d_constraint_radius);
    static void point3dEdgeConflictToSphere3dLargeConstraints(const Point3dEdgeConflict * point3d_conflict_ptr,
        std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints, double sphere3d_constraint_radius);
    static void point3dEdgeConflictToSphere3dXLargeConstraints(const Point3dEdgeConflict * point3d_conflict_ptr,
        std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints, double sphere3d_constraint_radius);
    /// @brief Convert a point3d conflict to edge or vertex constraints.
    /// @param point3d_conflict_ptr
    /// @return
    /// @note This function adds pairs of <agent_id, constraints> to the agent_constraints vector.
    static void point3dVertexConflictToVertexConstraints(const Point3dVertexConflict * point3d_conflict_ptr,
        std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints);

    /// @brief Convert a point3d conflict to edge or vertex constraints.
    /// @param point3d_conflict_ptr
    /// @return
    /// @note This function adds pairs of <agent_id, constraints> to the agent_constraints vector.
    static void point3dEdgeConflictToEdgeConstraints(const Point3dEdgeConflict * point3d_conflict_ptr,
        std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints);

    /// @brief Convert a vertex point3d conflict to a partial vertex constraint.
    /// @param point3d_conflict_ptr
    /// @return
    /// @note This function adds pairs of <agent_id, constraints> to the agent_constraints vector.
    static void point3dVertexConflictToPartialVertexConstraints(const Point3dVertexConflict * point3d_conflict_ptr,
        std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints);
    /// @brief Convert an edge point3d conflict to a partial edge constraint.
    /// @param point3d_conflict_ptr
    /// @return
    /// @note This function adds pairs of <agent_id, constraints> to the agent_constraints vector.
    static void point3dEdgeConflictToPartialEdgeConstraints(const Point3dEdgeConflict * point3d_conflict_ptr,
        std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints);


protected:
    // Variables.
    // Extra context that some conversions need.
    ConflictConversionContext context_;

    // Functions.
};

}  // namespace conflict_conversions
}  // namespace ims

#endif //SEARCH_COMMON_CONFLICT_CONVERSIONS_HPP
