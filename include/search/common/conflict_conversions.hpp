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

namespace ims {
namespace conflict_conversions {
/// @brief Convert a vertex conflict to a vertex constraint.
/// @param conflict_ptr 
/// @return
/// @note This function adds pairs of <agent_id, constraints> to the agent_constraints vector.
void vertexConflictToVertexConstraints(const VertexConflict * vertex_conflict_ptr, 
                                       std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints);

/// @brief Convert a edge conflict to an edge constraint.
/// @param conflict_ptr
/// @return
/// @note This function adds pairs of <agent_id, constraints> to the agent_constraints vector.
void edgeConflictToEdgeConstraints(const EdgeConflict * edge_conflict_ptr,
                                        std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints);

/// @brief Convert a vertex conflict to a vertex constraint.
/// @param conflict_ptr 
/// @return
/// @note This function adds pairs of <agent_id, constraints> to the agent_constraints vector.
void vertexConflictToVertexStateAvoidanceConstraints(const VertexConflict * vertex_conflict_ptr, 
                                       std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints);

/// @brief Convert a edge conflict to an edge constraint.
/// @param conflict_ptr
/// @return
/// @note This function adds pairs of <agent_id, constraints> to the agent_constraints vector.
void edgeConflictToEdgeStateAvoidanceConstraints(const EdgeConflict * edge_conflict_ptr,
                                        std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints);


/// @brief Convert a point3d conflict to a sphere3d constraint.
/// @param private_grids_edge_conflict_ptr
/// @return
/// @note This function adds pairs of <agent_id, constraints> to the agent_constraints vector.
void point3dConflictToSphere3dConstraints(const Point3dConflict * point3d_conflict_ptr,
std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints, double sphere3d_constraint_radius);



}  // namespace conflict_conversions
}  // namespace ims

#endif //SEARCH_COMMON_CONFLICT_CONVERSIONS_HPP
