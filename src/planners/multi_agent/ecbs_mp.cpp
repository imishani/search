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
 * \file   ecbs_mp.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   October 19 2023
 */

#include <search/planners/wastar.hpp>
#include <search/planners/multi_agent/ecbs_mp.hpp>
#include <search/planners/multi_agent/ecbs.hpp>

ims::ECBSMP::ECBSMP(const ims::ECBSMPParams& params) : params_(params), ECBS(params) {
    open_ = new FocalAndAnchorQueueWrapper<SearchState, ECBSOpenCompare, ECBSFocalCompare>();
}

std::vector<std::pair<int, std::vector<std::shared_ptr<ims::Constraint>>>> ims::ECBSMP::conflictsToConstraints(const std::vector<std::shared_ptr<ims::Conflict>>& conflicts) {
    std::vector<std::pair<int, std::vector<std::shared_ptr<ims::Constraint>>>> agent_constraints;

    // Iterate through the conflicts and convert them to constraints.
    for (auto& conflict_ptr : conflicts) {
        // Create a new constraint given the conflict.
        if (conflict_ptr->type == ConflictType::VERTEX) {
            auto* vertex_conflict_ptr = dynamic_cast<VertexConflict*>(conflict_ptr.get());
            // Check if the conversion succeeded.
            if (vertex_conflict_ptr == nullptr) {
                throw std::runtime_error("Conflict is a vertex conflict, but could not be converted to a VertexConflict.");
            }

            // For each affected agent (2, in EACBS), create a new constraint, and a search state for each as well.
            ims::conflict_conversions::ConflictsToConstraintsConverter::vertexConflictToVertexStateAvoidanceConstraints(vertex_conflict_ptr, agent_constraints);
        }

        // Otherwise, if the conflict is an edge conflict, add an edge constraint to each of the two affected agents.
        else if (conflict_ptr->type == ConflictType::EDGE) {
            auto* edge_conflict_ptr = dynamic_cast<EdgeConflict*>(conflict_ptr.get());

            // Check if the conversion succeeded.
            if (edge_conflict_ptr == nullptr) {
                throw std::runtime_error("Conflict is an edge conflict, but could not be converted to an EdgeConflict.");
            }
            ims::conflict_conversions::ConflictsToConstraintsConverter::edgeConflictToEdgeStateAvoidanceConstraints(edge_conflict_ptr, agent_constraints);
        }

        else {
            throw std::runtime_error("Conflict type " + std::to_string(static_cast<int>(conflict_ptr->type)) + " is not supported by EACBS.");
        }
    }

    return agent_constraints;
}
