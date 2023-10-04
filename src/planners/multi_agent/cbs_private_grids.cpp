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
 * \file   ecbs_private_grids.cpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   August 24 2023
 */

#include <search/planners/multi_agent/cbs_private_grids.hpp>

ims::CBSPrivateGrids::CBSPrivateGrids(const ims::CBSParams& params) : params_(params), ims::CBS(params) {}

std::vector<std::pair<int, std::vector<std::shared_ptr<ims::Constraint>>>> ims::CBSPrivateGrids::conflictsToConstraints(const std::vector<std::shared_ptr<ims::Conflict>>& conflicts) {
    std::vector<std::pair<int, std::vector<std::shared_ptr<ims::Constraint>>>> agent_constraints;
    // TODO(yoraish): this is WIP.
    // Iterate through the conflicts and convert them to constraints.
    for (auto& conflict_ptr : conflicts) {
        // Create a new constraint given the conflict.

        // ===========================
        // Vertex Conflicts.
        // ===========================
        if (conflict_ptr->type == ConflictType::VERTEX) {
            auto* vertex_conflict_ptr = dynamic_cast<VertexConflict*>(conflict_ptr.get());
            // Check if the conversion succeeded.
            if (vertex_conflict_ptr == nullptr) {
                throw std::runtime_error("Conflict is a vertex conflict, but could not be converted to a VertexConflict.");
            }
            ims::conflict_conversions::vertexConflictToVertexConstraints(vertex_conflict_ptr, agent_constraints);
        }

        // ===========================
        // Edge Conflicts.
        // ===========================
        // Otherwise, if the conflict is an edge conflict, add an edge constraint to each of the two affected agents.
        else if (conflict_ptr->type == ConflictType::EDGE) {
            auto* edge_conflict_ptr = dynamic_cast<EdgeConflict*>(conflict_ptr.get());

            // Check if the conversion succeeded.
            if (edge_conflict_ptr == nullptr) {
                throw std::runtime_error("Conflict is an edge conflict, but could not be converted to an EdgeConflict.");
            }
            ims::conflict_conversions::edgeConflictToEdgeConstraints(edge_conflict_ptr, agent_constraints);
        }

        // ===========================
        // Private Grids Vertex Conflicts.
        // ===========================
        else if (conflict_ptr->type == ConflictType::PRIVATE_GRIDS_VERTEX) {
            // Get the location of each of the agents. Each one is specified in its own grid.
            auto* private_grids_vertex_conflict_ptr = dynamic_cast<PrivateGridsVertexConflict*>(conflict_ptr.get());
            // Check if the conversion succeeded.
            if (private_grids_vertex_conflict_ptr == nullptr) {
                throw std::runtime_error("Conflict is a private grids vertex conflict, but could not be converted to a PrivateGridsVertexConflict.");
            }

            // For each affected agent (2, in CBS), create a new constraint, and down the line a search state for each as well.
            for (int i = 0; i < private_grids_vertex_conflict_ptr->agent_ids.size(); i++) {
                int agent_id = private_grids_vertex_conflict_ptr->agent_ids[i];

                // Create a new vertex constraint.
                VertexConstraint constraint = VertexConstraint(private_grids_vertex_conflict_ptr->states[i]);

                // Update the constraints collective to also include the new constraint.
                agent_constraints.emplace_back(agent_id, std::vector<std::shared_ptr<ims::Constraint>>{std::make_shared<VertexConstraint>(constraint)});
            }
        }

        // ===========================
        // Private Grids Edge Conflicts.
        // ===========================
        else if (conflict_ptr->type == ConflictType::PRIVATE_GRIDS_EDGE) {
            // Get the location of each of the agents. Each one is specified in its own grid.
            auto* private_grids_edge_conflict_ptr = dynamic_cast<PrivateGridsEdgeConflict*>(conflict_ptr.get());
            // Check if the conversion succeeded.
            if (private_grids_edge_conflict_ptr == nullptr) {
                throw std::runtime_error("Conflict is a private grids edge conflict, but could not be converted to a PrivateGridsEdgeConflict.");
            }

            // We have two or more affected agents. For example say we have two and call them agent_a and agent_b. The conflict is 'a' moving 'state_from' to 'state_to' and 'b' moving from its own 'state_from' to 'state_to', each on their own private grid.
            for (int i = 0; i < private_grids_edge_conflict_ptr->agent_ids.size(); i++) {
                int agent_id = private_grids_edge_conflict_ptr->agent_ids[i];
                StateType state_from = private_grids_edge_conflict_ptr->from_states[i];
                StateType state_to = private_grids_edge_conflict_ptr->to_states[i];

                // It could be that one of the agents is not in transition while the other one is, so ceate a new edge constraint only if the states are different.
                if (state_from != state_to) {
                    EdgeConstraint constraint = EdgeConstraint(state_from, state_to);

                    // Update the constraints collective to also include the new constraint.
                    agent_constraints.emplace_back(agent_id, std::vector<std::shared_ptr<ims::Constraint>>{std::make_shared<EdgeConstraint>(constraint)});
                }

                // Otherwise, create a new vertex constraint.
                else {
                    VertexConstraint constraint = VertexConstraint(state_from);

                    // Update the constraints collective to also include the new constraint.
                    agent_constraints.emplace_back(agent_id, std::vector<std::shared_ptr<ims::Constraint>>{std::make_shared<VertexConstraint>(constraint)});
                }
            }
        }
    }
    return agent_constraints;
}
