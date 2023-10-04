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
 * \file   conflict_conversion.cpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   Sep 1 2023
*/

// Project includes.
#include <search/common/conflict_conversions.hpp>

namespace ims {
namespace conflict_conversions {

// void vertexConflictToVertexConstraints(const VertexConflict * vertex_conflict_ptr, 
//                                        std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints){
//     for (int agent_id : vertex_conflict_ptr->agent_ids) {

//         // Create a new vertex constraint.
//         VertexConstraint constraint = VertexConstraint(vertex_conflict_ptr->state);

//         // Update the constraints collective to also include the new constraint.
//         agent_constraints.emplace_back(agent_id, std::vector<std::shared_ptr<Constraint>>{std::make_shared<VertexConstraint>(constraint)});
//     }
// }

// void edgeConflictToEdgeConstraints(const EdgeConflict * edge_conflict_ptr,
//                                         std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints){
//     // We have exactly two affected agents. Call them agent_a and agent_b. The conflict is 'a' moving 'state_from' to 'state_to' and 'b' moving 'state_to' to 'state_from', with time decremented and incremented by 1, respectively.
//     int agent_a = edge_conflict_ptr->agent_id_from;
//     int agent_b = edge_conflict_ptr->agent_id_to;

//     // Create a new edge constraint.
//     EdgeConstraint constraint_a = EdgeConstraint(edge_conflict_ptr->state_from, edge_conflict_ptr->state_to);
//     StateType state_from_b = edge_conflict_ptr->state_to;

//     // Decrement the time of the state_from_b.
//     state_from_b.back() -= 1;
//     StateType state_to_b = edge_conflict_ptr->state_from;

//     // Increment the time of the state_to_b.
//     state_to_b.back() += 1;
//     EdgeConstraint constraint_b = EdgeConstraint(state_from_b, state_to_b);

//     // Add to the constraints object.
//     agent_constraints.emplace_back(agent_a, std::vector<std::shared_ptr<ims::Constraint>>{std::make_shared<EdgeConstraint>(constraint_a)});
//     agent_constraints.emplace_back(agent_b, std::vector<std::shared_ptr<ims::Constraint>>{std::make_shared<EdgeConstraint>(constraint_b)});
// }

void vertexConflictToVertexConstraints(const VertexConflict * private_grids_vertex_conflict_ptr, 
                                       std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints){

    // For each affected agent (2, in ECBS), create a new constraint, and down the line a search state for each as well.
    for (int i = 0; i < private_grids_vertex_conflict_ptr->agent_ids.size(); i++) {
        int agent_id = private_grids_vertex_conflict_ptr->agent_ids[i];

        // Create a new vertex constraint.
        VertexConstraint constraint = VertexConstraint(private_grids_vertex_conflict_ptr->states[i]);

        // Update the constraints collective to also include the new constraint.
        agent_constraints.emplace_back(agent_id, std::vector<std::shared_ptr<ims::Constraint>>{std::make_shared<VertexConstraint>(constraint)});
    }
}


void edgeConflictToEdgeConstraints(const EdgeConflict * private_grids_edge_conflict_ptr,
                                        std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints){
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

void privateGridsVertexConflictToVertexConstraints(const PrivateGridsVertexConflict * private_grids_vertex_conflict_ptr, 
                                       std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints){
    // For each affected agent (2, in ECBS), create a new constraint, and down the line a search state for each as well.
    for (int i = 0; i < private_grids_vertex_conflict_ptr->agent_ids.size(); i++) {
        int agent_id = private_grids_vertex_conflict_ptr->agent_ids[i];

        // Create a new vertex constraint.
        VertexConstraint constraint = VertexConstraint(private_grids_vertex_conflict_ptr->states[i]);

        // Update the constraints collective to also include the new constraint.
        agent_constraints.emplace_back(agent_id, std::vector<std::shared_ptr<ims::Constraint>>{std::make_shared<VertexConstraint>(constraint)});
    }
}

void privateGridsEdgeConflictToEdgeConstraints(const PrivateGridsEdgeConflict * private_grids_edge_conflict_ptr,
                                        std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints){
    // We have two or more affected agents. For example say we have two and call them agent_a and agent_b. The conflict is 'a' moving 'from_state' to 'to_state' and 'b' moving from its own 'from_state' to 'to_state', each on their own private grid.
    for (int i = 0; i < private_grids_edge_conflict_ptr->agent_ids.size(); i++) {
        int agent_id = private_grids_edge_conflict_ptr->agent_ids[i];
        StateType from_state = private_grids_edge_conflict_ptr->from_states[i];
        StateType to_state = private_grids_edge_conflict_ptr->to_states[i];

        // It could be that one of the agents is not in transition while the other one is, so ceate a new edge constraint only if the states are different.
        if (from_state != to_state) {
            EdgeConstraint constraint = EdgeConstraint(from_state, to_state);

            // Update the constraints collective to also include the new constraint.
            agent_constraints.emplace_back(agent_id, std::vector<std::shared_ptr<ims::Constraint>>{std::make_shared<EdgeConstraint>(constraint)});
        }

        // Otherwise, create a new vertex constraint.
        else {
            VertexConstraint constraint = VertexConstraint(from_state);

            // Update the constraints collective to also include the new constraint.
            agent_constraints.emplace_back(agent_id, std::vector<std::shared_ptr<ims::Constraint>>{std::make_shared<VertexConstraint>(constraint)});
        }
    }
}

void point3dConflictToSphere3dConstraints(const Point3dConflict * point3d_conflict_ptr,
std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints, double sphere3d_constraint_radius){

    // Decide if this point3d conflict should yield a vertex (sphere3d constraint on a single timestep) or an edge constraint (sphere3d constraint on a time interval).
    if (point3d_conflict_ptr->to_states.empty()) {
        // Assert that the time is integral.
        assert(point3d_conflict_ptr->from_states.back().back() == std::round(point3d_conflict_ptr->from_states.back().back()));

        // Create a new vertex constraint for each of the agents. Point3DConflict assumes private grids, so each agent has its own state.
        for (int i = 0; i < point3d_conflict_ptr->agent_ids.size(); i++) {
            int agent_id = point3d_conflict_ptr->agent_ids[i];
            TimeType time = point3d_conflict_ptr->from_states.front().back();

            // Create a new sphere3d constraint for this timestep.
            Sphere3dConstraint constraint = Sphere3dConstraint(point3d_conflict_ptr->point, sphere3d_constraint_radius, time);

            // Update the constraints collective to also include the new constraint.
            agent_constraints.emplace_back(agent_id, std::vector<std::shared_ptr<ims::Constraint>>{std::make_shared<Sphere3dConstraint>(constraint)});
        }
    }

    // Otherwise, if we have to_states, then the conflict was found in a transition between states and will yield an "edge constraint". We will emulate an edge constraint here by adding a sphere constraint at the integral time steps before and after the conflict.
    else {
        // Create a new edge constraint for each of the agents. Point3DConflict assumes private grids, so each agent has its own state.
        for (int i = 0; i < point3d_conflict_ptr->agent_ids.size(); i++) {
            int agent_id = point3d_conflict_ptr->agent_ids[i];

            TimeType time_from = std::round(point3d_conflict_ptr->from_states[i].back());
            TimeType time_to = time_from + 1; // TODO(yoraish): this +1 should come from a discretization object. It is within the action type that may not be available to the actionspace object, which the planner holds.

            // Create a new sphere3d constraint for each of the two timesteps.
            Sphere3dConstraint constraint_from = Sphere3dConstraint(point3d_conflict_ptr->point, sphere3d_constraint_radius, time_from);
            Sphere3dConstraint constraint_to = Sphere3dConstraint(point3d_conflict_ptr->point, sphere3d_constraint_radius, time_to);

            // Update the constraints collective to also include the new constraint.
            // TODO(yoraish): the bottom two constraints MUST be added together. We should change this entire function to return pairs of {agent_it: {constraint0, constraint1, ...}}. Accordingly, in the expand() method of any CBS variant, accept these sets and put all constraints within a set in a new CT node, as they are created.
            agent_constraints.emplace_back(agent_id, std::vector<std::shared_ptr<ims::Constraint>>{std::make_shared<Sphere3dConstraint>(constraint_from), std::make_shared<Sphere3dConstraint>(constraint_to)});
        }
    }
}

}  // namespace conflict_conversions
}  // namespace ims
