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

void vertexConflictToVertexStateAvoidanceConstraints(const VertexConflict * vertex_conflict_ptr, 
                                       std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints){

    // For each affected agent, create a new constraint, and down the line a search state for each as well.
    for (int i = 0; i < vertex_conflict_ptr->agent_ids.size(); i++) {
        int agent_id = vertex_conflict_ptr->agent_ids[i];

        // Create a new vertex constraint. The agent states are the other states that the agent should avoid. Their corresponding agent ids are also passed.
        std::vector<StateType> states_to_avoid;
        std::vector<int> agent_ids_to_avoid;
        for (int j = 0; j < vertex_conflict_ptr->agent_ids.size(); j++) {
            if (j != i) {
                states_to_avoid.push_back(vertex_conflict_ptr->states[j]);
                agent_ids_to_avoid.push_back(vertex_conflict_ptr->agent_ids[j]);
            }
        }

        VertexStateAvoidanceConstraint constraint = VertexStateAvoidanceConstraint(agent_ids_to_avoid, states_to_avoid);

        // Update the constraints collective to also include the new constraint.
        agent_constraints.emplace_back(agent_id, std::vector<std::shared_ptr<ims::Constraint>>{std::make_shared<VertexStateAvoidanceConstraint>(constraint)});
    }
}

void edgeConflictToEdgeStateAvoidanceConstraints(const EdgeConflict * edge_conflict_ptr,
                                        std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints){
    // We have two or more affected agents. For example say we have two and call them agent_a and agent_b. The conflict is 'a' moving 'state_from' to 'state_to' and 'b' moving from its own 'state_from' to 'state_to', each on their own private grid. The constraint imposed in this case is, for each agent, to avoid the transition of all other agents.
    for (int i = 0; i < edge_conflict_ptr->agent_ids.size(); i++) {
        int agent_id = edge_conflict_ptr->agent_ids[i];

        std::vector<StateType> states_to_avoid_from;
        std::vector<StateType> states_to_avoid_to;
        std::vector<int> agent_ids_to_avoid;
        for (int j = 0; j < edge_conflict_ptr->agent_ids.size(); j++) {
            if (j != i) {
                // Create a new edge constraint. The agent states are the other states that the agent should avoid. Their corresponding agent ids are also passed.
                states_to_avoid_from.push_back(edge_conflict_ptr->from_states[j]);
                states_to_avoid_to.push_back(edge_conflict_ptr->to_states[j]);
                agent_ids_to_avoid.push_back(edge_conflict_ptr->agent_ids[j]);
            }
        }

        EdgeStateAvoidanceConstraint constraint = EdgeStateAvoidanceConstraint(agent_ids_to_avoid, states_to_avoid_from, states_to_avoid_to);

        // Update the constraints collective to also include the new constraint.
        agent_constraints.emplace_back(agent_id, std::vector<std::shared_ptr<ims::Constraint>>{std::make_shared<EdgeStateAvoidanceConstraint>(constraint)});
    }
}

void point3dVertexConflictToVertexStateAvoidanceConstraints(const Point3dVertexConflict * point3d_conflict_ptr,
                                                std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints){

    // Create a new Vertex avoidance constraint for each of the agents. Point3dVertexConflict assumes private grids, so each agent has its own state.
    for (int i = 0; i < point3d_conflict_ptr->agent_ids.size(); i++) {
        int agent_id = point3d_conflict_ptr->agent_ids[i];

        // Create a new Vertex constraint. The agent states are the other states that the agent should avoid. Their corresponding agent ids are also passed.
        std::vector<StateType> states_to_avoid;
        std::vector<int> agent_ids_to_avoid;
        for (int j = 0; j < point3d_conflict_ptr->agent_ids.size(); j++) {
            if (j != i) {
                states_to_avoid.push_back(point3d_conflict_ptr->states[j]);
                agent_ids_to_avoid.push_back(point3d_conflict_ptr->agent_ids[j]);
            }
        }

        VertexStateAvoidanceConstraint constraint = VertexStateAvoidanceConstraint(agent_ids_to_avoid, states_to_avoid);

        // Update the constraints collective to also include the new constraint.
        agent_constraints.emplace_back(agent_id, std::vector<std::shared_ptr<ims::Constraint>>{std::make_shared<VertexStateAvoidanceConstraint>(constraint)});
    }
}

void point3dEdgeConflictToEdgeStateAvoidanceConstraints(const Point3dEdgeConflict * point3d_conflict_ptr,
                                                std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints){

    // Create a new edge avoidance constraint for each of the agents. Point3dEdgeConflict assumes private grids, so each agent has its own state.
    for (int i = 0; i < point3d_conflict_ptr->agent_ids.size(); i++) {
        int agent_id = point3d_conflict_ptr->agent_ids[i];

        // Create a new edge constraint. The agent states are the other states that the agent should avoid. Their corresponding agent ids are also passed.
        std::vector<StateType> states_to_avoid_from;
        std::vector<StateType> states_to_avoid_to;
        std::vector<int> agent_ids_to_avoid;
        for (int j = 0; j < point3d_conflict_ptr->agent_ids.size(); j++) {
            if (j != i) {
                states_to_avoid_from.push_back(point3d_conflict_ptr->from_states[j]);
                states_to_avoid_to.push_back(point3d_conflict_ptr->to_states[j]);
                agent_ids_to_avoid.push_back(point3d_conflict_ptr->agent_ids[j]);
            }
        }

        EdgeStateAvoidanceConstraint constraint = EdgeStateAvoidanceConstraint(agent_ids_to_avoid, states_to_avoid_from, states_to_avoid_to);

        // Update the constraints collective to also include the new constraint.
        agent_constraints.emplace_back(agent_id, std::vector<std::shared_ptr<ims::Constraint>>{std::make_shared<EdgeStateAvoidanceConstraint>(constraint)});
    }
}

void point3dVertexConflictToVertexPriorityConstraints(const Point3dVertexConflict * point3d_conflict_ptr,
                                                const std::vector<std::string>& agent_names,
                                                std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints){
    // Create a new Vertex avoidance constraint for each of the agents. Point3dVertexConflict assumes private grids, so each agent has its own state.
    for (int i = 0; i < point3d_conflict_ptr->agent_ids.size(); i++) {
        int agent_id = point3d_conflict_ptr->agent_ids[i];

        // Create a new Vertex constraint. The agent states are the other states that the agent should avoid. Their corresponding agent ids are also passed.
        TimeType time = (TimeType)point3d_conflict_ptr->states.front().back();
        // The agent names to avoid are the agent names corresponding to the agent ids to avoid.
        std::vector<int> agent_ids_to_avoid;
        std::vector<std::string> agent_names_to_avoid;
        for (int agent_id_to_avoid : point3d_conflict_ptr->agent_ids) {
            if (agent_id_to_avoid != agent_id) {
                agent_ids_to_avoid.push_back(agent_id_to_avoid);
                agent_names_to_avoid.push_back(agent_names[agent_id_to_avoid]);
            }
        }

        VertexPriorityConstraint constraint = VertexPriorityConstraint(agent_ids_to_avoid, time);
        constraint.agent_names_to_avoid = agent_names_to_avoid;

        // Update the constraints collective to also include the new constraint.
        agent_constraints.emplace_back(agent_id, std::vector<std::shared_ptr<ims::Constraint>>{std::make_shared<VertexPriorityConstraint>(constraint)});
    }
}

void point3dEdgeConflictToEdgePriorityConstraints(const Point3dEdgeConflict * point3d_conflict_ptr,
                                                const std::vector<std::string>& agent_names,
                                                std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints){

    // Create a new edge avoidance constraint for each of the agents. Point3dEdgeConflict assumes private grids, so each agent has its own state.
    for (int i = 0; i < point3d_conflict_ptr->agent_ids.size(); i++) {
        int agent_id = point3d_conflict_ptr->agent_ids[i];

        // Create a new edge constraint. The agent states are the other states that the agent should avoid. Their corresponding agent ids are also passed.
        TimeType time_from = (TimeType)std::round(point3d_conflict_ptr->from_states[i].back());
        TimeType time_to = (TimeType)std::round(point3d_conflict_ptr->to_states[i].back());

        // The agent names to avoid are the agent names corresponding to the agent ids to avoid.
        std::vector<int> agent_ids_to_avoid;
        std::vector<std::string> agent_names_to_avoid;
        for (int agent_id_to_avoid : point3d_conflict_ptr->agent_ids) {
            if (agent_id_to_avoid != agent_id) {
                agent_ids_to_avoid.push_back(agent_id_to_avoid);
                agent_names_to_avoid.push_back(agent_names[agent_id_to_avoid]);
            }
        }
        EdgePriorityConstraint constraint = EdgePriorityConstraint(agent_ids_to_avoid, time_from, time_to);
        constraint.agent_names_to_avoid = agent_names_to_avoid;

        // Update the constraints collective to also include the new constraint.
        agent_constraints.emplace_back(agent_id, std::vector<std::shared_ptr<ims::Constraint>>{std::make_shared<EdgePriorityConstraint>(constraint)});
    }
}

void point3dVertexConflictToSphere3dConstraints(const Point3dVertexConflict * point3d_conflict_ptr,
std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints, double sphere3d_constraint_radius){

    // Create a new vertex constraint for each of the agents. Point3dVertexConflict assumes private grids, so each agent has its own state.
    for (int i = 0; i < point3d_conflict_ptr->agent_ids.size(); i++) {
        int agent_id = point3d_conflict_ptr->agent_ids[i];
        auto time = (TimeType)point3d_conflict_ptr->states.front().back();

        // Create a new sphere3d constraint for this timestep.
        Sphere3dConstraint constraint = Sphere3dConstraint(point3d_conflict_ptr->point, sphere3d_constraint_radius, time);

        // Update the constraints collective to also include the new constraint.
        agent_constraints.emplace_back(agent_id, std::vector<std::shared_ptr<ims::Constraint>>{std::make_shared<Sphere3dConstraint>(constraint)});
    }
}

void point3dVertexConflictToSphere3dLargeConstraints(const Point3dVertexConflict * point3d_conflict_ptr,
std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints, double sphere3d_constraint_radius){

    // Create a new vertex constraint for each of the agents. Point3dVertexConflict assumes private grids, so each agent has its own state.
    for (int i = 0; i < point3d_conflict_ptr->agent_ids.size(); i++) {
        int agent_id = point3d_conflict_ptr->agent_ids[i];
        auto time = (TimeType)point3d_conflict_ptr->states.front().back();

        // Create a new sphere3d constraint for this timestep.
        Sphere3dLargeConstraint constraint = Sphere3dLargeConstraint(point3d_conflict_ptr->point, sphere3d_constraint_radius, time);

        // Update the constraints collective to also include the new constraint.
        agent_constraints.emplace_back(agent_id, std::vector<std::shared_ptr<ims::Constraint>>{std::make_shared<Sphere3dLargeConstraint>(constraint)});
    }
}
void point3dVertexConflictToSphere3dXLargeConstraints(const Point3dVertexConflict * point3d_conflict_ptr,
                                                     std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints, double sphere3d_constraint_radius){

    // Create a new vertex constraint for each of the agents. Point3dVertexConflict assumes private grids, so each agent has its own state.
    for (int i = 0; i < point3d_conflict_ptr->agent_ids.size(); i++) {
        int agent_id = point3d_conflict_ptr->agent_ids[i];
        auto time = (TimeType)point3d_conflict_ptr->states.front().back();

        // Create a new sphere3d constraint for this timestep.
        Sphere3dXLargeConstraint constraint = Sphere3dXLargeConstraint(point3d_conflict_ptr->point, sphere3d_constraint_radius, time);

        // Update the constraints collective to also include the new constraint.
        agent_constraints.emplace_back(agent_id, std::vector<std::shared_ptr<ims::Constraint>>{std::make_shared<Sphere3dXLargeConstraint>(constraint)});
    }
}

void point3dEdgeConflictToSphere3dConstraints(const Point3dEdgeConflict * point3d_conflict_ptr,
std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints, double sphere3d_constraint_radius){

    // Create a new edge constraint for each of the agents. Point3DConflict assumes private grids, so each agent has its own state.
    for (int i = 0; i < point3d_conflict_ptr->agent_ids.size(); i++) {
        int agent_id = point3d_conflict_ptr->agent_ids[i];

        auto time_from = (TimeType)std::round(point3d_conflict_ptr->from_states[i].back());
        TimeType time_to = time_from + 1;

        // Create a new sphere3d constraint for each of the two timesteps.
        Sphere3dConstraint constraint_from = Sphere3dConstraint(point3d_conflict_ptr->point, sphere3d_constraint_radius, time_from);
        Sphere3dConstraint constraint_to = Sphere3dConstraint(point3d_conflict_ptr->point, sphere3d_constraint_radius, time_to);

        // Update the constraints collective to also include the new constraint.
        // Notice that the two constraints are added together to one agent. 
        agent_constraints.emplace_back(agent_id, std::vector<std::shared_ptr<ims::Constraint>>{std::make_shared<Sphere3dConstraint>(constraint_from), std::make_shared<Sphere3dConstraint>(constraint_to)});
    }
}

void point3dEdgeConflictToSphere3dLargeConstraints(const Point3dEdgeConflict * point3d_conflict_ptr,
std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints, double sphere3d_constraint_radius){

    // Create a new edge constraint for each of the agents. Point3DConflict assumes private grids, so each agent has its own state.
    for (int i = 0; i < point3d_conflict_ptr->agent_ids.size(); i++) {
        int agent_id = point3d_conflict_ptr->agent_ids[i];

        auto time_from = (TimeType)std::round(point3d_conflict_ptr->from_states[i].back());
        TimeType time_to = time_from + 1;

        // Create a new sphere3d constraint for each of the two timesteps.
        Sphere3dLargeConstraint constraint_from = Sphere3dLargeConstraint(point3d_conflict_ptr->point, sphere3d_constraint_radius, time_from);
        Sphere3dLargeConstraint constraint_to = Sphere3dLargeConstraint(point3d_conflict_ptr->point, sphere3d_constraint_radius, time_to);

        // Update the constraints collective to also include the new constraint.
        // Notice that the two constraints are added together to one agent. 
        agent_constraints.emplace_back(agent_id, std::vector<std::shared_ptr<ims::Constraint>>{std::make_shared<Sphere3dLargeConstraint>(constraint_from), std::make_shared<Sphere3dLargeConstraint>(constraint_to)});
    }
}
void point3dEdgeConflictToSphere3dXLargeConstraints(const Point3dEdgeConflict * point3d_conflict_ptr,
                                                   std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints, double sphere3d_constraint_radius){

    // Create a new edge constraint for each of the agents. Point3DConflict assumes private grids, so each agent has its own state.
    for (int i = 0; i < point3d_conflict_ptr->agent_ids.size(); i++) {
        int agent_id = point3d_conflict_ptr->agent_ids[i];

        auto time_from = (TimeType)std::round(point3d_conflict_ptr->from_states[i].back());
        TimeType time_to = time_from + 1;

        // Create a new sphere3d constraint for each of the two timesteps.
        Sphere3dXLargeConstraint constraint_from = Sphere3dXLargeConstraint(point3d_conflict_ptr->point, sphere3d_constraint_radius, time_from);
        Sphere3dXLargeConstraint constraint_to = Sphere3dXLargeConstraint(point3d_conflict_ptr->point, sphere3d_constraint_radius, time_to);

        // Update the constraints collective to also include the new constraint.
        // Notice that the two constraints are added together to one agent.
        agent_constraints.emplace_back(agent_id, std::vector<std::shared_ptr<ims::Constraint>>{std::make_shared<Sphere3dXLargeConstraint>(constraint_from), std::make_shared<Sphere3dXLargeConstraint>(constraint_to)});
    }
}

void point3dVertexConflictToVertexConstraints(const Point3dVertexConflict * point3d_conflict_ptr, 
                                       std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints){
    // Assert that the time is integral.
    assert(point3d_conflict_ptr->states.back().back() == std::round(point3d_conflict_ptr->states.back().back()));

    // Create a new vertex constraint for each of the agents. Point3dVertexConflict assumes private grids, so each agent has its own state.
    for (int i = 0; i < point3d_conflict_ptr->agent_ids.size(); i++) {
        int agent_id = point3d_conflict_ptr->agent_ids[i];

        // Create a new vertex constraint.
        VertexConstraint constraint = VertexConstraint(point3d_conflict_ptr->states.at(i));

        // Update the constraints vector to also include the new constraint.
        agent_constraints.emplace_back(agent_id, std::vector<std::shared_ptr<ims::Constraint>>{std::make_shared<VertexConstraint>(constraint)});
    }
}

void point3dEdgeConflictToEdgeConstraints(const Point3dEdgeConflict * point3d_conflict_ptr, 
                                       std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints){
    // Create a new edge constraint for each of the agents. Point3dEdgeConflict assumes private grids, so each agent has its own state.
    for (int i = 0; i < point3d_conflict_ptr->agent_ids.size(); i++) {
        int agent_id = point3d_conflict_ptr->agent_ids[i];

        // Create a new edge constraint.
        EdgeConstraint constraint = EdgeConstraint(point3d_conflict_ptr->from_states[i], point3d_conflict_ptr->to_states[i]);

        // Update the constraints vector to also include the new constraint.
        agent_constraints.emplace_back(agent_id, std::vector<std::shared_ptr<ims::Constraint>>{std::make_shared<EdgeConstraint>(constraint)});
    }
}

void point3dVertexConflictToPathPriorityConstraints(const Point3dVertexConflict * point3d_conflict_ptr,
                                                      const std::vector<std::string>& agent_names,
                                                      std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints){
    // Create a new Vertex avoidance constraint for each of the agents. Point3dVertexConflict assumes private grids, so each agent has its own state.
    for (int i = 0; i < point3d_conflict_ptr->agent_ids.size(); i++) {
        int agent_id = point3d_conflict_ptr->agent_ids[i];

        // Create a new Vertex constraint. The agent states are the other states that the agent should avoid. Their corresponding agent ids are also passed.
        auto time = (TimeType)point3d_conflict_ptr->states.front().back();
        // The agent names to avoid are the agent names corresponding to the agent ids to avoid.
        std::vector<int> agent_ids_to_avoid;
        std::vector<std::string> agent_names_to_avoid;
        for (int agent_id_to_avoid : point3d_conflict_ptr->agent_ids) {
            if (agent_id_to_avoid != agent_id) {
                agent_ids_to_avoid.push_back(agent_id_to_avoid);
                agent_names_to_avoid.push_back(agent_names[agent_id_to_avoid]);
            }
        }

        PathPriorityConstraint constraint = PathPriorityConstraint(agent_ids_to_avoid);
        constraint.agent_names_to_avoid = agent_names_to_avoid;

        // Update the constraints collective to also include the new constraint.
        agent_constraints.emplace_back(agent_id, std::vector<std::shared_ptr<ims::Constraint>>{std::make_shared<PathPriorityConstraint>(constraint)});
    }
}

void point3dEdgeConflictToPathPriorityConstraints(const Point3dEdgeConflict * point3d_conflict_ptr,
                                                  const std::vector<std::string>& agent_names,
                                                  std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>>& agent_constraints){

    // Create a new edge avoidance constraint for each of the agents. Point3dEdgeConflict assumes private grids, so each agent has its own state.
    for (int i = 0; i < point3d_conflict_ptr->agent_ids.size(); i++) {
        int agent_id = point3d_conflict_ptr->agent_ids[i];

        // Create a new edge constraint. The agent states are the other states that the agent should avoid. Their corresponding agent ids are also passed.
        auto time_from = (TimeType)std::round(point3d_conflict_ptr->from_states[i].back());
        auto time_to = (TimeType)std::round(point3d_conflict_ptr->to_states[i].back());

        // The agent names to avoid are the agent names corresponding to the agent ids to avoid.
        std::vector<int> agent_ids_to_avoid;
        std::vector<std::string> agent_names_to_avoid;
        for (int agent_id_to_avoid : point3d_conflict_ptr->agent_ids) {
            if (agent_id_to_avoid != agent_id) {
                agent_ids_to_avoid.push_back(agent_id_to_avoid);
                agent_names_to_avoid.push_back(agent_names[agent_id_to_avoid]);
            }
        }
        PathPriorityConstraint constraint = PathPriorityConstraint(agent_ids_to_avoid);
        constraint.agent_names_to_avoid = agent_names_to_avoid;

        // Update the constraints collective to also include the new constraint.
        agent_constraints.emplace_back(agent_id, std::vector<std::shared_ptr<ims::Constraint>>{std::make_shared<PathPriorityConstraint>(constraint)});
    }
}

}  // namespace conflict_conversions
}  // namespace ims
