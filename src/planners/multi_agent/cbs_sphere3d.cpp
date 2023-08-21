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
 * \file   cbs_sphere3d.cpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   August 21 2023
 */

#include <search/planners/astar.hpp>
#include <search/planners/multi_agent/cbs_sphere3d.hpp>

ims::CBSSphere3d::CBSSphere3d(const ims::CBSParams& params) : params_(params), ims::CBS(params) {}

std::vector<std::pair<int, std::shared_ptr<ims::Constraint>>> ims::CBSSphere3d::conflictsToConstraints(const std::vector<std::shared_ptr<ims::Conflict>>& conflicts) {
    std::vector<std::pair<int, std::shared_ptr<ims::Constraint>>> agent_constraints;
    // TODO(yoraish): this is WIP.
    // Iterate through the conflicts and convert them to constraints.
    for (auto& conflict_ptr : conflicts) {
        // Create a new constraint given the conflict.
        switch (conflict_ptr->type){
            case ConflictType::POINT3D: {
                // Get the location of each of the agents. Each one is specified in its own grid.
                auto* point3d_conflict_ptr = dynamic_cast<Point3dConflict*>(conflict_ptr.get());
                // Check if the conversion succeeded.
                if (point3d_conflict_ptr == nullptr) {
                    throw std::runtime_error("Conflict is a point3d conflict, but could not be converted to a Point3dConflict.");
                }

                // For each affected agent (2, in CBS), create a new constraint, and down the line a search state for each as well.
                for (int i = 0; i < point3d_conflict_ptr->agent_ids.size(); i++) {
                    int agent_id = point3d_conflict_ptr->agent_ids[i];
                    TimeType time = point3d_conflict_ptr->states.front().back();

                    // Create a new vertex constraint.
                    Sphere3dConstraint constraint = Sphere3dConstraint(point3d_conflict_ptr->point, sphere3d_constraint_radius_, time);

                    // Update the constraints collective to also include the new constraint.
                    agent_constraints.emplace_back(agent_id, std::make_shared<Sphere3dConstraint>(constraint));
                }
            }
        }
    }
    return agent_constraints;
}
