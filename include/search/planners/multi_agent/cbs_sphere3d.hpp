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
 * \file   cbs_sphere3d.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   August 21 2023
 */

#ifndef SEARCH_CBS_SPHERE3D_HPP
#define SEARCH_CBS_SPHERE3D_HPP

// Standard includes.
#include <algorithm>
#include <functional>
#include <iostream>
#include <numeric>
#include <utility>
#include <vector>

// Project includes.
#include <search/common/conflicts.hpp>
#include <search/common/constraints.hpp>
#include <search/heuristics/standard_heuristics.hpp>
#include <search/planners/astar.hpp>
#include <search/planners/best_first_search.hpp>
#include <search/planners/multi_agent/cbs.hpp>

#include "search/action_space/constrained_action_space.hpp"

namespace ims {

/// @brief An object for mapping [agent_ids][timestamp] to a set of constraints.
using MultiAgentConstraintsCollective = std::unordered_map<int, ConstraintsCollective>;

/// @brief An object for mapping [agent_ids][timestamp] to a state.
using MultiAgentPaths = std::unordered_map<int, std::vector<StateType>>;

// ==========================
// Related structs: CBSSphere3dParams
// ==========================

/// @class CBSParams class.
/// @brief The parameters for the CBS algorithm
struct CBSSphere3dParams : public CBSParams {
    /// @brief Constructor
    explicit CBSSphere3dParams() : CBSParams() {}

    /// @brief Destructor
    ~CBSSphere3dParams() override = default;

    /// @brief Exhaustive search flag. If true, the algorithm will continue to search until the goal is found or the open list is empty.
    bool exhaustive = false;

    /// @brief The radius of the sphere3d constraint.
    double sphere3d_constraint_radius = 0.001;
};

// ==========================
// CBS Sphere-3D Algorithm.
// ==========================
/// @class CBS for private grids class.
/// @brief The CBS algorithm for private grids. Perhaps obviously.
class CBSSphere3d : public CBS {
private:

    /// @brief The radius of the sphere3d constraint.
    double sphere3d_constraint_radius_ = 0.05;

public:
    /// @brief Constructor
    /// @param params The parameters
    explicit CBSSphere3d(const CBSSphere3dParams& params);

    /// @brief Destructor
    ~CBSSphere3d() override = default;

    /// @brief Get the conflict types.
    /// @return The conflict types.
    inline std::vector<ConflictType> getConflictTypes() override { return conflict_types_; }

protected:
    /// @brief  Convert conflicts to constraints. This is one of the main differentiators between CBS and CBSSphere3d. CBS can handle vertex and edge constraints in its original formulation. Now we would like it to instead handle Sphere3d constraints. To enable that, this method converts conflicts (most likely Point3d conflicts) to Sphere3d constraints.
    /// @param conflicts
    /// @return mapping from agent id to constraints.
    std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>> conflictsToConstraints(const std::vector<std::shared_ptr<Conflict>>& conflicts) override;

    // Parameters.
    CBSSphere3dParams params_;

    /// @brief The conflict types that this algorithm asks for from the action space. Shadow the value in the parent class.
    std::vector<ConflictType> conflict_types_{ConflictType::POINT3D};
};

}  // namespace ims

#endif  // SEARCH_CBS_SPHERE3D_HPP
