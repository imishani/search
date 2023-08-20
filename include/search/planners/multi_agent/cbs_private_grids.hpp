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
 * \file   cbs_private_grids.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   August 18 2023
 */

#ifndef SEARCH_CBS_PRIVATE_GRIDS_HPP
#define SEARCH_CBS_PRIVATE_GRIDS_HPP

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

// ==========================
// Related structs: Constraints
// ==========================
// None new ones, for now.

/// @brief An object for mapping [agent_ids][timestamp] to a set of constraints.
using MultiAgentConstraintsCollective = std::unordered_map<int, ConstraintsCollective>;

/// @brief An object for mapping [agent_ids][timestamp] to a state.
using MultiAgentPaths = std::unordered_map<int, std::vector<StateType>>;

// ==========================
// Related structs: Conflicts
// ==========================
/// @brief A struct for storing a vertex conflict on private grids.
struct PrivateGridsVertexConflict : public Conflict {
    /// @brief The state vector. Could be a robot configuration.
    // We specify the states directly since their ID may change in future low-level plan iterations.
    std::vector<StateType> states;

    // The agent IDs.
    std::vector<int> agent_ids;

    /// @brief Constructor, allowing to set the state, time, and type.
    /// @param state The state vector.
    explicit PrivateGridsVertexConflict(std::vector<StateType> states, std::vector<int> agent_ids) : states(std::move(states)), agent_ids(std::move(agent_ids)) {
        /// @brief The type of the Conflict.
        type = ConflictType::PRIVATE_GRIDS_VERTEX_CONFLICT;
    }
};

// ==========================
// CBS Private Grids Algorithm.
// ==========================
/// @class CBS for private grids class.
/// @brief The CBS algorithm for private grids. Perhaps obviously.
class CBSPrivateGrids : public CBS {
private:
public:
    /// @brief Constructor
    /// @param params The parameters
    explicit CBSPrivateGrids(const CBSParams& params);

    /// @brief Destructor
    ~CBSPrivateGrids() override = default;

protected:
    /// @brief  Convert conflicts to constraints. This is a main differentiator between CBS and CBSPrivateGrids. CBS can handle vertex and edge constraints, so CBSPrivateGrids converts conflicts to these types of constraints.
    /// @param conflicts
    /// @return mapping from agent id to constraints.
    std::vector<std::pair<int, std::shared_ptr<Constraint>>> conflictsToConstraints(const std::vector<std::shared_ptr<Conflict>>& conflicts) override;

    // Parameters.
    CBSParams params_;
};

}  // namespace ims

#endif  // SEARCH_CBS_PRIVATE_GRIDS_HPP
