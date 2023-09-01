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

#ifndef SEARCH_ECBS_PRIVATE_GRIDS_HPP
#define SEARCH_ECBS_PRIVATE_GRIDS_HPP

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
#include <search/planners/multi_agent/ecbs.hpp>

#include "search/action_space/constrained_action_space.hpp"

namespace ims {

/// @brief An object for mapping [agent_ids][timestamp] to a set of constraints.
using MultiAgentConstraintsCollective = std::unordered_map<int, ConstraintsCollective>;

/// @brief An object for mapping [agent_ids][timestamp] to a state.
using MultiAgentPaths = std::unordered_map<int, std::vector<StateType>>;

// ==========================
// ECBS Private Grids Algorithm.
// ==========================
/// @class ECBS for private grids class.
/// @brief The ECBS algorithm for private grids. Perhaps obviously.
class ECBSPrivateGrids : public ECBS {
private:
public:
    /// @brief Constructor
    /// @param params The parameters
    explicit ECBSPrivateGrids(const ECBSParams& params);

    /// @brief Destructor
    ~ECBSPrivateGrids() override = default;

    /// @brief Get the required conflict types.
    /// @return The required conflict types.
    inline std::vector<ConflictType> getConflictTypes() override { return conflict_types_; }

protected:
    /// @brief  Convert conflicts to constraints. This is a main differentiator between ECBS and ECBSPrivateGrids. ECBS can handle vertex and edge constraints, so ECBSPrivateGrids converts conflicts to these types of constraints.
    /// @param conflicts
    /// @return mapping from agent id to constraints.
    std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>> conflictsToConstraints(const std::vector<std::shared_ptr<Conflict>>& conflicts) override;

    // Parameters.
    ECBSParams params_;

    // The required conflict types.
    std::vector<ConflictType> conflict_types_ = {ConflictType::PRIVATE_GRIDS_VERTEX, ConflictType::PRIVATE_GRIDS_EDGE};
};

}  // namespace ims

#endif  // SEARCH_ECBS_PRIVATE_GRIDS_HPP
