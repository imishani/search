/*
 * Copyright (C) 2024, Yorai Shaoul
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
 * \file   cbs_sipp.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   2024-01-17
 */
#pragma once

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
#include <search/planners/sipp.hpp>
#include <search/planners/best_first_search.hpp>
#include <search/planners/multi_agent/cbs.hpp>

#include "search/action_space/constrained_action_space.hpp"

namespace ims {

/// @brief An object for mapping [agent_ids][timestamp] to a set of constraints.
using MultiAgentConstraintsCollective = std::unordered_map<int, ConstraintsCollective>;

/// @brief An object for mapping [agent_ids][timestamp] to a state.
using MultiAgentPaths = std::unordered_map<int, std::vector<StateType>>;

// ==========================
// Related structs: CBSSIPPParams
// ==========================

/// @class CBSParams class.
/// @brief The parameters for the CBS algorithm
struct CBSSIPPParams : public CBSParams {
    /// @brief Constructor
    explicit CBSSIPPParams() : CBSParams() {}

    /// @brief Destructor
    ~CBSSIPPParams() override = default;
};

// ==========================
// CBS Sphere-3D Algorithm.
// ==========================
/// @class CBS for private grids class.
/// @brief The CBS algorithm for private grids. Perhaps obviously.
class CBSSIPP : public CBS {
private:

public:
    /// @brief Constructor
    /// @param params The parameters
    explicit CBSSIPP(const CBSSIPPParams& params);

    /// @brief Destructor
    ~CBSSIPP() override = default;

    void initializePlanner(std::vector<std::shared_ptr<ConstrainedActionSpace>>& action_space_ptrs,
                                 const std::vector<StateType>& starts, const std::vector<StateType>& goals) override;
    void initializePlanner(std::vector<std::shared_ptr<ConstrainedActionSpace>> &action_space_ptrs,
                           const std::vector<std::string> &agent_names, const std::vector<StateType> &starts,
                           const std::vector<StateType> &goals) override;
    void createRootInOpenList() override;

protected:
    void expand(int state_id) override;


    // Parameters.
    CBSSIPPParams params_;

    // The low-level planners are SIPP.
    std::vector<std::shared_ptr<SIPP>> agent_planner_ptrs_;
};

}  // namespace ims

