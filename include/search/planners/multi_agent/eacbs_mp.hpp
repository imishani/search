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
 * \file   eacbs_mp.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   October 19 2023
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
#include <search/planners/eawastar.hpp>
#include <search/planners/best_first_search.hpp>
#include <search/common/conflict_conversions.hpp>
#include <search/planners/multi_agent/eacbs.hpp>

#include "search/action_space/subcost_experience_accelerated_constrained_action_space.hpp"

namespace ims {

// ==========================
// Related structs: EACBSMPParams
// ==========================

/// @class EACBSMPParams class.
/// @brief The parameters for the EACBSMP algorithm
struct EACBSMPParams : public EACBSParams {
    /// @brief Constructor
    explicit EACBSMPParams() : EACBSParams() {
    }

    /// @brief Destructor
    ~EACBSMPParams() override = default;
};

/// @brief An object for mapping [agent_ids][timestamp] to a set of constraints.
using MultiAgentConstraintsCollective = std::unordered_map<int, ConstraintsCollective>;

/// @brief An object for mapping [agent_ids][timestamp] to a state.
using MultiAgentPaths = std::unordered_map<int, std::vector<StateType>>;

// ==========================
// EACBSMP Algorithm.
// ==========================
/// @class EACBSMP class.
/// @brief The EACBSMP algorithm.
class EACBSMP : public EACBS {
private:    
    friend class EAECBS;
public:
    /// @brief Constructor
    /// @param params The parameters
    explicit EACBSMP(const EACBSMPParams& params);

    /// @brief Destructor
    ~EACBSMP() override = default;

protected:

    /// @brief Get constraints from conflicts. This is where this algorithm differs from CBS. It creates state-avoidance constraints (avoid another agent at their specified state and time), as opposed to a negative costraint on the ego robot.
    /// @param conflicts 
    /// @return 
    virtual std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>> conflictsToConstraints(const std::vector<std::shared_ptr<Conflict>>& conflicts) override;

    /// Member variables.
    // The search parameters.
    EACBSMPParams params_;
};

}  // namespace ims

