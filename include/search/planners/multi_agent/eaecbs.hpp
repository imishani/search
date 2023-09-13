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
 * \file   eaecbs.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   September 11 2023
 */

#ifndef SEARCH_EAECBS_HPP
#define SEARCH_EAECBS_HPP

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
#include <search/planners/wastar.hpp>
#include <search/planners/best_first_search.hpp>
#include <search/planners/multi_agent/ecbs.hpp>
#include <search/planners/multi_agent/eacbs.hpp>

#include "search/action_space/constrained_action_space.hpp"

namespace ims {

// ==========================
// Related structs: EAECBSParams
// ==========================
/// @class EAECBSParams class.
/// @brief The parameters for the ECBS algorithm
struct EAECBSParams : public ECBSParams {
    /// @brief Constructor
    explicit EAECBSParams() : ECBSParams() {}

    /// @brief Destructor
    ~EAECBSParams() override = default;
};

// ==========================
// EAECBS Algorithm.
// ==========================
/// @class EAECBS class.
/// @brief The EAECBS algorithm.
class EAECBS : public ECBS {
private:

public:
    /// @brief Constructor
    /// @param params The parameters
    explicit EAECBS(const EAECBSParams& params);

    /// @brief Destructor
    ~EAECBS() override = default;

    void initializePlanner(std::vector<std::shared_ptr<ConstrainedActionSpace>>& action_space_ptrs,
                           const std::vector<StateType>& starts, const std::vector<StateType>& goals);
    void initializePlanner(std::vector<std::shared_ptr<ExperienceAcceleratedConstrainedActionSpace>>& action_space_ptrs,
                           const std::vector<StateType>& starts, const std::vector<StateType>& goals);
    void initializePlanner(std::vector<std::shared_ptr<ConstrainedActionSpace>>& action_space_ptrs, const std::vector<std::string>& agent_names, const std::vector<StateType>& starts, const std::vector<StateType>& goals);
    void initializePlanner(std::vector<std::shared_ptr<ExperienceAcceleratedConstrainedActionSpace>>& action_space_ptrs, const std::vector<std::string>& agent_names, const std::vector<StateType>& starts, const std::vector<StateType>& goals);

    /// @brief plan a path
    /// @param path The path
    /// @return whether the plan was successful or not
    bool plan(MultiAgentPaths& paths);

protected:

    struct SearchState: public EACBS::SearchState{};

    /// @brief The open list. We set it to a deque for fast pop_front().
    using OpenList = ::smpl::IntrusiveHeap<SearchState, SearchStateCompare>;
    OpenList open_;

    // The states that have been created.
    std::vector<SearchState*> states_;

    /// @brief Get the state by id
    /// @param state_id The id of the state
    /// @return The search state
    /// @note Use this function to get search states that are already created.
    auto getSearchState(int state_id) -> SearchState*;

    /// @brief Get the state by id or create a new one if it does not exist
    /// @param state_id The id of the state
    /// @return The search state
    auto getOrCreateSearchState(int state_id) -> SearchState*;

    /// @brief Generate descendents of a state, a key method in most search algorithms.
    /// @param state_id
    void expand(int state_id) override;

    void verifyStartAndGoalInputStates(const std::vector<StateType>& starts, const std::vector<StateType>& goals);
    
    // Public variable. For shadowing.
    /// @brief The conflict types that this algorithm asks for from the action space.
    std::vector<ConflictType> conflict_types_ = {ConflictType::EDGE, ConflictType::VERTEX};

    /// Member variables.
    // The search parameters.
    ECBSParams params_;

    // The action spaces for the individual agents.
    std::vector<std::shared_ptr<ExperienceAcceleratedConstrainedActionSpace>> agent_action_space_ptrs_;

    // The low-level planners.
    std::vector<std::shared_ptr<EAwAStarUniformCost>> agent_planner_ptrs_;

};



}  // namespace ims

#endif  // SEARCH_EAECBS_HPP
