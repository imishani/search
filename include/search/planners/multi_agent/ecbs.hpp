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
 * \file   ecbs.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   07/08/22
 */

#ifndef SEARCH_ECBS_HPP
#define SEARCH_ECBS_HPP

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
#include <search/planners/focal_search/focal_wastar.hpp>
#include <search/planners/best_first_search.hpp>
#include <search/planners/multi_agent/cbs.hpp>
#include <search/planners/focal_search/focal_wsipp.hpp>

#include "search/action_space/subcost_action_space.hpp"

namespace ims {

// ==========================
// Related structs: ECBSParams
// ==========================
/// @class ECBSParams class.
/// @brief The parameters for the ECBS algorithm
struct ECBSParams : public CBSParams {
    /// @brief Constructor
    explicit ECBSParams() : CBSParams() {
    }

    /// @brief Destructor
    ~ECBSParams() override = default;

    /// @brief Exhaustive search flag. If true, the algorithm will continue to search until the goal is found or the open list is empty.
    bool exhaustive = false;

    double high_level_focal_suboptimality = 2.0;
    double low_level_focal_suboptimality = 2.0;
};

/// @brief An object for mapping [agent_ids][timestamp] to a set of constraints.
using MultiAgentConstraintsCollective = std::unordered_map<int, ConstraintsCollective>;

/// @brief An object for mapping [agent_ids][timestamp] to a state.
using MultiAgentPaths = std::unordered_map<int, std::vector<StateType>>;

// ==========================
// ECBS Algorithm.
// ==========================
/// @class ECBS class.
/// @brief The ECBS algorithm.
class ECBS : public CBS {
private:
public:
    /// @brief Constructor
    /// @param params The parameters
    explicit ECBS(const ECBSParams& params);

    /// @brief Destructor
    ~ECBS() override = default;

    /// @brief Initialize the planner.
    /// @param action_spaces_ptr The action space. The action spaces of all agents must be pointing to the same scene interface.
    /// @param starts The start states for all agents.
    /// @param goals The goal states for all agents.
    void initializePlanner(std::vector<std::shared_ptr<SubcostConstrainedActionSpace>>& action_space_ptrs,
                           const std::vector<StateType>& starts, const std::vector<StateType>& goals);

    /// @brief Initialize the planner and set the agent names.
    /// @param action_spaces_ptr The action space. The action spaces of all agents must be pointing to the same scene interface.
    /// @param agent_names The names of the agents.
    /// @param starts The start states for all agents.
    /// @param goals The goal states for all agents.
    void initializePlanner(std::vector<std::shared_ptr<SubcostConstrainedActionSpace>>& action_space_ptrs, const std::vector<std::string>& agent_names, const std::vector<StateType>& starts, const std::vector<StateType>& goals);

    /// @brief Create the root node in the open list. This node has single-agent plans that were planned without any constraints.
    void createRootInOpenList() override;

    /// @brief plan a path
    /// @param path The path
    /// @return whether the plan was successful or not
    bool plan(MultiAgentPaths& paths);

    /// @brief Print the statistics of the search execution.
    PlannerStats reportStats() const{
        return stats_;
    }
protected:

    /// @brief The search state compare struct.
    struct ECBSOpenCompare{
        bool operator()(const SearchState& s1, const SearchState& s2) const{
            double f1 = s1.f;
            double f2 = s2.f;
            double g1 = s1.g;
            double g2 = s2.g;
            double c1 = (double)s1.unresolved_conflicts->size();
            double c2 = (double)s2.unresolved_conflicts->size();

            if (f1 == f2) {
                if (g1 == g2) {
                    if (c1 == c2) {
                        return s1.state_id < s2.state_id; // Fourth.
                    } else {
                        return c1 < c2; // Third.
                    }
                } else {
                    return g1 < g2; // Second.
                }
            } else {
                return f1 < f2; // First.
            }
        }   
    };

    /// @brief The search state compare struct.
    struct ECBSFocalCompare{
        bool operator()(const SearchState& s1, const SearchState& s2) const{
            if (s1.unresolved_conflicts->size() == s2.unresolved_conflicts->size()) {
                if (s1.f == s2.f) {
                    if (s1.g == s2.g) {
                        return s1.state_id < s2.state_id;
                    }
                    return s1.g < s2.g;
                }
                return s1.f < s2.f;
            }
            return s1.unresolved_conflicts->size() < s2.unresolved_conflicts->size();
        }
    };

    /// @brief Generate descendents of a state, a key method in most search algorithms.
    /// @param state_id
    void expand(int state_id) override;

    /// @brief Create the low level planners.
    void createLowLevelPlanners() override;

    /// @brief (Re)Initialize the low level planners and plan. Populate the paths and stats objects.
    /// @param agent_id the agent integer identifier.
    /// @param paths The paths to populate.
    /// @param stats The stats to populate.
    /// @return Whether the initialization and planning was successful.
    virtual bool initializeAndPlanLowLevel(int agent_id, std::vector<StateType>& path, FocalSearchPlannerStats& stats);

    // Public variable. For shadowing.
    /// @brief The conflict types that this algorithm asks for from the action space.
    std::vector<ConflictType> conflict_types_ = {ConflictType::EDGE, ConflictType::VERTEX};

    /// Member variables.
    // The search parameters.
    ECBSParams params_;

    // The low-level planners. Overrides the CBS planners set to be wAStar.
    std::vector<std::shared_ptr<FocalSearch>> agent_planner_ptrs_;

    // The action spaces for the individual agents.
    std::vector<std::shared_ptr<SubcostConstrainedActionSpace>> agent_action_space_ptrs_;

    // Statistics.
    FocalSearchPlannerStats stats_;

};

class ECBSSIPP : public ECBS {
public:
    explicit ECBSSIPP(const ECBSParams& params) : ECBS(params) {}
private:
protected:
    /// @brief Create the low level planners. We use focal weighted SIPP in this instance.
    void createLowLevelPlanners() override;

    /// @brief (Re)Initialize the low level planners and plan. Populate the paths and stats objects.
    /// @param agent_id the agent integer identifier.
    /// @param paths The paths to populate.
    /// @param stats The stats to populate.
    bool initializeAndPlanLowLevel(int agent_id, std::vector<StateType>& path, FocalSearchPlannerStats& stats) override;

    // The low-level planners. Overrides the CBS planners set to be wAStar.
    std::vector<std::shared_ptr<FocalwSIPP>> agent_planner_ptrs_;

};

}  // namespace ims

#endif  // SEARCH_ECBS_HPP
