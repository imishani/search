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
 * \file   drrt.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   October 15 2023
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
#include <search/heuristics/standard_heuristics.hpp>
#include <search/planners/wastar.hpp>
#include <search/planners/best_first_search.hpp>
#include <search/planners/multi_agent/multi_agent_planner.hpp>
#include <search/planners/sampling_based/sampling_based_planner.hpp>
#include <search/action_space/roadmap_action_space.hpp>

namespace ims {

// ==========================
// Related structs: dRRTParams
// ==========================

/// @class dRRTParams class.
/// @brief The parameters for the dRRT algorithm
struct dRRTParams : virtual public PlannerParams {
    /// @brief Constructor
    explicit dRRTParams() : PlannerParams() {
    }

    /// @brief Destructor
    ~dRRTParams() override = default;
    
    /// @brief The heuristics to be used by the low level planners.
    std::vector<ims::BaseHeuristic*> low_level_heuristic_ptrs;

    /// @brief The weight to use in the low level planner heuristic.
    double weight_low_level_heuristic = 1.0;

    /// @brief A priority ordering for planning the agents.
    std::vector<int> agent_priority_ordering = {};
};

/// @brief An object for mapping [agent_ids][timestamp] to a state.
using MultiAgentPaths = std::unordered_map<int, std::vector<StateType>>;

// ==========================
// Discrete RRT Algorithm.
// ==========================
/// @class dRRT class.
/// @brief The Prioritized Planning algorithm.
class dRRT : public MultiAgentPlanner {
private:

    /// @brief The search state.
    struct SearchState: public ims::SearchState {

        /// @brief The parent state
        int parent_id = UNSET;
        /// @brief The cost to come
        double g = INF_DOUBLE;
        /// @brief The f value
        double f = INF_DOUBLE;

        void print() override{
            std::cout << "State: " << state_id << " Parent: " << parent_id << " g: " << g << " f: " << f << std::endl;
        }
    };

    /// @brief The search state compare struct.
    struct SearchStateCompare{
        bool operator()(const SearchState& s1, const SearchState& s2) const{
            if ((s1.f == s2.f) && (s1.g == s2.g))
                return (s1.state_id < s2.state_id);
            else if (s1.f == s2.f)
                // For tie breaking, we prefer the state with the larger g value as it is closer to the goal (lower h in the case of an informed search).
                return s1.g > s2.g;
            else
                return s1.f < s2.f;
        }
    };

public:
    /// @brief Constructor
    /// @param params The parameters
    explicit dRRT(const dRRTParams& params);

    /// @brief Destructor
    ~dRRT() override = default;

    /// @brief Initialize the planner.
    /// @param action_spaces_ptr The action space. The action spaces of all agents must be pointing to the same scene interface.
    /// @param starts The start states for all agents.
    /// @param goals The goal states for all agents.
    void initializePlanner(std::vector<std::shared_ptr<RoadmapActionSpace>>& action_space_ptrs,
                           const std::vector<StateType>& starts, const std::vector<StateType>& goals);

    /// @brief Initialize the planner and set the agent names.
    /// @param action_spaces_ptr The action space. The action spaces of all agents must be pointing to the same scene interface.
    /// @param agent_names The names of the agents.
    /// @param starts The start states for all agents.
    /// @param goals The goal states for all agents.
    void initializePlanner(std::vector<std::shared_ptr<RoadmapActionSpace>>& action_space_ptrs, const std::vector<std::string>& agent_names, const std::vector<StateType>& starts, const std::vector<StateType>& goals);
    void initializePlanner (std::vector<std::shared_ptr<ActionSpace>>& action_space_ptrs,
                                       const std::vector<StateType>& starts,
                                       const std::vector<StateType>& goals) override {
        throw std::runtime_error("dRRT::initializePlanner() does not support action spaces that are not 'roadmap'.");
    }

    /// @brief Plan a path for all agents.
    /// @param paths The paths for all agents.
    /// @return True if a path was found for all agents, false otherwise.
    bool plan(MultiAgentPaths& paths) override;

protected:

    /// @brief Pad a set of paths such that they are all the maximum length.
    /// @note The padding is done by repeating the last state and incrementing time accordingly.
    /// @param paths The paths to pad.
    void padPathsToMaxLength(MultiAgentPaths& paths);

    /// @brief Checks that the start and goals states are valid. The checks are for time (all initial times are zero and all goal times are -1), for individual agents, and between agents.
    /// @param starts
    /// @param goals
    void verifyStartAndGoalInputStates(const std::vector<StateType>& starts, const std::vector<StateType>& goals);

    /// @brief Reset all the member variables in the planner.
    void resetPlanningData() override;

    SearchState* getSearchState(int state_id);
    SearchState* getOrCreateSearchState(int state_id);

    /// Member variables.
    // The search parameters.
    dRRTParams params_;

    // The action spaces for the individual agents.
    std::vector<std::shared_ptr<RoadmapActionSpace>> agent_action_space_ptrs_;

    /// @brief The start and goal states of the single agents. Remember that these have a time dimension in them.
    std::vector<StateType> starts_;
    std::vector<StateType> goals_;

    /// @brief The number of agents.
    int num_agents_;

    /// @brief The names of the agents.
    std::vector<std::string> agent_names_;

    /// @brief The states.
    std::vector<SearchState*> states_;
};

}  // namespace ims
