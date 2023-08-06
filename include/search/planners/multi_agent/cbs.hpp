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
 * \file   cbs.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   07/07/23
 */

#ifndef SEARCH_CBS_HPP
#define SEARCH_CBS_HPP

// Standard includes.
#include <algorithm>
#include <functional>
#include <iostream>
#include <numeric>
#include <utility>
#include <vector>

// Project includes.
#include <search/common/conflicts.hpp>
#include <search/common/constrained_action_space.hpp>
#include <search/common/constraints.hpp>
#include <search/heuristics/standard_heuristics.hpp>
#include <search/planners/astar.hpp>
#include <search/planners/best_first_search.hpp>

/*
Some things that need to be done:
=== For the low level planner ===
=== For the high level planner ===
1. Fix the waiting-at-goal error. (Wait-at-goal just added blindly. Next plans should be at least as long as the previous ones and waiting pruned?)
3. Add the edge constraints.
4. Continue work on the main loop. (Not on expand.)

*/

namespace ims {

// ==========================
// Related structs: CBSParams
// ==========================

/// @class CBSParams class.
/// @brief The parameters for the CBS algorithm
struct CBSParams : public BestFirstSearchParams {
    /// @brief Constructor
    explicit CBSParams() : BestFirstSearchParams(new ims::ZeroHeuristic()) {
    }

    /// @brief Destructor
    ~CBSParams() override = default;

    /// @brief Exhaustive search flag. If true, the algorithm will continue to search until the goal is found or the open list is empty.
    bool exhaustive = false;
};

// ==========================
// Related structs: Constraints
// ==========================

struct VertexConstraint : public Constraint {
    /// @brief The state vector. Could be a robot configuration.
    // We specify the states directly since their ID may change in future low-level plan iterations.
    StateType state;

    /// @brief Constructor, allowing to set the state, time, and type.
    /// @param state The state vector.
    explicit VertexConstraint(StateType state) : state(std::move(state)) {
        /// @brief The type of the constraint.
        type = ConstraintType::VERTEX_CONSTRAINT;
    }

    /// @brief String representation of the constraint.
    /// @return The string representation.
    std::string toString() const override {
        std::stringstream ss;
        ss << "VertexConstraint: "
           << " (" << state[0] << ", " << state[1] << ", " << state[2] << ")";
        return ss.str();
    }

    /// @brief The time interval of the constraint.
    std::pair<int, int> getTimeInterval() const {
        return std::make_pair(state.back(), state.back());
    }
};

struct EdgeConstraint : public Constraint {
    /// @brief The state vector. Could be a robot configuration.
    // We specify the states directly since their ID may change in future low-level plan iterations.
    StateType from_state;
    StateType to_state;

    /// @brief Constructor, allowing to set the state, time, and type.
    /// @param state The state vector.
    explicit EdgeConstraint(StateType from_state, StateType to_state) : from_state(std::move(from_state)), to_state(std::move(to_state)) {
        /// @brief The type of the constraint.
        type = ConstraintType::EDGE_CONSTRAINT;
    }
    
    std::string toString() const override {
        std::stringstream ss;
        ss << "EdgeConstraint. From: (" << from_state[0] << ", " << from_state[1] << ", " << from_state[2] << ") To: (" << to_state[0] << ", " << to_state[1] << ", " << to_state[2] << ")";
        return ss.str();
    }

    /// @brief The time interval of the constraint.
    std::pair<int, int> getTimeInterval() const {
        return std::make_pair(from_state.back(), to_state.back());
    }
};

/// @brief An object for mapping [agent_ids][timestamp] to a set of constraints.
using MultiAgentConstraintsCollective = std::unordered_map<int, ConstraintsCollective>;

/// @brief An object for mapping [agent_ids][timestamp] to a state.
using MultiAgentPaths = std::unordered_map<int, std::vector<StateType>>;

// ==========================
// Related structs: Conflicts
// ==========================

struct VertexConflict : public Conflict {
    /// @brief The state vector. Could be a robot configuration.
    // We specify the states directly since their ID may change in future low-level plan iterations.
    StateType state;

    // The agent IDs.
    std::vector<int> agent_ids;

    /// @brief Constructor, allowing to set the state, time, and type.
    /// @param state The state vector.
    explicit VertexConflict(StateType state, std::vector<int> agent_ids) : state(std::move(state)), agent_ids(std::move(agent_ids)) {
        /// @brief The type of the Conflict.
        type = ConflictType::VERTEX_CONFLICT;
    }
};

struct EdgeConflict : public Conflict {
    /// @brief The state vector. Could be a robot configuration.
    // We specify the states directly since their ID may change in future low-level plan iterations.
    StateType from_state;
    StateType to_state;

    // The agent IDs.
    int agent_id_from;
    int agent_id_to;

    /// @brief Constructor, allowing to set the state, time, and type.
    /// @param state The state vector.
    explicit EdgeConflict(StateType from_state, StateType to_state, int agent_id_from, int agent_id_to) : from_state(std::move(from_state)), to_state(std::move(to_state)), agent_id_from(agent_id_from), agent_id_to(agent_id_to) {
        /// @brief The type of the Conflict.
        type = ConflictType::EDGE_CONFLICT;
    }
};

// ==========================
// CBS Algorithm.
// ==========================
/// @class CBS class.
/// @brief The CBS algorithm.
class CBS : public BestFirstSearch {
private:
    // friend wAStar;

    // The searchState struct. Keeps track of the state id, parent id, and cost. In CBS, we also add the constraints and paths.
    /// @brief The search state.
    struct SearchState : public ims::BestFirstSearch::SearchState {
        // Map from agent id to a map from time to a set of constraints. Note the quick check for any constraints at a given time. By constraints[agent_id][time].empty() we  can check if there are any constraints at a given time.
        MultiAgentConstraintsCollective constraints_collectives;

        // Map from agent id to a path. Get the state vector for agent i at time t by paths[agent_id][t].
        MultiAgentPaths paths;

        // The path costs.
        std::unordered_map<int, double> paths_costs;
    };

    /// @brief The open list. We set it to a deque for fast pop_front().
    using OpenList = smpl::IntrusiveHeap<SearchState, SearchStateCompare>;
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

    /// @brief Pad a set of paths such that they are all the maximum length.
    /// @note The padding is done by repeating the last state and incrementing time accordingly.
    /// @param paths The paths to pad.
    void padPathsToMaxLength(MultiAgentPaths& paths);

    std::vector<std::pair<int, std::shared_ptr<Constraint>>> conflictsToConstraints(const std::vector<std::shared_ptr<Conflict>>& conflicts);

public:
    /// @brief Constructor
    /// @param params The parameters
    explicit CBS(const CBSParams& params);

    /// @brief Destructor
    ~CBS() override = default;

    /// @brief Initialize the planner.
    /// @param action_spaces_ptr The action space. The action spaces of all agents must be pointing to the same scene interface.
    /// @param starts The start states for all agents.
    /// @param goals The goal states for all agents.
    void initializePlanner(std::vector<std::shared_ptr<ConstrainedActionSpace>>& action_space_ptrs,
                           const std::vector<StateType>& starts, const std::vector<StateType>& goals);

    /// @brief plan a path
    /// @param path The path
    /// @return whether the plan was successful or not
    bool plan(MultiAgentPaths& paths);

protected:
    /// @brief Set the search state struct values.
    /// @param state_id
    /// @param parent_id
    /// @param cost
    void setStateVals(int state_id, int parent_id, double cost) override;

    /// @brief Generate descendents of a state, a key method in most search algorithms.
    /// @param state_id
    void expand(int state_id) override;

    /// @brief Checks that the start and goals states are valid. The checks are for time (all initial times are zero and all goal times are -1), for individual agents, and between agents.
    /// @param starts
    /// @param goals
    void verifyStartAndGoalInputStates(const std::vector<StateType>& starts, const std::vector<StateType>& goals);

    /// Member variables.
    // The search parameters.
    CBSParams params_;

    // The action spaces for the individual agents.
    std::vector<std::shared_ptr<ConstrainedActionSpace>> agent_action_space_ptrs_;

    // The low-level planners.
    std::vector<std::shared_ptr<AStar>> agent_planner_ptrs_;

    // The low-level planners parameters.
    EuclideanRemoveTimeHeuristic* low_level_planner_heuristic_ptr_ = new ims::EuclideanRemoveTimeHeuristic();
    AStarParams astar_params_ = AStarParams(low_level_planner_heuristic_ptr_);

    /// @brief The start and goal states of the single agents. Remember that these have a time dimension in them.
    std::vector<StateType> starts_;
    std::vector<StateType> goals_;

    /// @brief The goal states. Remember that these have a time dimension in them. In CBS the end time is not known, so we set it to -1.
    // TODO(yoraish): consider creating a MultiAgentPlanner class that will hold the starts, goals, and other multi-agent related variables.
    std::vector<int> found_goal_search_state_ids_;

    /// @brief The number of agents.
    int num_agents_;
};

}  // namespace ims

#endif  // SEARCH_CBS_HPP
