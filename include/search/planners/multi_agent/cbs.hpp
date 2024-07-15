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
#include <search/common/constraints.hpp>
#include <search/heuristics/standard_heuristics.hpp>
#include <search/planners/wastar.hpp>
#include <search/planners/best_first_search.hpp>
#include <search/common/conflict_conversions.hpp>
#include <search/common/queue_general.h>
#include <search/action_space/constrained_action_space.hpp>
#include <search/action_space/action_space.hpp>


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
    
    /// @brief The heuristics to be used by the low level planners.
    std::vector<ims::BaseHeuristic*> low_level_heuristic_ptrs;

    /// @brief The weight to use in the low level planner heuristic.
    double weight_low_level_heuristic = 1.0;
};

// ==========================
// Type definitions.
// ==========================
/// @brief An object for mapping [agent_ids][timestamp] to a set of constraints.
using MultiAgentConstraintsCollective = std::unordered_map<int, ConstraintsCollective>;

// ==========================
// Related classes: CBSBase
// ==========================
/// @brief Base class for all CBS variants. Defines some required methods.
class CBSBase: public BestFirstSearch {
public:
/// @brief Get the conflict types requested by the algorithm.
explicit CBSBase(const CBSParams& params);

/// @brief Destructor.
~CBSBase() override{
    for (auto state_ptr : states_){
        delete state_ptr;
    }
    delete open_;
}

/// @return The conflict types.
virtual std::vector<ConflictType> getConflictTypes() = 0;

protected:
/// @brief Convert conflicts to constraints.
/// @param conflicts The conflicts.
/// @return The constraints. Each conflict is converted to a pair mapping agent id to a vector of constraints.
virtual std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>> conflictsToConstraints(const std::vector<std::shared_ptr<Conflict>>& conflicts) = 0;

/// @brief Verify that the sets of starts and goals are valid.
/// @param starts The start states.
/// @param goals The goal states.
/// @param action_space_ptrs The action spaces.
void verifyStartAndGoalInputStates(const std::vector<StateType>& starts,
                                   const std::vector<StateType>& goals,
                                   const std::vector<std::shared_ptr<ConstrainedActionSpace>>& action_space_ptrs);

/// @brief Initialize the low level planners. Store them in the agent_planner_ptrs_ vector.
virtual void createLowLevelPlanners() = 0;

/// @brief (Re)Initialize and plan low level planners passed to this function.

// The searchState struct. Keeps track of the state id, parent id, and cost. In CBS, we also add the constraints and paths.
/// @brief The search state.
struct SearchState : public ims::BestFirstSearch::SearchState, public SearchStateLowerBoundMixin {
    // Map from agent id to a path. Get the state vector for agent i at time t by paths[agent_id][t].
    MultiAgentPaths paths;

    // The path costs.
    std::unordered_map<int, double> paths_costs;
    double sum_of_costs = 0.0;
    std::unordered_map<int, std::vector<double>> paths_transition_costs;

    // The lower bound on the cost of the solution. This is the sum of the lower bounds of the individual agents.
    std::unordered_map<int, double> path_cost_lower_bounds;
    double sum_of_path_cost_lower_bounds = 0.0;

    // The conflicts. This is a subset of all the conflicts that exist in the current state paths solution. The number of conflicts is determined by the user. For CBS, for example, we only consider the first conflict so the size here could be 1, or larger than 1 and then only one conflict will be converted to a constraint.
    std::vector<std::shared_ptr<Conflict>> unresolved_conflicts = {};

    // Constraints created from the identified conflicts and any previously imposed constraints. Map from agent id to a map from time to a set of constraints. Note the quick check for any constraints at a given time. By constraints[agent_id][time].empty() we  can check if there are any constraints at a given time.
    MultiAgentConstraintsCollective constraints_collectives;

    // The counts of the constraints in each constraint type.
    std::unordered_map<ConstraintType, int> constraint_type_count;

    // A set of agent_ids whose paths are out of date. If there are any constraints here, the f value (aka its sum of costs) of this CT node is only an estimate, as well as potentially other values. If, upon evaluation, the g value become larger than the current g value, then the state is returned to the open list.
    std::vector<int> agent_ids_need_replan = {};

    /// @brief Required for FocalQueue
    /// @return 
    double getLowerBound() const override {
        assert(sum_of_path_cost_lower_bounds > 0.0);
        return sum_of_path_cost_lower_bounds;
    }

};

    // The base CBS class holds an abstract queue. The queue is instantiated in the constructor of the derived classes and reset in the initializePlanner() method of each derived class.
    AbstractQueue<SearchState>* open_;
};

// ==========================
// CBS Algorithm.
// ==========================
/// @class CBS class.
/// @brief The CBS algorithm.
class CBS : public CBSBase {
private:

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
    virtual void initializePlanner(std::vector<std::shared_ptr<ConstrainedActionSpace>>& action_space_ptrs,
                           const std::vector<StateType>& starts, const std::vector<StateType>& goals);

    /// @brief Initialize the planner and set the agent names.
    /// @param action_spaces_ptr The action space. The action spaces of all agents must be pointing to the same scene interface.
    /// @param agent_names The names of the agents.
    /// @param starts The start states for all agents.
    /// @param goals The goal states for all agents.
    virtual void initializePlanner(std::vector<std::shared_ptr<ConstrainedActionSpace>>& action_space_ptrs, const std::vector<std::string>& agent_names, const std::vector<StateType>& starts, const std::vector<StateType>& goals);

    /// @brief  Initialize the conflict tree. This is done by computing a single-agent path for each agent and storing them in a search state in the CT.
    virtual void createRootInOpenList();

    /// @brief plan a path
    /// @param path The path
    /// @return whether the plan was successful or not
    virtual bool plan(MultiAgentPaths& paths);

protected:
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

    std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>> conflictsToConstraints(const std::vector<std::shared_ptr<Conflict>>& conflicts) override;

    /// @brief Set the search state struct values.
    /// @param state_id
    /// @param parent_id
    /// @param cost
    void setStateVals(int state_id, int parent_id, double cost) override;

    /// @brief Generate descendents of a state, a key method in most search algorithms.
    /// @param state_id
    void expand(int state_id) override;

    /// @brief Get the conflict types requested by the algorithm.
    /// @return The conflict types.
    /// @note Derived class, aka CBS variants that request different conflict types (e.g., point3d, etc.) should override this method and return the conflict types that they need from the action space. The action space will then be queried for these conflict types.
    inline std::vector<ConflictType> getConflictTypes() override {
        return conflict_types_;
    }

    /// @brief Create the low level planner objects.
    void createLowLevelPlanners() override;

    /// @brief (Re)Initialize the low level planners and plan. Populate the paths and stats objects.
    /// @param agent_id the agent integer identifier.
    /// @param paths The paths to populate.
    /// @param stats The stats to populate.
    /// @return Whether the initialization and planning was successful.
    virtual bool initializeAndPlanLowLevel(int agent_id, std::vector<StateType>& path, PlannerStats& stats);


    /// Member variables.
    // The search parameters.
    CBSParams params_;

    // The action spaces for the individual agents.
    std::vector<std::shared_ptr<ConstrainedActionSpace>> agent_action_space_ptrs_;

    // The low-level planners.
    std::vector<std::shared_ptr<BestFirstSearch>> agent_planner_ptrs_;

    // The low-level planners parameters.
    // EuclideanRemoveTimeHeuristic* low_level_planner_heuristic_ptr_ = new ims::EuclideanRemoveTimeHeuristic();
    // wAStarParams wastar_params_ = wAStarParams(low_level_planner_heuristic_ptr_, params_.weight_low_level_heuristic);

    /// @brief The start and goal states of the single agents. Remember that these have a time dimension in them.
    std::vector<StateType> starts_;
    std::vector<StateType> goals_;

    /// @brief The number of agents.
    int num_agents_;

    /// @brief The names of the agents.
    std::vector<std::string> agent_names_;

    // Public variable. For shadowing.
    /// @brief The conflict types that this algorithm asks for from the action space.
    std::vector<ConflictType> conflict_types_ = {ConflictType::EDGE, ConflictType::VERTEX};
};

}  // namespace ims

#endif  // SEARCH_CBS_HPP
