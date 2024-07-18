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
 * \file   generalized_cbs.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   2024-01-12
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
#include <search/common/conflict_conversions.hpp>
#include <search/common/constraints.hpp>
#include <search/heuristics/standard_heuristics.hpp>
#include <search/planners/focal_search/focal_wastar.hpp>
#include <search/planners/best_first_search.hpp>
#include <search/planners/multi_agent/cbs.hpp>

#include "search/action_space/subcost_action_space.hpp"

namespace ims {

// ==========================
// Related structs: GeneralizedCBSParams
// ==========================
/// @class GeneralizedCBSParams class.
/// @brief The parameters for the GeneralizedCBS algorithm
struct GeneralizedCBSParams : public CBSParams {
    /// @brief Constructor
    explicit GeneralizedCBSParams() : CBSParams() {
    }

    /// @brief Destructor
    ~GeneralizedCBSParams() override = default;

    /// @brief Exhaustive search flag. If true, the algorithm will continue to search until the goal is found or the open list is empty.
    bool exhaustive = false;

    /// @brief The sub-optimality bound on the high-level focal search.
    double high_level_focal_suboptimality = 1.0;

    /// @brief The constraints to create from the conflicts.
    std::vector<ConstraintType> constraint_types_to_create = {ConstraintType::EDGE, // "Do not traverse this edge between these times."
                                                         ConstraintType::VERTEX, // "Do not be at this vertex at this time."
                                                        };
};

/// @brief An object for mapping [agent_ids][timestamp] to a set of constraints.
using MultiAgentConstraintsCollective = std::unordered_map<int, ConstraintsCollective>;

/// @brief An object for mapping [agent_ids][timestamp] to a state.
using MultiAgentPaths = std::unordered_map<int, std::vector<StateType>>;

// ==========================
// GeneralizedCBS Algorithm.
// ==========================
/// @class GeneralizedCBS class.
/// @brief The GeneralizedCBS algorithm.
class GeneralizedCBS : public CBS {
private:

public:
    /// @brief Constructor
    /// @param params The parameters
    explicit GeneralizedCBS(const GeneralizedCBSParams& params);

    /// @brief Destructor
    ~GeneralizedCBS() override = default;

    /// @brief Initialize the planner.
    /// @param action_spaces_ptr The action space. The action spaces of all agents must be pointing to the same scene interface.
    /// @param starts The start states for all agents.
    /// @param goals The goal states for all agents.
    void initializePlanner(std::vector<std::shared_ptr<ConstrainedActionSpace>>& action_space_ptrs,
                           const std::vector<StateType>& starts, const std::vector<StateType>& goals);

    /// @brief Initialize the planner and set the agent names.
    /// @param action_spaces_ptr The action space. The action spaces of all agents must be pointing to the same scene interface.
    /// @param agent_names The names of the agents.
    /// @param starts The start states for all agents.
    /// @param goals The goal states for all agents.
    void initializePlanner(std::vector<std::shared_ptr<ConstrainedActionSpace>>& action_space_ptrs, const std::vector<std::string>& agent_names, const std::vector<StateType>& starts, const std::vector<StateType>& goals);

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
    struct GeneralizedCBSOpenCompare{
        bool operator()(const SearchState& s1, const SearchState& s2) const{
            double f1 = s1.f;
            double f2 = s2.f;
            double g1 = s1.g;
            double g2 = s2.g;

            if (f1 == f2) {
                if (g1 == g2) {
                    return s1.state_id < s2.state_id;
                } else {
                    return g1 < g2;
                }
            } else {
                return f1 < f2;
            }
        }   
    };

    struct GeneralizedCBSConstraintDensityFocalCompare {
    protected:
        std::vector<ConstraintType> constraint_types_;
    public:
        bool operator()(const SearchState& s1, const SearchState& s2) const{
            int constraints_count_s1 = std::accumulate(s1.constraint_type_count.begin(), s1.constraint_type_count.end(), 0, [](int sum, const std::pair<ConstraintType, int>& p){return sum + p.second;});
            int constraints_count_s2 = std::accumulate(s2.constraint_type_count.begin(), s2.constraint_type_count.end(), 0, [](int sum, const std::pair<ConstraintType, int>& p){return sum + p.second;});

            double constraint_density_s1 = 0;
            double constraint_density_s2 = 0;
            int num_constraint_type_s1 = 0;
            int num_constraint_type_s2 = 0;

            for (const auto& constraint_type : constraint_types_){
                if (s1.constraint_type_count.find(constraint_type) != s1.constraint_type_count.end()){
                    num_constraint_type_s1 += s1.constraint_type_count.at(constraint_type);
                }
                if (s2.constraint_type_count.find(constraint_type) != s2.constraint_type_count.end()){
                    num_constraint_type_s2 += s2.constraint_type_count.at(constraint_type);
                }
            }
            constraint_density_s1 = num_constraint_type_s1 / (double)constraints_count_s1;
            constraint_density_s2 = num_constraint_type_s2 / (double)constraints_count_s2;

            if (constraint_density_s1 == constraint_density_s2) {
                if (s1.f == s2.f) {
                    if (s1.g == s2.g) {
                        // Compare unresolved conflicts count
                        return s1.state_id < s2.state_id;
                    }
                    return s1.g < s2.g;
                }
                return s1.f < s2.f;
            }
            return constraint_density_s1 > constraint_density_s2;
        }
    };

    /// @brief The search state compare structs for the HL focal lists.
    struct GeneralizedCBSSphere3dConstraintFocalCompare : public GeneralizedCBSConstraintDensityFocalCompare{
        GeneralizedCBSSphere3dConstraintFocalCompare() {
            constraint_types_ = {ConstraintType::VERTEX_SPHERE3D};
        }
    };

    struct GeneralizedCBSSphere3dLargeConstraintFocalCompare : public GeneralizedCBSConstraintDensityFocalCompare{
        GeneralizedCBSSphere3dLargeConstraintFocalCompare() {
            constraint_types_ = {ConstraintType::SPHERE3DLARGE};
        }
    };

    struct GeneralizedCBSSphere3dXLargeConstraintFocalCompare : public GeneralizedCBSConstraintDensityFocalCompare{
        GeneralizedCBSSphere3dXLargeConstraintFocalCompare() {
            constraint_types_ = {ConstraintType::SPHERE3DXLARGE};
        }
    };

    /// @brief The search state compare structs for the HL focal lists.
    struct GeneralizedCBSPriorityConstraintFocalCompare : public GeneralizedCBSConstraintDensityFocalCompare{
        GeneralizedCBSPriorityConstraintFocalCompare() {
            constraint_types_ = {ConstraintType::EDGE_PRIORITY, ConstraintType::VERTEX_PRIORITY};
        }
    };

    /// @brief The search state compare structs for the HL focal lists.
    struct GeneralizedCBSStateAvoidanceConstraintFocalCompare : public GeneralizedCBSConstraintDensityFocalCompare{
        GeneralizedCBSStateAvoidanceConstraintFocalCompare() {
            constraint_types_ = {ConstraintType::EDGE_STATE_AVOIDANCE, ConstraintType::VERTEX_STATE_AVOIDANCE};
        }
    };

    /// @brief The search state compare structs for the HL focal lists prioritizing path constraints.
    struct GeneralizedCBSPathPriorityConstraintFocalCompare : public GeneralizedCBSConstraintDensityFocalCompare{
        GeneralizedCBSPathPriorityConstraintFocalCompare() {
            constraint_types_ = {ConstraintType::PATH_PRIORITY};
        }
    };

    /// @brief Generate descendents of a state, a key method in most search algorithms.
    /// @param state_id
    virtual void expand(int state_id) = 0;

    /// @brief Convert conflicts to constraints. In generalized CBS, conflicts are converted to all of the constraints that they can be converted to. That is, more than two constraint sets may be created for a single conflict.
    /// @param conflicts 
    /// @return 
    virtual std::vector<std::pair<int, std::vector<std::shared_ptr<ims::Constraint>>>> conflictsToConstraints(const std::vector<std::shared_ptr<ims::Conflict>>& conflicts) = 0;

    /// @brief Get the conflict types requested by the algorithm.
    /// @return The conflict types.
    /// @note Derived class, aka CBS variants that request different conflict types (e.g., point3d, etc.) should override this method and return the conflict types that they need from the action space. The action space will then be queried for these conflict types.
    std::vector<ConflictType> getConflictTypes() override = 0;

    /// @brief Replan all agents that have unincorporated constraints.
    /// @param state 
    virtual bool replanOutdatedAgents(SearchState* state) = 0;

    // Public variable. For shadowing.
    /// @brief The conflict types that this algorithm asks for from the action space.
    // TODO(yoraish): this should be different for each genCBS version?
    // std::vector<ConflictType> conflict_types_ = {ConflictType::EDGE, ConflictType::VERTEX};
    std::vector<ConflictType> conflict_types_;

    /// Member variables.
    // The search parameters.
    GeneralizedCBSParams params_;

    // The low-level planners. Overrides the CBS planners set to be wAStar.
    std::vector<std::shared_ptr<wAStar>> agent_planner_ptrs_;

    // The action spaces for the individual agents.
    std::vector<std::shared_ptr<ConstrainedActionSpace>> agent_action_space_ptrs_;

    // Statistics.
    FocalSearchPlannerStats stats_;

    /// @brief The current index in the round-robin priority function for the focal search. Zero means that the anchor queue is being popped.
    int current_priority_function_index_ = 0; 

    // The open list.
    MultiFocalAndAnchorDTSQueueWrapper<SearchState, GeneralizedCBSOpenCompare>* open_;

    // Parameters for Dynamic Thompson Sampling (DTS). We keep an alpha and beta for each priority function (each focal queue).
    std::unordered_map<int, std::pair<double, double>> dts_alpha_beta_;
    double dts_c_ = 5.0;
};

// ==========================
// Derived class: GeneralizedCBSPoint3d.
// Seeks out point-3d conflicts and imposes sphere-3d, edge, and vertex constraints.
// ==========================

/// @class GeneralizedCBSParams class.
/// @brief The parameters for the GeneralizedCBS algorithm
struct GeneralizedCBSPoint3dParams : public GeneralizedCBSParams {
    /// @brief Constructor
    explicit GeneralizedCBSPoint3dParams() : GeneralizedCBSParams() {
    }

    /// @brief Destructor
    ~GeneralizedCBSPoint3dParams() override = default;

    /// @brief Parameters for specific constraint types.
    double sphere3d_constraint_radius = 0.1;

    /// @brief The constraints to create from the conflicts.
    std::unordered_set<ConstraintType> constraint_types_to_create = {
            ConstraintType::VERTEX_SPHERE3D, // "Do not be in this sphere at this time."
            ConstraintType::SPHERE3DLARGE,
            ConstraintType::SPHERE3DXLARGE ,
            ConstraintType::EDGE_STATE_AVOIDANCE, // "Between these times, avoid those agents taking the specified config. transitions."
            ConstraintType::VERTEX_STATE_AVOIDANCE, // "At this time, avoid those agents taking the specified configurations."
            ConstraintType::EDGE, // "Do not traverse this edge between these times."
            ConstraintType::VERTEX, // "Do not be at this vertex at this time."
            ConstraintType::EDGE_PRIORITY, // "Between these times, avoid those agents (whereever they are)."
            ConstraintType::VERTEX_PRIORITY, // "At this time, avoid those agents (whereever they are)."
//            ConstraintType::PATH_PRIORITY,
    };

    /// @brief The focal queues that should be created.
    std::vector<FocalQueueType> focal_queue_types = {
            FocalQueueType::SPHERE3D_CONSTRAINT_DENSITY,
            FocalQueueType::SPHERE3D_LARGE_CONSTRAINT_DENSITY,
            FocalQueueType::SPHERE3D_XLARGE_CONSTRAINT_DENSITY,
            FocalQueueType::PRIORITY_CONSTRAINT_DENSITY,
            FocalQueueType::STATE_AVOIDANCE_CONSTRAINT_DENSITY,
//            FocalQueueType::PATH_PRIORITY_CONSTRAINT_DENSITY,
    };

    /// @brief Whether to use One-Step-Lazy evaluations.
    bool use_one_step_lazy = true;

};

// ==========================
// GeneralizedCBSPoint3d Algorithm.
// ==========================
/// @class GeneralizedCBSPoint3d class.
/// @brief The GeneralizedCBSPoint3d algorithm.
class GeneralizedCBSPoint3d : public GeneralizedCBS {
private:

public:
    /// @brief Constructor
    /// @param params The parameters
    explicit GeneralizedCBSPoint3d(const GeneralizedCBSPoint3dParams& params);

    /// @brief Destructor
    ~GeneralizedCBSPoint3d() override = default;

    /// @brief Initialize the planner.
    /// @param action_spaces_ptr The action space. The action spaces of all agents must be pointing to the same scene interface.
    /// @param starts The start states for all agents.
    /// @param goals The goal states for all agents.
    void initializePlanner(std::vector<std::shared_ptr<ConstrainedActionSpace>>& action_space_ptrs,
                           const std::vector<StateType>& starts, const std::vector<StateType>& goals);

    /// @brief Initialize the planner and set the agent names.
    /// @param action_spaces_ptr The action space. The action spaces of all agents must be pointing to the same scene interface.
    /// @param agent_names The names of the agents.
    /// @param starts The start states for all agents.
    /// @param goals The goal states for all agents.
    void initializePlanner(std::vector<std::shared_ptr<ConstrainedActionSpace>>& action_space_ptrs, const std::vector<std::string>& agent_names, const std::vector<StateType>& starts, const std::vector<StateType>& goals);

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

    /// @brief Generate descendents of a state, a key method in most search algorithms.
    /// @param state_id
    void expand(int state_id) override;

    /// @brief Convert conflicts to constraints. In generalized CBS, conflicts are converted to all of the constraints that they can be converted to. That is, more than two constraint sets may be created for a single conflict.
    /// @param conflicts 
    /// @return 
    std::vector<std::pair<int, std::vector<std::shared_ptr<ims::Constraint>>>> conflictsToConstraints(const std::vector<std::shared_ptr<ims::Conflict>>& conflicts) override;

    /// @brief Get the conflict types requested by the algorithm.
    /// @return The conflict types.
    /// @note Derived class, aka CBS variants that request different conflict types (e.g., point3d, etc.) should override this method and return the conflict types that they need from the action space. The action space will then be queried for these conflict types.
    inline std::vector<ConflictType> getConflictTypes() override {
        return conflict_types_;
    }

    /// @brief Replan all agents that are specified as needing replan.
    /// @param state 
    bool replanOutdatedAgents(SearchState* state) override;

    // Public variable. For shadowing.
    /// @brief The conflict types that this algorithm asks for from the action space.
    // TODO(yoraish): this should be different for each genCBS version?
    // std::vector<ConflictType> conflict_types_ = {ConflictType::EDGE, ConflictType::VERTEX};
    std::vector<ConflictType> conflict_types_ = {ConflictType::VERTEX_POINT3D, ConflictType::EDGE_POINT3D};

    /// Member variables.
    // The search parameters.
    GeneralizedCBSPoint3dParams params_;

    // The low-level planners. Overrides the CBS planners set to be wAStar.
    std::vector<std::shared_ptr<wAStar>> agent_planner_ptrs_;

    // The action spaces for the individual agents.
    std::vector<std::shared_ptr<ConstrainedActionSpace>> agent_action_space_ptrs_;

    // Statistics.
    FocalSearchPlannerStats stats_;

    /// @brief The current index in the round-robin priority function for the focal search. Zero means that the anchor queue is being popped.
    int current_priority_function_index_ = 0; 

};


}  // namespace ims

