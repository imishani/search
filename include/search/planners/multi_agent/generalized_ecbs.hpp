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
 * \file   generalized_ecbs.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   2024-01-20
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
#include <search/planners/focal_search/focal_wastar.hpp>
#include <search/planners/focal_search/focal_eawastar.hpp>
#include <search/planners/best_first_search.hpp>
#include <search/planners/multi_agent/generalized_cbs.hpp>
#include <search/action_space/subcost_action_space.hpp>
#include <search/planners/focal_search/focal_eawastar.hpp>
#include <search/planners/multi_agent/eacbs.hpp>
#include <search/common/constraints.hpp>

namespace ims {

// ==========================
// Related structs: GeneralizedECBSParams
// ==========================
/// @class GeneralizedECBSParams class.
/// @brief The parameters for the GeneralizedECBS algorithm
struct GeneralizedECBSParams : public GeneralizedCBSParams {
    /// @brief Constructor
    explicit GeneralizedECBSParams() : GeneralizedCBSParams() {
    }

    /// @brief Destructor
    ~GeneralizedECBSParams() override = default;

    /// @brief Exhaustive search flag. If true, the algorithm will continue to search until the goal is found or the open list is empty.
    bool exhaustive = false;

    /// @brief The sub-optimality bound on the high-level focal search.
    double high_level_focal_suboptimality = 2.0;
    double low_level_focal_suboptimality = 2.0;

    /// @brief Parameters for specific constraint types.
    double sphere3d_constraint_radius = 0.2;

    /// @brief The constraints to create from the conflicts.
    std::unordered_set<ConstraintType> constraint_types_to_create = {
                                                        ConstraintType::SPHERE3D, // "Do not be in this sphere at this time."
                                                        ConstraintType::EDGE_STATE_AVOIDANCE, // "Between these times, avoid those agents taking the specified config. transitions."
                                                        ConstraintType::VERTEX_STATE_AVOIDANCE, // "At this time, avoid those agents taking the specified configurations."
                                                        ConstraintType::EDGE, // "Do not traverse this edge between these times."
                                                        ConstraintType::VERTEX, // "Do not be at this vertex at this time."

                                                        // ConstraintType::EDGE_PRIORITY, // "Between these times, avoid those agents (whereever they are)."
                                                        // ConstraintType::VERTEX_PRIORITY, // "At this time, avoid those agents (whereever they are)."
                                                        };
};

// ==========================
// GeneralizedECBS Algorithm.
// ==========================
/// @class GeneralizedECBS class.
/// @brief The GeneralizedECBS algorithm.
class GeneralizedECBS : public GeneralizedCBS {
private:

public:
    /// @brief Constructor
    /// @param params The parameters
    explicit GeneralizedECBS(const GeneralizedECBSParams& params);

    /// @brief Destructor
    ~GeneralizedECBS() override = default;

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
    struct GeneralizedECBSOpenCompare{
        bool operator()(const SearchState& s1, const SearchState& s2) const{
            double f1 = s1.f;
            double f2 = s2.f;
            double g1 = s1.g;
            double g2 = s2.g;
            double c1 = s1.unresolved_conflicts.size();
            double c2 = s2.unresolved_conflicts.size();

            if (f1 == f2) {
                if (g1 == g2) {
                    if (c1 == c2) {
                        return s1.state_id < s2.state_id;
                    } else {
                        return c1 < c2;
                    }
                } else {
                    return g1 < g2;
                }
            } else {
                return f1 < f2;
            }
        }   
    };

    /// @brief The search state compare structs for the HL focal lists.
    struct GeneralizedECBSSphere3dConstraintFocalCompare{
        bool operator()(const SearchState& s1, const SearchState& s2) const{
            int constraints_count_s1 = std::accumulate(s1.constraint_type_count.begin(), s1.constraint_type_count.end(), 0, [](int sum, const std::pair<ConstraintType, int>& p){return sum + p.second;});
            int constraints_count_s2 = std::accumulate(s2.constraint_type_count.begin(), s2.constraint_type_count.end(), 0, [](int sum, const std::pair<ConstraintType, int>& p){return sum + p.second;});

            double constraint_density_s1 = 0;
            double constraint_density_s2 = 0;
            
            if (s1.constraint_type_count.find(ConstraintType::SPHERE3D) != s1.constraint_type_count.end()){
                constraint_density_s1 = s1.constraint_type_count.at(ConstraintType::SPHERE3D) / (double)constraints_count_s1;
            }
            if (s2.constraint_type_count.find(ConstraintType::SPHERE3D) != s2.constraint_type_count.end()){
                constraint_density_s2 = s2.constraint_type_count.at(ConstraintType::SPHERE3D) / (double)constraints_count_s2;
            }

            if (constraint_density_s1 == constraint_density_s2) {
                if (s1.f == s2.f) {
                    if (s1.g == s2.g) {
                        int c1 = s1.unresolved_conflicts.size();
                        int c2 = s2.unresolved_conflicts.size();
                        // Compare unresolved conflicts count
                        if (c1 == c2) {
                            return s1.state_id < s2.state_id;
                        }
                        // s1 will come before s2 if it has fewer conflicts.
                        return c1 < c2;                    }
                    return s1.g < s2.g;
                }
                return s1.f < s2.f;
            }

            // s1 will come before s2 if it has a higher constraint density.
            return constraint_density_s1 > constraint_density_s2;
        }
    };

    /// @brief The search state compare structs for the HL focal lists.
    struct GeneralizedECBSPriorityConstraintFocalCompare{
        bool operator()(const SearchState& s1, const SearchState& s2) const{
            int constraints_count_s1 = std::accumulate(s1.constraint_type_count.begin(), s1.constraint_type_count.end(), 0, [](int sum, const std::pair<ConstraintType, int>& p){return sum + p.second;});
            int constraints_count_s2 = std::accumulate(s2.constraint_type_count.begin(), s2.constraint_type_count.end(), 0, [](int sum, const std::pair<ConstraintType, int>& p){return sum + p.second;});

            double constraint_density_s1 = 0;
            double constraint_density_s2 = 0;
            
            int num_edge_constraints_s1 = 0;
            int num_edge_constraints_s2 = 0;
            int num_vertex_constraints_s1 = 0;
            int num_vertex_constraints_s2 = 0;

            if (s1.constraint_type_count.find(ConstraintType::EDGE_PRIORITY) != s1.constraint_type_count.end()){
                num_edge_constraints_s1 = s1.constraint_type_count.at(ConstraintType::EDGE_PRIORITY);
            }
            if (s2.constraint_type_count.find(ConstraintType::EDGE_PRIORITY) != s2.constraint_type_count.end()){
                num_edge_constraints_s2 = s2.constraint_type_count.at(ConstraintType::EDGE_PRIORITY);
            }
            if (s1.constraint_type_count.find(ConstraintType::VERTEX_PRIORITY) != s1.constraint_type_count.end()){
                num_vertex_constraints_s1 = s1.constraint_type_count.at(ConstraintType::VERTEX_PRIORITY);
            }
            if (s2.constraint_type_count.find(ConstraintType::VERTEX_PRIORITY) != s2.constraint_type_count.end()){
                num_vertex_constraints_s2 = s2.constraint_type_count.at(ConstraintType::VERTEX_PRIORITY);
            }

            constraint_density_s1 = (num_edge_constraints_s1 + num_vertex_constraints_s1) / (double)constraints_count_s1;
            constraint_density_s2 = (num_edge_constraints_s2 + num_vertex_constraints_s2) / (double)constraints_count_s2;

            if (constraint_density_s1 == constraint_density_s2) {
                if (s1.f == s2.f) {
                    if (s1.g == s2.g) {
                        int c1 = s1.unresolved_conflicts.size();
                        int c2 = s2.unresolved_conflicts.size();
                        // Compare unresolved conflicts count
                        if (c1 == c2) {
                            return s1.state_id < s2.state_id;
                        }
                        // s1 will come before s2 if it has fewer conflicts.
                        return c1 < c2;
                    }
                    return s1.g < s2.g;
                }
                return s1.f < s2.f;
            }

            // s1 will come before s2 if it has a higher constraint density.
            return constraint_density_s1 > constraint_density_s2;
        }
    };


    /// @brief The search state compare structs for the HL focal lists.
    struct GeneralizedECBSStateAvoidanceConstraintFocalCompare{
        bool operator()(const SearchState& s1, const SearchState& s2) const{
            int constraints_count_s1 = std::accumulate(s1.constraint_type_count.begin(), s1.constraint_type_count.end(), 0, [](int sum, const std::pair<ConstraintType, int>& p){return sum + p.second;});
            int constraints_count_s2 = std::accumulate(s2.constraint_type_count.begin(), s2.constraint_type_count.end(), 0, [](int sum, const std::pair<ConstraintType, int>& p){return sum + p.second;});

            double constraint_density_s1 = 0;
            double constraint_density_s2 = 0;
            
            int num_edge_constraints_s1 = 0;
            int num_edge_constraints_s2 = 0;
            int num_vertex_constraints_s1 = 0;
            int num_vertex_constraints_s2 = 0;

            if (s1.constraint_type_count.find(ConstraintType::EDGE_STATE_AVOIDANCE) != s1.constraint_type_count.end()){
                num_edge_constraints_s1 = s1.constraint_type_count.at(ConstraintType::EDGE_STATE_AVOIDANCE);
            }
            if (s2.constraint_type_count.find(ConstraintType::EDGE_STATE_AVOIDANCE) != s2.constraint_type_count.end()){
                num_edge_constraints_s2 = s2.constraint_type_count.at(ConstraintType::EDGE_STATE_AVOIDANCE);
            }
            if (s1.constraint_type_count.find(ConstraintType::VERTEX_STATE_AVOIDANCE) != s1.constraint_type_count.end()){
                num_vertex_constraints_s1 = s1.constraint_type_count.at(ConstraintType::VERTEX_STATE_AVOIDANCE);
            }
            if (s2.constraint_type_count.find(ConstraintType::VERTEX_STATE_AVOIDANCE) != s2.constraint_type_count.end()){
                num_vertex_constraints_s2 = s2.constraint_type_count.at(ConstraintType::VERTEX_STATE_AVOIDANCE);
            }

            constraint_density_s1 = (num_edge_constraints_s1 + num_vertex_constraints_s1) / (double)constraints_count_s1;
            constraint_density_s2 = (num_edge_constraints_s2 + num_vertex_constraints_s2) / (double)constraints_count_s2;

            if (constraint_density_s1 == constraint_density_s2) {
                if (s1.f == s2.f) {
                    if (s1.g == s2.g) {
                        int c1 = s1.unresolved_conflicts.size();
                        int c2 = s2.unresolved_conflicts.size();
                        // Compare unresolved conflicts count
                        if (c1 == c2) {
                            return s1.state_id < s2.state_id;
                        }
                        // s1 will come before s2 if it has fewer conflicts.
                        return c1 < c2;
                    }
                    return s1.g < s2.g;
                }
                return s1.f < s2.f;
            }

            // s1 will come before s2 if it has a higher constraint density.
            return constraint_density_s1 > constraint_density_s2;
        }
    };

    struct GeneralizedECBSConflictCountFocalCompare{
        bool operator()(const SearchState& s1, const SearchState& s2) const{
            if (s1.unresolved_conflicts.size() == s2.unresolved_conflicts.size()) {
                if (s1.f == s2.f) {
                    if (s1.g == s2.g) {
                        return s1.state_id < s2.state_id;
                    }
                    return s1.g < s2.g;
                }
                return s1.f < s2.f;
            }
            return s1.unresolved_conflicts.size() < s2.unresolved_conflicts.size();
        }
    };

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

    /// @brief Replan all agents that have unincorporated constraints.
    /// @param state 
    bool replanOutdatedAgents(SearchState* state) override;

    // Public variable. For shadowing.
    /// @brief The conflict types that this algorithm asks for from the action space.
    // TODO(yoraish): this should be different for each genCBS version?
    // std::vector<ConflictType> conflict_types_ = {ConflictType::EDGE, ConflictType::VERTEX};
    std::vector<ConflictType> conflict_types_ = {ConflictType::POINT3D_VERTEX, ConflictType::POINT3D_EDGE};

    /// Member variables.
    // The search parameters.
    GeneralizedECBSParams params_;

    // The low-level planners. Overrides the CBS planners set to be wAStar.
    std::vector<std::shared_ptr<FocalwAStar>> agent_planner_ptrs_;

    // The action spaces for the individual agents.
    std::vector<std::shared_ptr<SubcostConstrainedActionSpace>> agent_action_space_ptrs_;

    // Statistics.
    FocalSearchPlannerStats stats_;

    /// @brief The current index in the round-robin priority function for the focal search. Zero means that the anchor queue is being popped.
    int current_priority_function_index_ = 0; 

    // The open list. Not inherited from GeneralizedCBS as it uses a multifocal queue. TODO(yoraish): make the change to a multifocal queue here as well.
    // AbstractQueue<SearchState>* open_;
    MultiFocalAndAnchorDTSQueueWrapper<SearchState, GeneralizedECBSOpenCompare>* open_;
};


///////////////////////////////////////////////////////////////////////////


// ==========================
// Related structs: GeneralizedXECBSParams
// ==========================
/// @class GeneralizedXECBSParams class.
/// @brief The parameters for the GeneralizedECBS algorithm
struct GeneralizedXECBSParams : public EACBSParams {
    /// @brief Constructor
    explicit GeneralizedXECBSParams() : EACBSParams() {
    }

    /// @brief Destructor
    ~GeneralizedXECBSParams() override = default;

    /// @brief Exhaustive search flag. If true, the algorithm will continue to search until the goal is found or the open list is empty.
    bool exhaustive = false;

    /// @brief The sub-optimality bound on the high-level focal search.
    double high_level_focal_suboptimality = 2.0;
    double low_level_focal_suboptimality = 2.0;

    /// @brief Parameters for specific constraint types.
    double sphere3d_constraint_radius = 0.1;


    /// @brief The constraints to create from the conflicts.
    std::unordered_set<ConstraintType> constraint_types_to_create = {ConstraintType::EDGE, // "Do not traverse this edge between these times."
                                                         ConstraintType::VERTEX, // "Do not be at this vertex at this time."
                                                         ConstraintType::SPHERE3D, // "Do not be in this sphere at this time."
                                                        //  ConstraintType::EDGE_PRIORITY, // "Between these times, avoid those agents (whereever they are)."
                                                        //  ConstraintType::VERTEX_PRIORITY, // "At this time, avoid those agents (whereever they are)."
                                                        //  ConstraintType::EDGE_STATE_AVOIDANCE, // "Between these times, avoid those agents taking the specified config. transitions."
                                                        //  ConstraintType::VERTEX_STATE_AVOIDANCE, // "At this time, avoid those agents taking the specified configurations."
                                                        };
};

// ==========================
// GeneralizedXECBS Algorithm.
// ==========================
/// @class GeneralizedXECBS class.
/// @brief The GeneralizedXECBS algorithm.
class GeneralizedXECBS : public EACBS {
private:

public:
    /// @brief Constructor
    /// @param params The parameters
    explicit GeneralizedXECBS(const GeneralizedXECBSParams& params);

    /// @brief Destructor
    ~GeneralizedXECBS() override = default;

    /// @brief Initialize the planner.
    /// @param action_spaces_ptr The action space. The action spaces of all agents must be pointing to the same scene interface.
    /// @param starts The start states for all agents.
    /// @param goals The goal states for all agents.
    void initializePlanner(std::vector<std::shared_ptr<SubcostExperienceAcceleratedConstrainedActionSpace>>& action_space_ptrs,
                           const std::vector<StateType>& starts, const std::vector<StateType>& goals);

    /// @brief Initialize the planner and set the agent names.
    /// @param action_spaces_ptr The action space. The action spaces of all agents must be pointing to the same scene interface.
    /// @param agent_names The names of the agents.
    /// @param starts The start states for all agents.
    /// @param goals The goal states for all agents.
    void initializePlanner(std::vector<std::shared_ptr<SubcostExperienceAcceleratedConstrainedActionSpace>>& action_space_ptrs, const std::vector<std::string>& agent_names, const std::vector<StateType>& starts, const std::vector<StateType>& goals);

    void initializePlanner(std::vector<std::shared_ptr<SubcostConstrainedActionSpace>>& action_space_ptrs,
                           const std::vector<StateType>& starts, const std::vector<StateType>& goals);
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
    struct GeneralizedXECBSOpenCompare{
        bool operator()(const SearchState& s1, const SearchState& s2) const{
            double f1 = s1.f;
            double f2 = s2.f;
            double g1 = s1.g;
            double g2 = s2.g;
            double c1 = s1.unresolved_conflicts.size();
            double c2 = s2.unresolved_conflicts.size();

            if (f1 == f2) {
                if (g1 == g2) {
                    if (c1 == c2) {
                        return s1.state_id < s2.state_id;
                    } else {
                        return c1 < c2;
                    }
                } else {
                    return g1 < g2;
                }
            } else {
                return f1 < f2;
            }
        }   
    };

    /// @brief The search state compare structs for the HL focal lists.
    struct GeneralizedXECBSSphere3dConstraintFocalCompare{
        bool operator()(const SearchState& s1, const SearchState& s2) const{
            int constraints_count_s1 = std::accumulate(s1.constraint_type_count.begin(), s1.constraint_type_count.end(), 0, [](int sum, const std::pair<ConstraintType, int>& p){return sum + p.second;});
            int constraints_count_s2 = std::accumulate(s2.constraint_type_count.begin(), s2.constraint_type_count.end(), 0, [](int sum, const std::pair<ConstraintType, int>& p){return sum + p.second;});

            double constraint_density_s1 = 0;
            double constraint_density_s2 = 0;
            
            if (s1.constraint_type_count.find(ConstraintType::SPHERE3D) != s1.constraint_type_count.end()){
                constraint_density_s1 = s1.constraint_type_count.at(ConstraintType::SPHERE3D) / (double)constraints_count_s1;
            }
            if (s2.constraint_type_count.find(ConstraintType::SPHERE3D) != s2.constraint_type_count.end()){
                constraint_density_s2 = s2.constraint_type_count.at(ConstraintType::SPHERE3D) / (double)constraints_count_s2;
            }

            if (constraint_density_s1 == constraint_density_s2) {
                if (s1.f == s2.f) {
                    if (s1.g == s2.g) {
                        int c1 = s1.unresolved_conflicts.size();
                        int c2 = s2.unresolved_conflicts.size();
                        // Compare unresolved conflicts count
                        if (c1 == c2) {
                            return s1.state_id < s2.state_id;
                        }
                        // s1 will come before s2 if it has fewer conflicts.
                        return c1 < c2;                    }
                    return s1.g < s2.g;
                }
                return s1.f < s2.f;
            }

            // s1 will come before s2 if it has a higher constraint density.
            return constraint_density_s1 > constraint_density_s2;
        }
    };

    struct GeneralizedXECBSConflictCountFocalCompare{
        bool operator()(const SearchState& s1, const SearchState& s2) const{
            if (s1.unresolved_conflicts.size() == s2.unresolved_conflicts.size()) {
                if (s1.f == s2.f) {
                    if (s1.g == s2.g) {
                        return s1.state_id < s2.state_id;
                    }
                    return s1.g < s2.g;
                }
                return s1.f < s2.f;
            }
            return s1.unresolved_conflicts.size() < s2.unresolved_conflicts.size();
        }
    };


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

    // Public variable. For shadowing.
    /// @brief The conflict types that this algorithm asks for from the action space.
    // TODO(yoraish): this should be different for each genCBS version?
    // std::vector<ConflictType> conflict_types_ = {ConflictType::EDGE, ConflictType::VERTEX};
    std::vector<ConflictType> conflict_types_ = {ConflictType::POINT3D_VERTEX, ConflictType::POINT3D_EDGE};

    /// Member variables.
    // The search parameters.
    GeneralizedXECBSParams params_;

    // The low-level planners. Overrides the CBS planners set to be wAStar.
    std::vector<std::shared_ptr<FocalEAwAStarUniformCost>> agent_planner_ptrs_;

    // The action spaces for the individual agents.
    std::vector<std::shared_ptr<SubcostExperienceAcceleratedConstrainedActionSpace>> agent_action_space_ptrs_;

    // Statistics.
    FocalSearchPlannerStats stats_;

    /// @brief The current index in the round-robin priority function for the focal search. Zero means that the anchor queue is being popped.
    int current_priority_function_index_ = 0; 

    // The open list. Inherited from GeneralizedCBS.
    // FocalAndAnchorQueueWrapper<SearchState, GeneralizedXECBSOpenCompare, GeneralizedXECBSSphere3dConstraintFocalCompare>* open_;
    FocalAndAnchorQueueWrapper<SearchState, GeneralizedXECBSOpenCompare, GeneralizedXECBSConflictCountFocalCompare>* open_;

    // Experience reuse type.
    ExperienceReuseType experience_reuse_type_ = ExperienceReuseType::PREVIOUS_SOLUTION;
};



}  // namespace ims

