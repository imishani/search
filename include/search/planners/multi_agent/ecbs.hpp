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
#include <search/planners/wastar.hpp>
#include <search/planners/best_first_search.hpp>
#include <search/planners/multi_agent/cbs.hpp>

#include "search/action_space/constrained_action_space.hpp"

/*
This is a first iteration on ECBS. We start with a hack: setting the f value of the high level nodes to a weighted sum of the SOC and the number of conflicts.

This class should not be derived from BestFirstSearch, but rather from FocalSearch, which needs to be implemented still.
*/

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

    /// @brief The weight of the number of conflicts in the f value of the high level nodes.
    double weight_num_conflicts = 1.0;
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
    void initializePlanner(std::vector<std::shared_ptr<ConstrainedActionSpace>>& action_space_ptrs,
                           const std::vector<StateType>& starts, const std::vector<StateType>& goals);

    /// @brief Initialize the planner and set the agent names.
    /// @param action_spaces_ptr The action space. The action spaces of all agents must be pointing to the same scene interface.
    /// @param agent_names The names of the agents.
    /// @param starts The start states for all agents.
    /// @param goals The goal states for all agents.
    void initializePlanner(std::vector<std::shared_ptr<ConstrainedActionSpace>>& action_space_ptrs, const std::vector<std::string>& agent_names, const std::vector<StateType>& starts, const std::vector<StateType>& goals);

    /// @brief plan a path
    /// @param path The path
    /// @return whether the plan was successful or not
    bool plan(MultiAgentPaths& paths);

protected:

    /// @brief Generate descendents of a state, a key method in most search algorithms.
    /// @param state_id
    void expand(int state_id) override;

    // Public variable. For shadowing.
    /// @brief The conflict types that this algorithm asks for from the action space.
    std::vector<ConflictType> conflict_types_ = {ConflictType::EDGE, ConflictType::VERTEX};

    /// Member variables.
    // The search parameters.
    ECBSParams params_;

    // /// @brief The search state compare struct.
    // struct SearchStateCompareFhat{
    //     bool operator()(const ECBS::SearchState& s1, const ECBS::SearchState& s2) const{
    //          if (s1.unresolved_conflicts.size() == s2.unresolved_conflicts.size()){
    //             return ECBS::SearchStateCompare()(s1, s2);
    //          }
    //          else {
    //                 return s1.unresolved_conflicts.size() < s2.unresolved_conflicts.size();
    //          }
    //     }
    // };

    // // FOCAL priority queue.
    // /// @class ECBSFocalQueue
    // /// @brief A prioirity queue with a FOCAL list.
    // class ECBSFocalQueue {

    // private:
    // /// @brief The open list. We set it to a deque for fast pop_front().
    // using OpenList = ::smpl::IntrusiveHeap<ECBS::SearchState, SearchStateCompare>;
    // OpenList open_;

    // /// @brief The open list. We set it to a deque for fast pop_front().
    // using FocalList = ::smpl::IntrusiveHeap<ECBS::SearchState, SearchStateCompareFhat>;
    // FocalList focal_;

    // public:
    //     bool empty() const{
    //         return open_.empty() && focal_.empty();
    //     }

    //     size_t size() const;
    //     void clear();
    //     void push(ECBS::SearchState* e); // TODO(rishi): make sure search state is the right one for ECBS.
    //     void pop();
    //     bool contains(ECBS::SearchState* e);
    //     void update(ECBS::SearchState* e);
    

    // };

    
    // };


};

}  // namespace ims

#endif  // SEARCH_ECBS_HPP
