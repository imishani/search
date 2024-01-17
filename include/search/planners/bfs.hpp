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
 * \file   bfs.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   07/07/23
 */

#ifndef SEARCH_BFS_HPP
#define SEARCH_BFS_HPP

// Standard includes.
#include <functional>
#include <algorithm>
#include <utility>
#include <deque>

// Project includes.
#include <search/planners/best_first_search.hpp>
#include <search/heuristics/standard_heuristics.hpp>

namespace ims {

/// @class BFSParams class.
/// @brief The parameters for the BFS algorithm
struct BFSParams : public BestFirstSearchParams {
    /// @brief Constructor
    explicit BFSParams() : BestFirstSearchParams(new ims::ZeroHeuristic()){
    }

    /// @brief Destructor
    ~BFSParams() override = default;

    /// @brief Exhaustive search flag. If true, the algorithm will continue to search until the goal is found or the open list is empty.
    bool exhaustive = false;

};

/// @class BFS class.
/// @brief The BFS algorithm.
class BFS : public BestFirstSearch {
private:
    // Make wAStar a friend class for getting access to the search state.
    friend class wAStar;

    /// @brief The search state.
    struct SearchState : public ims::BestFirstSearch::SearchState {
        // No additional members.
    };

    /// @brief The open list. We set it to a deque for fast pop_front().
    using OpenList = std::deque<SearchState*>;
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

public:
    /// @brief Constructor
    /// @param params The parameters
    explicit BFS(const BFSParams& params);

    /// @brief Destructor
    ~BFS() override;

    /// @brief Initialize the planner
    /// @param action_space_ptr The action space
    /// @param starts Vector of start states
    /// @param goals Vector of goal states
    void initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
                           const std::vector<StateType>& starts,
                           const std::vector<StateType>& goals) override;

    /// @brief Initialize the planner
    /// @param action_space_ptr The action space
    /// @param start The start state
    /// @param goal The goal state
    void initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
                           const StateType& start, const StateType& goal) override;

    /// @brief plan a path
    /// @param path The path
    /// @return whether the plan was successful or not
    bool plan(std::vector<StateType>& path) override;

protected:

    /// @brief Set the search state struct values.
    /// @param state_id 
    /// @param parent_id 
    /// @param cost 
    void setStateVals(int state_id, int parent_id, double cost) override;

    /// @brief Generate descendents of a state, a key method in most search algorithms.
    /// @param state_id 
    void expand(int state_id) override;

    /// @brief Reconstruct the path from the goal state to the start state.
    /// @param path 
    void reconstructPath(std::vector<StateType>& path) override;
    void reconstructPath(std::vector<StateType>& path, std::vector<double>& costs) override;

    BFSParams params_;

};

}  // namespace ims

#endif  // SEARCH_BFS_HPP
