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

// standard includes
#include <functional>
// Standard includes
#include <algorithm>
#include <utility>

// project includes
#include <search/planners/best_first_search.hpp>

namespace ims {

/// @class AStarParams class.
/// @brief The parameters for the BFS algorithm
struct BFSParams : public BestFirstSearchParams {
    /// @brief Constructor
    /// @param heuristic The heuristic function. Passing the default heuristic function will result in a uniform cost search
    explicit BFSParams(BaseHeuristic* heuristic) : BestFirstSearchParams(heuristic){
    }

    /// @brief Destructor
    ~BFSParams() override = default;

    /// @brief Exhaustive search flag. If true, the algorithm will continue to search until the goal is found or the open list is empty.
    bool exhaustive = false;

};

/// @class BFS class. Weighted A* algorithm
/// @brief A weighted A* algorithm implementation. This algorithm is a modification of the A* algorithm that
class BFS : public BestFirstSearch {
private:
    friend class AStar;
    /// @brief The search state.
    struct SearchState : public ims::BestFirstSearch::SearchState {
        /// @brief The heuristic value
    };

    /// @brief The open list.
    using OpenList = smpl::IntrusiveHeap<SearchState, SearchStateCompare>;
    OpenList open_;

    std::vector<SearchState*> states_;

    /// @brief Get the state by id
    /// @param state_id The id of the state
    /// @return The state
    /// @note Use this function only if you are sure that the state exists
    auto getSearchState(int state_id) -> SearchState*;

    /// @brief Get the state by id or create a new one if it does not exist
    /// @param state_id The id of the state
    /// @return The state
    auto getOrCreateSearchState(int state_id) -> SearchState*;

public:
    /// @brief Constructor
    /// @param params The parameters
    explicit BFS(const BFSParams& params);

    /// @brief Destructor
    ~BFS() override = default;

    /// @brief Initialize the planner
    /// @param actionSpacePtr The action space
    /// @param start The start state
    /// @param goal The goal state
    void initializePlanner(const std::shared_ptr<ActionSpace>& actionSpacePtr,
                           const StateType& start, const StateType& goal) override;

    /// @brief plan a path
    /// @param path The path
    /// @return if the plan was successful or not
    bool plan(std::vector<StateType>& path) override;

protected:
    void setStateVals(int state_id, int parent_id, double cost) override;

    void expand(int state_id) override;

    void reconstructPath(std::vector<StateType>& path) override;

    BFSParams params_;

    bool exhaustive_;
};

}  // namespace ims

#endif  // SEARCH_BFS_HPP
