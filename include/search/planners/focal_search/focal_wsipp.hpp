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
 * \file   focal_wsipp.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   Jun 01 2024
*/
#pragma once

// standard includes
#include <functional>
// Standard includes
#include <utility>
#include <algorithm>
// Project includes.
#include <search/planners/focal_search/focal_search.hpp>
#include <search/planners/sipp.hpp>

namespace ims{

    /// @class AStarParams class.
    /// @brief The parameters for the AStar algorithm
    struct FocalwSIPPParams : public FocalSearchParams{

        /// @brief Constructor
        /// @param heuristic The heuristic function. Passing the default heuristic function will result in a uniform cost search
        explicit FocalwSIPPParams(BaseHeuristic* heuristic,
                              double focal_suboptimality,
                              double epsilon) : FocalSearchParams(heuristic, focal_suboptimality) {
            this->epsilon = epsilon;
            this->focal_suboptimality = focal_suboptimality;
        }

        /// @brief Destructor
        ~FocalwSIPPParams() override = default;

        /// @brief The weight for the heuristic function.
        double epsilon;
    };

    /// @class FocalwSIPP class. Weighted A* algorithm with a FOCAL queue.
    /// @brief A weighted A* algorithm implementation. This algorithm is a modification of the A* algorithm that
    /// uses inflation of the heuristic function to find a solution with a cost that is within a factor of epsilon
    /// of the optimal solution (epsilon-suboptimality).
    class FocalwSIPP : virtual public FocalSearch {

    private:

        // friend class FocalAStar; friend class FocalDijkstra;
        // friend class FocalExperienceWAstar; friend class FocalEAwAStarUniformCost;

    public:
        /// @brief Constructor
        /// @param params The parameters
        explicit FocalwSIPP(const FocalwSIPPParams &params);

        /// @brief Destructor
        ~FocalwSIPP() override;

        /// @brief Initialize the planner
        /// @param action_space_ptr The action space
        /// @param starts Vector of start states
        /// @param goals Vector of goal states
        void initializePlanner(const std::shared_ptr<SubcostActionSpace>& action_space_ptr,
                               const std::vector<StateType>& starts,
                               const std::vector<StateType>& goals) override;

        /// @brief Initialize the planner
        /// @param action_space_ptr The action space
        /// @param start The start state
        /// @param goal The goal state
        void initializePlanner(const std::shared_ptr<SubcostActionSpace>& action_space_ptr,
                               const StateType& start, const StateType& goal) override;

        /// @brief plan a path
        /// @param path The path
        /// @return if the plan was successful or not
//        bool plan(std::vector<StateType> &path) override;

        void setStateVals(int state_id, int parent_id, double cost, double subcost) override;

        /// @brief Get a search state given its configuration-id and safe interval.
        /// @param cfg_state_id The configuration id.
        /// @param safe_interval The safe interval.
        /// @return The id of the search state.
        auto getOrCreateSearchStateFromCfgIdAndSafeInterval(int cfg_state_id, const SafeIntervalType& safe_interval) -> SearchState* ;

        /// @brief Get the state by id
        /// @param state_id The id of the state
        /// @return The state
        /// @note Use this function only if you are sure that the state exists
        auto getSearchState(int state_id) -> SearchState*;

        /// @brief Get the state by id or create a new one if it does not exist
        /// @param state_id The id of the state
        /// @return The state
        auto getOrCreateSearchState(int state_id) -> SearchState*;


    protected:

        void expand(int state_id) override;

        FocalwSIPPParams params_;
    };

}


