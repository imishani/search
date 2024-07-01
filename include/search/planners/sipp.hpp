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
 * \file   sipp.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   2024-01-16
*/
#pragma once

// standard includes
#include <functional>
#include <unordered_map>
#include <utility>
// Standard includes
#include <utility>
#include <algorithm>
// Project includes.
#include <search/planners/best_first_search.hpp>
#include <search/action_space/constrained_action_space.hpp>

namespace ims{

    /// @class AStarParams class.
    /// @brief The parameters for the AStar algorithm
    struct SIPPParams : public BestFirstSearchParams{

        /// @brief Constructor
        /// @param heuristic The heuristic function. Passing the default heuristic function will result in a uniform cost search
        explicit SIPPParams(BaseHeuristic* heuristic,
                              double epsilon) : BestFirstSearchParams(heuristic) {
            this->epsilon = epsilon;
        }

        /// @brief Destructor
        ~SIPPParams() override = default;

        /// @brief The epsilon value for weighing the heuristic function.
        double epsilon;
    };

    /// @class SIPP class. Weighted A* algorithm
    /// @brief A weighted A* algorithm implementation. This algorithm is a modification of the A* algorithm that
    /// uses inflation of the heuristic function to find a solution with a cost that is within a factor of epsilon
    /// of the optimal solution (epsilon-sub-optimality).
    class SIPP : public BestFirstSearch{

    private:

        /// @brief The search state.
        struct SearchState: public ims::BestFirstSearch::SearchState{
            /// @brief The heuristic value
            double h {-1};

            /// @note The arrival time is the g-value of the node. This assumes inegral time steps.

            /// @brief The safe interval. Safe between [t1, t2] inclusive.
            std::pair<TimeType, TimeType> safe_interval;

            /// @brief The configuration id.
            int cfg_state_id {UNSET};
        };

        // Private members.
        /// @brief The open list.
        using OpenList = ::smpl::IntrusiveHeap<SearchState, SearchStateCompare>;
        OpenList open_;
        /// @brief The action space.
        std::shared_ptr<ConstrainedActionSpace> action_space_ptr_;

        /// @brief The list of search states.
        std::vector<SearchState*> states_;

        /// @brief The ids of the start and goal configurations. Different from regular search, in SIPP each configuration may have more than one state. One for each safe interval.
        int start_cfg_id_;
        int goal_cfg_id_;

        /// @brief A mapping between a configuration id all its corresponding search state ids.
        std::unordered_map<int, std::vector<int>> cfg_id_to_state_ids_;

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

    public:
        /// @brief Constructor
        /// @param params The parameters
        explicit SIPP(const SIPPParams &params);

        /// @brief Destructor
        ~SIPP() override;

        /// @brief Initialize the planner
        /// @param action_space_ptr The action space. Action type within this object must not be timed.
        /// @param starts Vector of start states, timed.
        /// @param goals Vector of goal states, timed.
        inline void initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
                                       const std::vector<StateType>& starts,
                                       const std::vector<StateType>& goals) override {
            throw std::runtime_error("SIPP requires a ConstrainedActionSpace. Use initializePlanner(const std::shared_ptr<ConstrainedActionSpace>& action_space_ptr, const std::vector<StateType>& starts, const std::vector<StateType>& goals) instead.");                           
            }

        /// @brief Initialize the planner
        /// @param action_space_ptr The action space
        /// @param start The start state
        /// @param goal The goal state
        inline void initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
                               const StateType& start, const StateType& goal) override{
            throw std::runtime_error("SIPP requires a ConstrainedActionSpace. Use initializePlanner(const std::shared_ptr<ConstrainedActionSpace>& action_space_ptr, const std::vector<StateType>& starts, const std::vector<StateType>& goals) instead.");                           
        }

        /// @brief Initialize the planner
        /// @param action_space_ptr The action space, the action type within this object must not be timed.
        /// @param start The start state, has time in its last component. Often set to 0.
        /// @param goal The goal state, has time in its last component. Often set to -1.
        void initializePlanner(const std::shared_ptr<ConstrainedActionSpace>& action_space_ptr,
                               const StateType& start, const StateType& goal);

        /// @brief plan a path
        /// @param path The path
        /// @return if the plan was successful or not
        bool plan(std::vector<StateType> &path) override;

        /// @brief Reset the planner
        void resetPlanningData() override;

        /// @brief Get all the search states.
        /// @return The states.
        auto getAllSearchStates() -> std::vector<SearchState*>;

        /// @brief Get the heuristic value of a state.
        double computeHeuristic(int state_id) override ;
        double computeHeuristic(int s1_id, int s2_id) override ;
    
        
    protected:
        /// @brief Expand a state. Adding valid successors to open.
        /// @param state_id 
        void expand(int state_id) override;

        /// @brief Reconstruct the path (and optionally also get the transition costs.)
        /// @param path The path to be populated
        /// @param costs The costs to be populated. Cost at index i is the cost of the transition from state i to state i+1. Thus, the cost at the goal state is zero as there is no transition from the goal state.
        void reconstructPath(std::vector<StateType>& path, std::vector<double>& costs) override;
        void reconstructPath(std::vector<StateType>& path) override;

        SIPPParams params_;

    };

}


