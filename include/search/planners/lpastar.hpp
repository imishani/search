/*
 * Copyright (C) 2024, Itamar Mishani
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
 * \file   lpastar.hpp
 * \author Carina Sanborn (czsanbor@andrew.cmu.edu)
 * \date   2/9/24
*/

#ifndef SEARCH_LPASTAR_HPP
#define SEARCH_LPASTAR_HPP

// standard includes
#include <functional>
// Standard includes
#include <utility>
#include <algorithm>
// project includes
#include <search/planners/best_first_search.hpp>

namespace ims{

    /// @class LPAStarParams class.
    /// @brief The parameters for the LPAStar algorithm
    struct LPAStarParams : public BestFirstSearchParams{

        /// @brief Constructor
        /// @param heuristic The heuristic function. Passing the default heuristic function will result in a uniform cost search
        explicit LPAStarParams(BaseHeuristic* heuristic) : BestFirstSearchParams(heuristic) {}

        /// @brief Destructor
        ~LPAStarParams() override = default;

    };

    /// @class LPAStar class. Lifelong Planning A* algorithm
    /// @brief A Lifelong Planning A* algorithm implementation.  LPA* is an incremental version of A*. 
    /// It applies to finite graph search problems on known graphs whose edge costs increase or decrease over time (which can also be used to
    /// model edges or vertices that are added or deleted).
    class LPAStar : public BestFirstSearch{

    private:

        friend class AStar; friend class Dijkstra;
        friend class ExperienceWAstar; friend class EAwAStarUniformCost;

        /// @brief The search state.
        struct SearchState: public ims::BestFirstSearch::SearchState{
            /// @brief The heuristic value
            double h {-1};
            double rhs {-1};
        };

        /// @brief The open list.
        using OpenList = ::smpl::IntrusiveHeap<SearchState, SearchStateCompare>;
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
        explicit LPAStar(const LPAStarParams &params);

        /// @brief Destructor
        ~LPAStar() override;

         /// @brief Calculte the key pair to insert in priority queue
         /// @param s A search state
         /// @return A pair of keys to insert into priority queue
        std::pair<double, double> calculateKeys(const ims::LPAStar::SearchState *s);

        /// @brief Initialize the priority queue and g(s) and rhs(s) values 
        void initialize();



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
        /// @return if the plan was successful or not
        bool plan(std::vector<StateType> &path) override;

        /// @brief Reset the planner
        void resetPlanningData() override;

        /// @brief Get all the search states.
        /// @return The states.
        auto getAllSearchStates() -> std::vector<SearchState*>;

        
    protected:

        void setStateVals(int state_id, int parent_id, double cost) override;

        void expand(int state_id) override;

        void reconstructPath(std::vector<StateType>& path) override;

        /// @brief Reconstruct the path and also get the transition costs.
        /// @param path The path to be populated
        /// @param costs The costs to be populated. Cost at index i is the cost of the transition from state i to state i+1. Thus, the cost at the goal state is zero as there is no transition from the goal state.
        void reconstructPath(std::vector<StateType>& path, std::vector<double>& costs) override;

        LPAStarParams params_;

    };

}


#endif //SEARCH_LPASTAR_HPP