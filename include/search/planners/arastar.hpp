/*
 * Copyright (C) 2023, Itamar Mishani
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
 * \file   arastar.hpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   7/21/23
*/

#ifndef SEARCH_SEARCH_INCLUDE_SEARCH_PLANNERS_ARASTAR_HPP_
#define SEARCH_SEARCH_INCLUDE_SEARCH_PLANNERS_ARASTAR_HPP_

// standard includes
#include <functional>
// Standard includes
#include <utility>
#include <algorithm>
// project includes
#include <search/planners/wastar.hpp>

namespace ims {

    /// @class ARAStarParams class.
    /// @brief The parameters for the ARAStar algorithm
    struct ARAStarParams : public wAStarParams{

        /// @brief Constructor
        /// @param heuristic The heuristic function. Passing the default heuristic function will result in a uniform cost search
        explicit ARAStarParams(BaseHeuristic* heuristic,
                               double initial_epsilon,
                               double delta_epsilon,
                               double final_epsilon = 1) : wAStarParams(heuristic, initial_epsilon),
                                                           epsilon_delta(delta_epsilon),
                                                           final_epsilon(final_epsilon){
            call_number = 0;
            ara_time_limit = INF_DOUBLE; // default: no time limit
            expansions_limit = INF_INT; // default: no expansion limit
            curr_cost = INF_DOUBLE;
            init_epsilon = initial_epsilon;
        }

        /// @brief Destructor
        ~ARAStarParams() override = default;

        int call_number;
        double final_epsilon, epsilon_delta, init_epsilon;
        double curr_cost;

        enum timing_types {TIME, EXPANSIONS, USER} type = TIME;
        double ara_time_limit; // TIME: time limit for the search
        int expansions_limit; // EXPANSIONS: limit on the number of expansions
        std::function<bool()> timed_out_fun; // USER: function to check if the search timed out

    };

    /// @class ARAStar class (ARA*: Anytime Repairing A*).
    /// @brief The ARA* algorithm is an anytime heuristic search algorithm based on weighted A*.
    /// The algorithm runs a series of weighted A* searches, starting with an (high) initial weight and
    /// decreasing it (decreasing bounds on the suboptimality) to return the best solution found
    /// within a given time bound. This algorithm reuses the search tree from the previous search to
    /// improve the efficiency rather the vanilla case which starts from scratch in each iteration.
    class ARAStar : public wAStar {

        friend class ExperienceARAstar;

    private:
    struct SearchState : public BestFirstSearch::SearchState {
        double h {-1} ;
        double v {INF_DOUBLE};
        unsigned short call_number {};
        bool in_incons {false};
    };

    /// @brief The open list.
    using OpenList = ::smpl::IntrusiveHeap<SearchState, SearchStateCompare>;
    OpenList open_;
    std::vector<SearchState*> incons_;
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
        /// @param params The parameters for the ARAStar algorithm
        explicit ARAStar(const ARAStarParams& params);

        /// @brief Destructor
        ~ARAStar() override;

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

        /// @brief Plan a path
        /// @param path The path to be filled
        /// @return True if a path was found, false otherwise
        bool plan(std::vector<StateType> &path) override;

    protected:

        /// @brief Improve path function (inner loop of the algorithm)
        /// @param path The path to be filled
        /// @return True if a path was found, false otherwise
        bool improvePath(std::vector<StateType> &path);

        void expand(int state_id) override;

        void setStateVals(int state_id, int parent_id, double cost) override;

        void reconstructPath(std::vector<StateType>& path) override;
        void reconstructPath(std::vector<StateType>& path, std::vector<double>& costs) override;

        /// @brief Reorder the open list
        void reorderOpen();

        /// @brief Update all bounds for the next iteration (time and suboptimality)
        virtual void updateBounds();

        /// @brief Reinit search state when in new iteration
        /// @param state The state to be reinitialized
        void reinitSearchState(SearchState* state) const;

        /// @brief Check if the search timed out based on time type
        bool timedOut();

        void resetPlanningData() override;

        ARAStarParams params_;


    };
}

#endif //SEARCH_SEARCH_INCLUDE_SEARCH_PLANNERS_ARASTAR_HPP_
