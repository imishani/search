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
 * \file   planner.hpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   3/28/23
*/


#ifndef SEARCH_BESTFIRSTSEARCH_HPP
#define SEARCH_BESTFIRSTSEARCH_HPP

// standard includes
#include <functional>
// Standard includes
#include <utility>
#include <algorithm>

// project includes
#include <planners/planner.hpp>
#include <common/base_heuristic.hpp>

namespace ims{

    /// @class BestFirstSearch Parameters
    /// @note Before initializing the planner, the heuristic function must be set
    /// So you define a heuristic function and then pass it to the constructor of the BestFirstSearchParams
    /// @note Since this is general BestFS, the heuristic function returns an f value!
    struct BestFirstSearchParams : public PlannerParams{
        /// @brief Constructor
        explicit BestFirstSearchParams(BaseHeuristic* heuristic) : PlannerParams(), heuristic_(heuristic) {}

        /// @brief Destructor
        ~BestFirstSearchParams() override = default;

        BaseHeuristic* heuristic_ = nullptr;
    };

    /// @class BestFirstSearch class.
    /// @brief A general search algorithm that uses heuristics and g values to find the optimal path
    class BestFirstSearch : public Planner{
    public:
        /// @brief Constructor
        /// @param params The parameters
        explicit BestFirstSearch(const BestFirstSearchParams &params);

        /// @brief Destructor
        ~BestFirstSearch() override = default;

        /// @brief Initialize the planner
        /// @param actionSpacePtr The action space
        /// @param start The start state
        /// @param goal The goal state
        void initializePlanner(const std::shared_ptr<ActionSpace>& actionSpacePtr,
                               const StateType& start, const StateType& goal) override;

        /// TODO: Do I need this function?
        /// @brief Get the state by id
        /// @param state_id The id of the state
        /// @return The state
        virtual State* getState(size_t state_id);

        /// @brief Compute the heuristic value of from state s to the goal state
        /// @param s The state
        virtual double computeHeuristic(State* s);

        /// @brief Compute the heuristic value from state s1 to state s2
        /// @param s1 The state
        /// @param s2 The state
        virtual double computeHeuristic(State* s1, State* s2);

        /// @brief plan
        /// @param path The path
        /// @return if the plan was successful or not
        bool plan(std::vector<State*>& path) override;

    protected:

        virtual void setStateVals(State* state_, State* parent, double cost);

        void expand(State* state_) override;

        void reconstructPath(std::vector<State*>& path) override;

        bool isGoalState(const State& s) override;

        BaseHeuristic* heuristic_ = nullptr;

    };

}




#endif //SEARCH_BESTFIRSTSEARCH_HPP
