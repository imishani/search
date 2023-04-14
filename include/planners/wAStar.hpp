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
 * \file   wAStar.hpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   3/28/23
*/

#ifndef SEARCH_WASTAR_HPP
#define SEARCH_WASTAR_HPP

// standard includes
#include <functional>
// Standard includes
#include <utility>
#include <algorithm>

// project includes
#include <planners/BestFirstSearch.hpp>

namespace ims{

    /// @class AStarParams class.
    /// @brief The parameters for the AStar algorithm
    struct wAStarParams : public BestFirstSearchParams{

        /// @brief Constructor
        /// @param heuristic The heuristic function. Passing the default heuristic function will result in a uniform cost search
        explicit wAStarParams(Heuristic &heuristic,
                              double &epsilon) : BestFirstSearchParams(heuristic) {
            this->epsilon = epsilon;
        }

        /// @brief Destructor
        ~wAStarParams() override = default;

        double epsilon;

    };


    /// @class wAStar class. Weighted A* algorithm
    /// @brief A weighted A* algorithm implementation. This algorithm is a modification of the A* algorithm that
    /// uses inflation of the heuristic function to find a solution with a cost that is within a factor of epsilon
    /// of the optimal solution (epsilon-suboptimality).
    class wAStar : public BestFirstSearch{
    public:
        /// @brief Constructor
        /// @param params The parameters
        explicit wAStar(const wAStarParams &params);

        /// @brief Destructor
        ~wAStar() override = default;

        /// @brief Initialize the planner
        /// @param actionSpacePtr The action space
        /// @param start The start state
        /// @param goal The goal state
        void initializePlanner(const std::shared_ptr<actionSpace>& actionSpacePtr,
                               const stateType& start, const stateType& goal) override;


    protected:

        void setStateVals(state* state_, state* parent, double cost) override;

        void expand(state* state_) override;

        Heuristic m_heuristicFunction;

        wAStarParams m_params;

    };

}


#endif //SEARCH_WASTAR_HPP