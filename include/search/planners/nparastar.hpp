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
 * \file   NPARAStar.hpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   7/21/23
 */

#ifndef SEARCH_SEARCH_INCLUDE_SEARCH_PLANNERS_NPARASTAR_HPP_
#define SEARCH_SEARCH_INCLUDE_SEARCH_PLANNERS_NPARASTAR_HPP_

// standard includes
#include <functional>
// Standard includes
#include <utility>
#include <algorithm>
// project includes
#include <search/planners/arastar.hpp>

namespace ims
{

    /// @class NPARAStarParams class.
    /// @brief The parameters for the NPARAStar algorithm
    struct NPARAStarParams : public ARAStarParams {
        /// @brief Constructor
        /// @param heuristic The heuristic function. Passing the default heuristic function will result in a uniform cost search
        explicit NPARAStarParams(BaseHeuristic *heuristic, double init_epsilon = 10e3) : ARAStarParams(heuristic, init_epsilon, 0) {}

        /// @brief Destructor
        ~NPARAStarParams() override = default;

    };

    /// @class NPARAStar class (ARA*: Anytime Repairing A*).
    /// @brief The ARA* algorithm is an anytime heuristic search algorithm based on weighted A*.
    /// The algorithm runs a series of weighted A* searches, starting with an (high) initial weight and
    /// decreasing it (decreasing bounds on the suboptimality) to return the best solution found
    /// within a given time bound. This algorithm reuses the search tree from the previous search to
    /// improve the efficiency rather the vanilla case which starts from scratch in each iteration.
    class NPARAStar : public ARAStar {

    public:
        /// @brief Constructor
        /// @param params The parameters for the NPARAStar algorithm
        explicit NPARAStar(const NPARAStarParams &params);


        /// @brief Update all bounds for the next iteration (time and suboptimality)
        void updateBounds() override;

    };
}

#endif // SEARCH_SEARCH_INCLUDE_SEARCH_PLANNERS_NPARASTAR_HPP_
