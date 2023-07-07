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
 * \file   astar.hpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   3/28/23
*/


#ifndef SEARCH_ASTAR_HPP
#define SEARCH_ASTAR_HPP

// standard includes
#include <functional>
// Standard includes
#include <utility>
#include <algorithm>

// project includes
#include <search/planners/wastar.hpp>
#include "search/common/base_heuristic.hpp"

namespace ims{

    /// @class AStarParams class.
    /// @brief The parameters for the AStar algorithm
    struct AStarParams : public BFSParams{

        /// @brief Constructor
        /// @param heuristic The heuristic function. Passing the default heuristic function will result in a uniform cost search
        explicit AStarParams(BaseHeuristic* heuristic) : BFSParams(heuristic, 1.0) {
        }

        /// @brief Destructor
        ~AStarParams() override = default;

    };


    /// @class AStar class.
    /// @brief A* is a best first search algorithm that uses admissible heuristics and g values to find the optimal path
    class AStar : public wAStar{

    public:
        /// @brief Constructor
        /// @param params The parameters
        explicit AStar(const AStarParams &params);
    };
}


#endif //SEARCH_ASTAR_HPP
