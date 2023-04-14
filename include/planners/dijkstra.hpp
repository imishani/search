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


#ifndef SEARCH_DIJKSTRA_HPP
#define SEARCH_DIJKSTRA_HPP

#include <planners/AStar.hpp>
#include <heuristics/standardHeu.hpp>

namespace ims{

    struct dijkstraParams : public AStarParams{

        /// @brief Constructor
        /// @param heuristic The heuristic function. Passing the default heuristic function will result in a uniform cost search
        dijkstraParams(zeroHeuristic* heuristic) : AStarParams(heuristic) {
        }

        /// @brief Destructor
        ~dijkstraParams() override = default;

    };


    class dijkstra : public AStar{
    public:
        /// @brief Constructor
        /// @param params The parameters
        explicit dijkstra(const dijkstraParams &params);

        /// @brief Destructor
        ~dijkstra() override = default;

        bool exhaustPlan();

    };

}


#endif //SEARCH_DIJKSTRA_HPP
