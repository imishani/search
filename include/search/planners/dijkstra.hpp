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

#include <search/planners/astar.hpp>
#include <search/heuristics/standard_heuristics.hpp>

namespace ims{

    struct dijkstraParams : public AStarParams{

        /// @brief Constructor
        /// @param heuristic The heuristic function. Passing the default heuristic function will result in a uniform cost search
        explicit dijkstraParams(ZeroHeuristic* heuristic) : AStarParams(heuristic) {
        }

        /// @brief Destructor
        ~dijkstraParams() override = default;

    };


    class dijkstra : public AStar{

    private:
        /// @brief The search state.
        struct SearchState: public ims::SearchState{

            /// @brief The parent state
            int parent_id = UNSET;
            /// @brief The cost to come
            double g = INF_DOUBLE;
            /// @brief The heuristic value
            double h = -1;
            /// @brief The f value
            double f = INF_DOUBLE;
            /// @brief open list boolean
            bool in_open = false;
            /// @brief closed list boolean
            bool in_closed = false;

            /// @brief set the state to open list (make sure it is not in closed list and if it is, update it)
            void setOpen(){
                in_open = true;
                in_closed = false;
            }

            /// @brief set the state to closed list (make sure it is not in open list and if it is, update it)
            void setClosed(){
                in_closed = true;
                in_open = false;
            }

            void print() override{
                std::cout << "State: " << state_id << " Parent: " << parent_id << " g: " << g << " f: " << f << std::endl;
            }
        };

        /// @brief The search state compare struct.
        struct SearchStateCompare{
            bool operator()(const SearchState& s1, const SearchState& s2) const{
                return s1.f < s2.f;
            }
        };

        /// @brief The open list.
        using OpenList =  smpl::IntrusiveHeap<SearchState, SearchStateCompare>;
        OpenList open_;

        std::vector<SearchState*> states_;

        /// TODO: Do I need this function?
        /// @brief Get the state by id
        /// @param state_id The id of the state
        /// @return The state
        auto getSearchState(size_t state_id) -> SearchState*;

    public:
        /// @brief Constructor
        /// @param params The parameters
        explicit dijkstra(const dijkstraParams &params);

        /// @brief Destructor
        ~dijkstra() override;

        bool exhaustPlan();

    };

}


#endif //SEARCH_DIJKSTRA_HPP
