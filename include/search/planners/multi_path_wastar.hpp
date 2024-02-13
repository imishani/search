//  * Copyright (c) 2024, Ramkumar Natarajan
//  * All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without
//  * modification, are permitted provided that the following conditions are met:
//  *
//  *     * Redistributions of source code must retain the above copyright
//  *       notice, this list of conditions and the following disclaimer.
//  *     * Redistributions in binary form must reproduce the above copyright
//  *       notice, this list of conditions and the following disclaimer in the
//  *       documentation and/or other materials provided with the distribution.
//  *     * Neither the name of the Carnegie Mellon University nor the names of its
//  *       contributors may be used to endorse or promote products derived from
//  *       this software without specific prior written permission.
//  *
//  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
//  * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  * POSSIBILITY OF SUCH DAMAGE.
//

/*!
 * \file multi_path_wastar.hpp
 * \author Ram Natarajan (rnataraj@cs.cmu.edu)
 * \date 2/12/24
*/

#ifndef SEARCH_MULTI_PATH_WASTAR_HPP
#define SEARCH_MULTI_PATH_WASTAR_HPP

// standard includes
#include <functional>
// Standard includes
#include <utility>
#include <algorithm>
// project includes
#include <search/planners/wastar.hpp>

namespace ims{

    /// @class AStarParams class.
    /// @brief The parameters for the AStar algorithm
    struct MultiPathwAStarParams : public wAStarParams{

        /// @brief Constructor
        /// @param heuristic The heuristic function. Passing the default heuristic function will result in a uniform cost search
        explicit MultiPathwAStarParams(BaseHeuristic* heuristic,
                              double epsilon) : wAStarParams(heuristic, epsilon) {
            this->epsilon = epsilon;
        }

        /// @brief Destructor
        ~MultiPathwAStarParams() override = default;

        double epsilon;

    };

    /// @class MultiPathwAStar class. Weighted A* algorithm
    /// @brief A weighted A* algorithm implementation. This algorithm is a modification of the A* algorithm that
    /// uses inflation of the heuristic function to find a solution with a cost that is within a factor of epsilon
    /// of the optimal solution (epsilon-suboptimality).
    class MultiPathwAStar : public wAStar{

    public:
        /// @brief Constructor
        /// @param params The parameters
        explicit MultiPathwAStar(const MultiPathwAStarParams &params);

        /// @brief Destructor
        ~MultiPathwAStar() override;

    protected:

        void expand(int state_id) override;

        void BFSUtil(const std::unordered_map<int, std::unordered_set<int>>& graph, int startVertex);

        void reconstructPath(std::vector<StateType>& path) override;

        /// @brief Reconstruct the path and also get the transition costs.
        /// @param path The path to be populated
        /// @param costs The costs to be populated. Cost at index i is the cost of the transition from state i to state i+1. Thus, the cost at the goal state is zero as there is no transition from the goal state.
        void reconstructPath(std::vector<StateType>& path, std::vector<double>& costs) override;

        MultiPathwAStarParams params_;

    };

}

#endif //SEARCH_MULTI_PATH_WASTAR_HPP
