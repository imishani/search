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
    private:
        /// @brief The search state.
        struct SearchState {
            /// @brief history of parents
            std::unordered_set<int> parent_ids_;

            struct HeapData : public ims::BestFirstSearch::SearchState{
                SearchState* me;
                /// @brief The heuristic value
                double h {-1};
            };
            HeapData heap_data[1];
        };
        struct HeapCompare {
            bool operator()(const SearchState::HeapData& s1,
                    const SearchState::HeapData& s2) const {
                if ((s1.f == s2.f) && (s1.g == s2.g))
                    return (s1.state_id < s2.state_id);
                else if (s1.f == s2.f)
                    // For tie breaking, we prefer the state with the larger g value as it is closer to the goal (lower h in the case of an informed search).
                    return s1.g > s2.g;
                else
                    return s1.f < s2.f;
            }
        };

        /// @brief The open lists
        using OpenList = ::smpl::IntrusiveHeap<SearchState::HeapData, HeapCompare>;
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
        explicit MultiPathwAStar(const MultiPathwAStarParams &params);

        /// @brief Destructor
        ~MultiPathwAStar() override;

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


        /// @brief Calculate a approx f* for ALL the expanded and MANY visited nodes by
        /// reconstructing a backward graph using the cached backpointers.
        std::unordered_map<int, double> reconstructFValue();

        double getGValue(int state_id);

        double getHValue(int state_id);

        StateType getState(int state_id);

        void writeDataToFile(std::string fpath,
                           std::string planner_name,
                           std::string map_id,
                           std::vector<double> start,
                           std::vector<double> goal,
                           double epsilon,
                           std::unordered_map<int, double>& sid_to_fval);

    protected:

        void setStateVals(int state_id, int parent_id, double cost) override;

        void expand(int state_id) override;

        std::unordered_map<int, double> Dijkstra(std::unordered_map<int, std::vector<int>>& graph,
                                                 std::unordered_map<int, std::vector<double>>& costs,
                                                 int start_id);

        void reconstructPath(std::vector<StateType>& path) override;

        /// @brief Reconstruct the path and also get the transition costs.
        /// @param path The path to be populated
        /// @param costs The costs to be populated. Cost at index i is the cost of the transition from state i to state i+1. Thus, the cost at the goal state is zero as there is no transition from the goal state.
        void reconstructPath(std::vector<StateType>& path, std::vector<double>& costs) override;


        MultiPathwAStarParams params_;

    };

}

#endif //SEARCH_MULTI_PATH_WASTAR_HPP
