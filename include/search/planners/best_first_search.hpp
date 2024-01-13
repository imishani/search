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
 * \file   best_first_search.hpp
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
#include <search/planners/planner.hpp>
#include "search/heuristics/base_heuristic.hpp"

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
    private:

        friend class AStar; friend class wAStar; friend class EAwAStarUniformCost;
        friend class BFS; friend class ARAStar;
        friend class plannerZero; friend class CBS; friend class CBSBase;
        friend class CBSSphere3d;
        friend class CBSMP;
        friend class ECBS; friend class ECBSMP;
        friend class EACBS; friend class EACBSMP;
        friend class EAECBS; friend class EAECBSMP;
        friend class GeneralizedCBS;

        /// @brief The search state.
        struct SearchState: public ims::SearchState {

            /// @brief The parent state
            int parent_id = UNSET;
            /// @brief The cost to come
            double g = INF_DOUBLE;
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
                if ((s1.f == s2.f) && (s1.g == s2.g))
                    return (s1.state_id < s2.state_id);
                else if (s1.f == s2.f)
                    // For tie breaking, we prefer the state with the larger g value as it is closer to the goal (lower h in the case of an informed search).
                    return s1.g > s2.g;
                else
                    return s1.f < s2.f;
            }
        };
        /// @brief The open list.
        using OpenList = ::smpl::IntrusiveHeap<SearchState, SearchStateCompare>;
        OpenList open_;

        std::vector<SearchState*> states_;

        /// @brief Get the state by id
        /// @param state_id The id of the state
        /// @return The state
        /// @note Use this function only if you are sure that the state exists.
        auto getSearchState(int state_id) -> SearchState*;

        /// @brief Get the state by id or create a new one if it does not exist. If a search state does not exist yet and a new one is created, it's ID will be set, and all other member fields will initialize to default values.
        /// @param state_id The id of the state
        /// @return The state
        auto getOrCreateSearchState(int state_id) -> SearchState*;

    public:
        /// @brief Constructor
        /// @param params The parameters
        explicit BestFirstSearch(const BestFirstSearchParams &params);

        /// @brief Destructor
        ~BestFirstSearch() override;

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
        bool plan(std::vector<StateType>& path) override;

        void resetPlanningData() override;

    protected:

        /// @brief 
        /// @param state_id The id of the state.
        /// @param parent_id The id of the parent state.
        /// @param cost The cost associated with the search state. For example, in A*, this would be the f value.
        virtual void setStateVals(int state_id, int parent_id, double cost);

        /// @brief Compute the heuristic value of from state s to the goal state
        /// @param s The state
        virtual double computeHeuristic(int state_id);

        /// @brief Compute the heuristic value from state s1 to state s2
        /// @param s1 The state
        /// @param s2 The state
        virtual double computeHeuristic(int s1_id, int s2_id);

        /// @brief Expand the current state
        virtual void expand(int state_id);

        void reconstructPath(std::vector<StateType>& path) override;
        void reconstructPath(std::vector<StateType>& path, std::vector<double>& costs) override;

        bool isGoalState(int state_id) override;

        BaseHeuristic* heuristic_ = nullptr;


    }; // class BestFirstSearch

}




#endif //SEARCH_BESTFIRSTSEARCH_HPP
