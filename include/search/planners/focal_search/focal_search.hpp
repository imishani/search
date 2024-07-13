/*
 * Copyright (C) 2023, Yorai Shaoul
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
 * \file   focal_search.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   Oct 05 2023
 */

#pragma once

// standard includes
#include <functional>
// Standard includes
#include <algorithm>
#include <utility>

// project includes
#include <search/action_space/subcost_action_space.hpp>
#include <search/common/queue_general.hpp>
#include <search/heuristics/base_heuristic.hpp>
#include <search/planners/planner.hpp>

namespace ims {

/// @class FocalSearch Parameters
struct FocalSearchParams : public PlannerParams {
    /// @brief Constructor
    explicit FocalSearchParams(BaseHeuristic* heuristic) : PlannerParams(), heuristic_(heuristic) {}
    explicit FocalSearchParams(BaseHeuristic* heuristic, double focal_suboptimality) : PlannerParams(), heuristic_(heuristic), focal_suboptimality(focal_suboptimality) {}

    /// @brief Destructor
    ~FocalSearchParams() override = default;

    BaseHeuristic* heuristic_ = nullptr;

    /// @brief The focal queueu suboptimality bound. This is a number greater than 1, used to bound the number of states that are put in the FOCAL queue.
    double focal_suboptimality = 2.0;
};

struct FocalSearchPlannerStats : public PlannerStats {
    double focal_suboptimality{1};  // AKA: epsilon
    double lower_bound{0}; // A lower bound on the cost of the solution.
};

/// @class FocalSearch class.
/// @brief A general search algorithm that uses heuristics and g values to find the optimal path
class FocalSearch : public Planner {
private:
    friend class FocalwAStar; friend class FocalEAwAStarUniformCost;
    friend class FocalwSIPP;

    /// @brief The search state.
    struct SearchState : public ims::SearchState, ims::SearchStateLowerBoundMixin {
        /// @brief The parent state
        int parent_id = UNSET;
        /// @brief The edge from the parent to the state.
        std::vector<int> edge_from_parent_state_ids;
        /// @brief The edge step-costs from the parent to the state.
        std::vector<double> edge_from_parent_transition_costs;
        /// @brief The cost to come
        double g = INF_DOUBLE;
        /// @brief The heuristic value. Even though not all planners use it, we have it here because many do.
        double h = INF_DOUBLE;
        /// @brief The f value
        double f = INF_DOUBLE;
        /// @brief The c-value. The value used to order states in focal.
        double c = INF_DOUBLE;
        /// @brief open list boolean
        bool in_open = false;
        /// @brief closed list boolean
        bool in_closed = false;

        /// @brief set the state to open list (make sure it is not in closed list and if it is, update it)
        void setOpen() {
            in_open = true;
            in_closed = false;
        }

        /// @brief set the state to closed list (make sure it is not in open list and if it is, update it)
        void setClosed() {
            in_closed = true;
            in_open = false;
        }

        void print() override {
            std::cout << "State: " << state_id << " Parent: " << parent_id << " g: " << g << " f: " << f << std::endl;
        }

        double getLowerBound() const override {
            return f;
        }
    };

    /// @brief The search state compare struct.
    struct SearchStateCompare {
        bool operator()(const SearchState& s1, const SearchState& s2) const {
            if ((s1.f == s2.f) && (s1.g == s2.g))
                return (s1.state_id < s2.state_id);
            else if (s1.f == s2.f)
                // For tie breaking, we prefer the state with the larger g value as it is closer to the goal (lower h in the case of an informed search).
                return s1.g > s2.g;
            else
                return s1.f < s2.f;
        }
    };

    /// @brief The search state compare struct for the FOCAL portion of the queue.
    struct SearchStateFocalCompare {
        bool operator()(const SearchState& s1, const SearchState& s2) const {
            if (s1.c == s2.c)
                return (s1.f < s2.f);
            else
                return s1.c < s2.c;
        }
    };

    /// @brief The parameters.
    FocalSearchParams params_;

    /// @brief The open list is a focal queue.
    FocalAndAnchorQueueWrapper<SearchState, SearchStateCompare, SearchStateFocalCompare> open_;

    /// @brief Keeping track of states by their id.
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
    explicit FocalSearch(const FocalSearchParams& params);

    /// @brief Destructor
    ~FocalSearch() override;

    /// @brief Initialize the planner
    /// @param action_space_ptr The action space
    /// @param starts Vector of start states
    /// @param goals Vector of goal states
    virtual void initializePlanner(const std::shared_ptr<SubcostActionSpace>& action_space_ptr,
                                   const std::vector<StateType>& starts,
                                   const std::vector<StateType>& goals);
    inline void initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
                                  const std::vector<StateType>& starts,
                                  const std::vector<StateType>& goals) override { throw std::runtime_error("FocalSearch::initializePlanner() called with an ActionSpace that is not a SubcostActionSpace."); }

    /// @brief Initialize the planner
    /// @param action_space_ptr The action space
    /// @param start The start state
    /// @param goal The goal state
    virtual void initializePlanner(const std::shared_ptr<SubcostActionSpace>& action_space_ptr,
                                   const StateType& start, const StateType& goal);
//    inline void initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
//                                  const StateType& start, const StateType& goal) override {
//        throw std::runtime_error("FocalSearch::initializePlanner() called with an ActionSpace that is not a SubcostActionSpace.");
//    }

    /// @brief plan a path
    /// @param path The path
    /// @return if the plan was successful or not
    bool plan(std::vector<StateType>& path) override;
    /// @brief plan a path and return it with the original transition sequences.
    /// \param seqs_path
    /// \param seqs_transition_costs
    /// \return
    virtual bool plan(std::vector<PathType>& seqs_path, std::vector<std::vector<double>>& seqs_transition_costs);

    void resetPlanningData() override;

    inline FocalSearchPlannerStats reportStats(){ return stats_;}
    inline FocalSearchPlannerStats getStats(){ return stats_;}

protected:
    /// @brief
    /// @param state_id The id of the state.
    /// @param parent_id The id of the parent state.
    /// @param cost The cost associated with the search state. For example, in A*, this would be the f value.
    virtual void setStateVals(int state_id, int parent_id, double cost, double subcost);

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
    void reconstructPath(std::vector<StateType>& path, std::vector<double>& transition_costs) override;

    /// @brief Reconstruct the path while keeping information about the sequence transition from the parent states to the child states (alongside the transition costs).
    /// \param seq_states_to_child Each element is a sequence of states from the parent state [i] to the child state [i+1]. Last one is empty.
    /// \param seq_transition_costs_to_child The sequence of transition costs from the parent state to the child state. The cost of transitioning from [i] to [i+1] is the sum of the costs in seq_transition_costs_to_child[i].
    void reconstructPath(std::vector<PathType>& seq_states_to_child,
                         std::vector<std::vector<double>> & seq_transition_costs_to_child);

    bool isGoalState(int state_id) override;

    BaseHeuristic* heuristic_ = nullptr;

    // The action space.
    std::shared_ptr<SubcostActionSpace> action_space_ptr_;

    // The stats.
    FocalSearchPlannerStats stats_;

};  // class FocalSearch

}  // namespace ims
