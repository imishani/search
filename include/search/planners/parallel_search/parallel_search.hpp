/*
 * Copyright (C) 2024, Hanlan Yang
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
 * \file   parallel_search.hpp
 * \author Hanlan Yang (yanghanlan666@gmail.com)
 * \date   July 12 2024
 */

#pragma once

// standard includes
#include <functional>
// Standard includes
#include <algorithm>
#include <future>
#include <utility>

// project includes
#include <search/action_space/action_space.hpp>
#include <search/common/queue_general.hpp>
#include <search/heuristics/base_heuristic.hpp>
#include <search/planners/planner.hpp>

namespace ims {

/// @class ParallelSearch Parameters
struct ParallelSearchParams : public PlannerParams {
    /// @brief Constructor
    explicit ParallelSearchParams(BaseHeuristic* heuristic) : PlannerParams(), heuristic_(heuristic) {}
    explicit ParallelSearchParams(BaseHeuristic* heuristic, int num_threads, double epsilon) : PlannerParams(), heuristic_(heuristic), num_threads_(num_threads), epsilon_(epsilon) {}

    /// @brief Destructor
    ~ParallelSearchParams() override = default;

    BaseHeuristic* heuristic_ = nullptr;
    int num_threads_ = 1;
    double epsilon_ = 1.0;
};

struct ParallelSearchPlannerStats : public PlannerStats {
    int num_threads_spawned{0};            // The number of threads spawned
    int num_evaluated{0};                  // The number of evaluated edges.
    std::vector<int> num_jobs_per_thread;  // The number of jobs per thread
};

/// @class ParallelSearch Abstract class.
/// @brief A Abstract class for parallel search algorithm that utilize multi-threads to find the optimal path
class ParallelSearch : public Planner {
   protected:
    /*ABSTRACT LEVEL/STRUCT DEFINITION*/

    /// @brief The search state.
    struct SearchState : public ims::SearchState, ims::SearchStateLowerBoundMixin {
        /// @brief The parent state
        int parent_id = UNSET;
        /// @brief The cost to come
        double g = INF_DOUBLE;
        /// @brief The heuristic value. Even though not all planners use it, we have it here because many do.
        double h = INF_DOUBLE;
        /// @brief The f value
        double f = INF_DOUBLE;
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

    /*DATA STRUCTURE/VARIABLES*/

    /// @brief The parameters.
    ParallelSearchParams params_;

    /// @brief The open list for states/edges.
    /// @note how to make it work with both states and edges?
    // SimpleQueue<SearchState, SearchStateCompare> open_;

    /// @brief A vector holding all the state/edge that is WIP
    // std::vector<SearchState*> wip_;

    /// @brief Keeping track of states by their id.
    std::vector<SearchState*> states_;

    /// @brief Pointer to the heuristic function
    BaseHeuristic* heuristic_ = nullptr;

    /// @brief The action space.
    std::shared_ptr<ActionSpace> action_space_ptr_;

    /// @brief The stats.
    ParallelSearchPlannerStats stats_;

    /// Multi-threading
    /// @brief lock
    mutable LockType lock_;

    /// @brief lock vector
    mutable std::vector<LockType> lock_vec_;

    /// @brief Multi-threading futures
    std::vector<std::future<void>> work_futures_;

    /// @brief Use a bool vector to keep track of the work status
    std::vector<bool> work_status_;

    /// @brief condition variable
    std::condition_variable cv_;

    /// @brief condition variable vectors
    std::vector<std::condition_variable> cv_vec_;

    /// Control variables
    /// @brief atomic variable to keep track of terminate_ flag
    std::atomic<bool> terminate_{false};
    /// @brief atomic variable to keep track of plan_found_ flag
    std::atomic<bool> plan_found_{false};
    /// @brief atomic bool to allow the main thread to wait for the worker threads
    std::atomic<bool> recheck_flag_{true};

    /*FUNCTIONS*/
    /// @brief Get the state by id
    /// @param state_id The id of the state
    /// @return The state
    /// @note Use this function only if you are sure that the state exists.
    auto getSearchState(int state_id) -> SearchState*;

    /// @brief Get the state by id or create a new one if it does not exist. If a search state does not exist yet and a new one is created, it's ID will be set, and all other member fields will initialize to default values.
    /// @param state_id The id of the state
    /// @return The state
    auto getOrCreateSearchState(int state_id) -> SearchState*;

    /// @brief Compute the heuristic value of from state s to the goal state
    /// @param s The state
    double computeHeuristic(int state_id);

    /// @brief Compute the heuristic value from state s1 to state s2
    /// @param s1 The state
    /// @param s2 The state
    double computeHeuristic(int s1_id, int s2_id);

    /// @brief Reconstruct the path by backtracking
    /// @param path The reference of vector of states to be populated
    void reconstructPath(std::vector<StateType>& path) override;

    /// @brief Reconstruct the path by backtracking
    /// @param path The reference of vector of states to be populated
    /// @param costs The reference of vector of costs to be populated (edge costs)
    void reconstructPath(std::vector<StateType>& path, std::vector<double>& costs) override;

    bool isGoalState(int state_id) override;

    /// @brief check if a thread is terminated using std::future
    template <typename T>
    inline bool isFutureReady(T& future) { return future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready; };

    /// @brief function for the worker thread to notify main thread
    inline void notifyMainThread() {
        recheck_flag_ = false;
        cv_.notify_one();
    }

   public:
    /// @brief Constructor
    /// @param params The parameters
    explicit ParallelSearch(const ParallelSearchParams& params);

    /// @brief Destructor
    ~ParallelSearch() override;

    /// @brief Initialize the planner
    /// @param action_space_ptr The action space
    /// @param starts Vector of start states
    /// @param goals Vector of goal states
    virtual void initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
                                   const std::vector<StateType>& starts,
                                   const std::vector<StateType>& goals) = 0;

    /// @brief Initialize the planner
    /// @param action_space_ptr The action space
    /// @param start The start state
    /// @param goal The goal state
    virtual void initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
                                   const StateType& start, const StateType& goal) = 0;

    /// @brief plan a path
    /// @param path The path
    /// @return if the plan was successful or not
    virtual bool plan(std::vector<StateType>& path) = 0;

    virtual void resetPlanningData() override;

    inline ParallelSearchPlannerStats reportStats() { return stats_; }
    inline ParallelSearchPlannerStats getStats() { return stats_; }

};  // class Parallel Search

}  // namespace ims
