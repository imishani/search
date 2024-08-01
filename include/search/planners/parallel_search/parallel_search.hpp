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
#include <boost/any.hpp>
#include <future>
#include <memory>
#include <utility>

// project includes
#include <search/action_space/edge_action_space.hpp>
#include <search/common/queue_general.hpp>
#include <search/heuristics/base_heuristic.hpp>
#include <search/planners/planner.hpp>

namespace ims {

/// @class ParallelSearch Parameters
struct ParallelSearchParams : public PlannerParams {
    /// @brief Constructor
    ParallelSearchParams(std::shared_ptr<BaseHeuristic> heuristic) : PlannerParams(), heuristic_(heuristic), i_heuristic_(heuristic) {}
    ParallelSearchParams(std::shared_ptr<BaseHeuristic> heuristic, std::shared_ptr<BaseHeuristic> i_heuristic) : PlannerParams(), heuristic_(heuristic), i_heuristic_(i_heuristic) {}
    ParallelSearchParams(std::shared_ptr<BaseHeuristic> heuristic, int num_threads, double epsilon) : PlannerParams(), heuristic_(heuristic), i_heuristic_(heuristic), num_threads_(num_threads), epsilon_(epsilon) {}
    ParallelSearchParams(std::shared_ptr<BaseHeuristic> heuristic, std::shared_ptr<BaseHeuristic> i_heuristic, int num_threads, double epsilon) : PlannerParams(), heuristic_(heuristic), i_heuristic_(i_heuristic), num_threads_(num_threads), epsilon_(epsilon) {}

    /// @brief Destructor
    ~ParallelSearchParams() override = default;

    std::shared_ptr<BaseHeuristic> heuristic_ = nullptr;
    std::shared_ptr<BaseHeuristic> i_heuristic_ = nullptr;  // Heuristic for independency check
    int num_threads_ = 1;
    double epsilon_ = 1.0;
};

struct ParallelSearchPlannerStats : public PlannerStats {
    int num_threads_spawned{0};            // The number of threads spawned
    int num_evaluated{0};                  // The number of evaluated edges.
    std::vector<int> num_jobs_per_thread;  // The number of jobs per thread
    double lock_time{0};                   // The time spent on locking
    double evaluation_time{0};             // The time spent on getSuccessor/edge evaluation
};

/// @class ParallelSearch Base class.
/// @brief A base class for parallel search algorithm that utilize multi-threads to find the optimal path
/// @note This class cannot run by itself as it is lacking implementation in essential functions (plan(),etc.).
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

        virtual void print(std::string str) {
            std::cout << str
                      << "State: " << state_id
                      << " | Parent: " << parent_id
                      << " | g: " << g
                      << " | f: " << f
                      << " | h: " << h
                      << std::endl;
        }

        double getLowerBound() const override {
            return f;
        }
    };

    struct SearchEdge : public ParallelSearch::SearchState {
        int edge_id = UNSET;
        int child_id = UNSET;
        double cost = INF_DOUBLE;
        bool is_proxy = true;
        double edge_priority = INF_DOUBLE;

        bool isState() const {
            return !is_proxy;
        }

        void print(std::string str) override {
            std::cout << str
                      << "Edge: " << edge_id
                      << " | Parent: " << parent_id
                      << " | g: " << g
                      << " | f: " << f
                      << " | h: " << h
                      << std::endl;
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

    /// @brief The search edge compare struct.
    struct SearchEdgeCompare {
        bool operator()(const SearchEdge& e1, const SearchEdge& e2) const {
            if (e1.edge_priority != e2.edge_priority)
                return e1.edge_priority < e2.edge_priority;
            if (e1.f != e2.f)
                return e1.f < e2.f;
            if (e1.g != e2.g)
                return e1.g > e2.g;
            return e1.state_id < e2.state_id;
        }
    };

    /*DATA STRUCTURE/VARIABLES*/

    /// @brief The parameters.
    ParallelSearchParams params_;

    /// @brief Keeping track of states by their id.
    std::vector<std::shared_ptr<SearchState>> states_;

    /// @brief Keeping track of edges by their id.
    std::vector<std::shared_ptr<SearchEdge>> edges_;

    /// @brief Pointer to the heuristic function
    std::shared_ptr<BaseHeuristic> heuristic_ = nullptr;

    /// @brief Pointer to the independence heuristic function
    std::shared_ptr<BaseHeuristic> i_heuristic_ = nullptr;

    /// @brief The action space.
    std::shared_ptr<EdgeActionSpace> action_space_ptr_;

    /// @brief The (abstract) open list for search state.
    std::unique_ptr<AbstractQueue<SearchState>> state_open_;

    /// @brief The (abstract) open list for search state.
    std::unique_ptr<AbstractQueue<SearchEdge>> edge_open_;

    /// @brief A vector holding all the state/edge that is WIP
    std::unique_ptr<std::vector<std::shared_ptr<ims::SearchState>>> work_in_progress_;

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
    std::vector<int> work_status_;

    /// @brief condition variable
    std::condition_variable cv_;

    /// @brief condition variable vectors
    std::vector<std::condition_variable> cv_vec_;

    /// @brief Vector to hold time stamp for each thread
    std::vector<std::chrono::time_point<std::chrono::steady_clock>> t_stamps_;

    /// Control variables
    /// @brief atomic variable to keep track of terminate_ flag
    std::atomic<bool> terminate_{false};
    /// @brief atomic variable to keep track of plan_found_ flag
    std::atomic<bool> plan_found_{false};
    /// @brief atomic bool to allow the main thread to wait for the worker threads
    std::atomic<bool> recheck_flag_{true};

    /*FUNCTIONS*/

    /**Not Implemented**/

    /// @brief independency check to prevent re-expansion/re-evaluation
    virtual bool independenceCheck(int id, const boost::any& popped_vec) = 0;

    /// @brief worker loop for multi-threading
    virtual void workerLoop(int thread_id) = 0;

    /**Implemented**/

    /// @brief Start the t_stamp_ timer for thread_id
    /// @param thread_id The id of the thread
    inline void stampTimer(int thread_id) { t_stamps_[thread_id] = std::chrono::steady_clock::now(); }

    /// @brief Get the time from the t_stamp_ timer for thread_id
    /// @param thread_id The id of the thread
    /// @return Time elapsed in second
    double getTimeFromStamp(int thread_id) {
        auto t_end = std::chrono::steady_clock::now();
        double scaler = 1e9;
        double elapsed_time = (double)std::chrono::duration_cast<std::chrono::nanoseconds>(t_end - t_stamps_[thread_id]).count();
        return elapsed_time /= scaler;
    }

    /// @brief Get the state by id
    /// @param state_id The id of the state
    /// @return The state
    /// @note Use this function only if you are sure that the state exists.
    auto getSearchState(int state_id) -> std::shared_ptr<SearchState> {
        assert(state_id < states_.size() && state_id >= 0);
        return states_[state_id];
    }

    /// @brief Get the edge by id
    /// @param edge_id The id of the state
    /// @return The state
    /// @note Use this function only if you are sure that the state exists.
    auto getSearchEdge(int edge_id) -> std::shared_ptr<SearchEdge> {
        assert(edge_id < edges_.size() && edge_id >= 0);
        return edges_[edge_id];
    }

    /// @brief Get the state by id or create a new one if it does not exist. If a search state does not exist yet and a new one is created, it's ID will be set, and all other member fields will initialize to default values.
    /// @param state_id The id of the state
    /// @return The state
    auto getOrCreateSearchState(int state_id) -> std::shared_ptr<SearchState> {
        if (state_id >= states_.size()) {
            states_.resize(state_id + 1, nullptr);
        }
        if (states_[state_id] == nullptr) {
            assert(state_id < states_.size() && state_id >= 0);
            states_[state_id] = std::make_shared<SearchState>();
            states_[state_id]->state_id = state_id;
        }
        return states_[state_id];
    }

    /// @brief Get the proxy edge by id or create a new one if it does not exist. Note that the input will be a pair of proxy ind and state ind.
    /// @param edge_id The id of the edge
    /// @param state_id The id of the state
    /// @return The edge
    auto getOrCreateProxyEdge(int edge_id, int state_id) -> std::shared_ptr<SearchEdge> {
        if (edge_id >= edges_.size()) {
            edges_.resize(edge_id + 1, nullptr);
        }
        if (edges_[edge_id] == nullptr) {
            assert(edge_id < edges_.size() && edge_id >= 0);
            edges_[edge_id] = std::make_shared<SearchEdge>();
            edges_[edge_id]->edge_id = edge_id;
            edges_[edge_id]->state_id = state_id;
            edges_[edge_id]->is_proxy = true;
        }
        else {
            assert(edges_[edge_id]->state_id == state_id);
        }
        return edges_[edge_id];
    }

    /// @brief sync the proxy edge value with its corresponding state
    /// @param edge_id The id of the edge
    virtual void setProxyVals(int edge_id) {
        auto edge = getSearchEdge(edge_id);
        if (!edge->isState()) {
            throw std::runtime_error("The edge is not a proxy edge");
        }
        auto state = getSearchState(edge->state_id);
        edge->parent_id = state->parent_id;
        edge->g = state->g;
        edge->h = state->h;
        edge->f = state->f;
        edge->edge_priority = state->f;
    }

    /// @brief set the state with parent id and cost
    /// @param state_id The id of the state.
    /// @param parent_id The id of the parent state.
    /// @param cost The cost associated with the search state. For example, in A*, this would be the f value.
    virtual void setStateVals(int state_id, int parent_id, double cost) {
        auto state_ = getSearchState(state_id);
        auto parent = getSearchState(parent_id);
        state_->parent_id = parent_id;
        state_->g = parent->g + cost;
        state_->h = computeHeuristic(state_id);
        state_->f = state_->g + params_.epsilon_ * state_->h;
    }

    /// @brief Compute the heuristic value of from state s to the goal state
    /// @param s The state
    double computeHeuristic(int state_id) {
        double dist;
        auto s = action_space_ptr_->getRobotState(state_id);
        if (!heuristic_->getHeuristic(s->state, dist))
            throw std::runtime_error("Heuristic function failed");
        else
            return dist;
    }

    /// @brief Compute the heuristic value from state s1 to state s2
    /// @param s1 The state
    /// @param s2 The state
    double computeHeuristic(int s1_id, int s2_id) {
        double dist;
        auto s1 = action_space_ptr_->getRobotState(s1_id);
        auto s2 = action_space_ptr_->getRobotState(s2_id);
        if (!i_heuristic_->getHeuristic(s1->state, s2->state, dist))
            throw std::runtime_error("Indepence Heuristic function failed");
        else
            return dist;
    }

    /// @brief Reconstruct the path by backtracking
    /// @param path The reference of vector of states to be populated
    void reconstructPath(std::vector<StateType>& path) override {
        path.clear();

        std::shared_ptr<SearchState> state = getSearchState(goal_);
        while (state->parent_id != -1) {
            path.push_back(action_space_ptr_->getRobotState(state->state_id)->state);
            state = getSearchState(state->parent_id);
        }
        path.push_back(action_space_ptr_->getRobotState(state->state_id)->state);
        std::reverse(path.begin(), path.end());
    }

    /// @brief Reconstruct the path by backtracking
    /// @param path The reference of vector of states to be populated
    /// @param costs The reference of vector of costs to be populated (edge costs)
    void reconstructPath(std::vector<StateType>& path, std::vector<double>& costs) override {
        path.clear();
        costs.clear();

        costs.push_back(0);  // The goal state gets a transition cost of 0.
        std::shared_ptr<SearchState> state_ = getSearchState(goal_);
        while (state_->parent_id != -1) {
            path.push_back(action_space_ptr_->getRobotState(state_->state_id)->state);

            // Get the transition cost. This is the difference between the g values of the current state and its parent.
            double transition_cost = state_->g - getSearchState(state_->parent_id)->g;
            costs.push_back(transition_cost);

            state_ = getSearchState(state_->parent_id);
        }
        path.push_back(action_space_ptr_->getRobotState(state_->state_id)->state);

        std::reverse(path.begin(), path.end());
        std::reverse(costs.begin(), costs.end());
    }

    inline bool isGoalState(int state_id) override {
        if (std::any_of(goals_.begin(), goals_.end(), [&state_id](int goal_ind) { return state_id == goal_ind; })) {
            return true;
        }
        // Also ask the action space if this state id is a goal state. Sometimes, states need to be determined as goal in real time.
        // This is EXTREMELY bug prone. Commenting this out for now until we have a proper GoalCondition object.
        // return action_space_ptr_->isGoalState(s_id, goals_);
        return false;
    }

    /// @brief Throw exception if the planner is not initialized correctly
    /// @note I am adding this function since I am not entirely comfortable exposing the
    /// initializePlanner() to user. Adding to prevent double plan() calls.
    void initializeCheck() const {
        if (action_space_ptr_ == nullptr) {
            throw std::runtime_error("Action space is not initialized");
        }
        if (heuristic_ == nullptr || i_heuristic_ == nullptr) {
            throw std::runtime_error("Heuristic is not initialized");
        }
        if (goals_.empty() || goal_ == -1) {
            throw std::runtime_error("Goals are not initialized");
        }
        if (state_open_->empty() && edge_open_->empty()) {
            throw std::runtime_error("Open list is empty, make sure to initialize the planner before run");
        }
    }

    /// @brief check if a thread is terminated using std::future
    template <typename T>
    inline bool isFutureReady(T& future) {
        return future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready;
    };

    /// @brief function for the worker thread to notify main thread
    inline void notifyMainThread() {
        recheck_flag_ = true;
        cv_.notify_one();
    }

    /// @brief helper function to join all the spawned threads
    void joinThread() {
        if (params_.num_threads_ == 1) return;
        // Notify all worker threads to stop.
        for (int thread_id{0}; thread_id < params_.num_threads_ - 1; ++thread_id) {
            std::unique_lock<LockType> locker(lock_vec_[thread_id]);
            work_status_[thread_id] = true;
            locker.unlock();
            cv_vec_[thread_id].notify_one();
        }

        stats_.num_threads_spawned = work_futures_.size() + 1;

        bool all_threads_exit = false;
        while (!all_threads_exit) {
            all_threads_exit = true;
            for (auto& fut : work_futures_) {
                if (!isFutureReady(fut)) {
                    all_threads_exit = false;
                    break;
                }
            }
        }
        work_futures_.clear();
    }

public:
    /**Not Implemented**/

    /// @brief Constructor
    /// @param params The parameters
    explicit ParallelSearch(const ParallelSearchParams& params) : Planner(params), params_(params) {
        // Instantiation
        state_open_ = std::make_unique<SimpleQueue<SearchState, SearchStateCompare>>();
        edge_open_ = std::make_unique<SimpleQueue<SearchEdge, SearchEdgeCompare>>();
        work_in_progress_ = std::make_unique<std::vector<std::shared_ptr<ims::SearchState>>>(params.num_threads_ - 1, nullptr);
        heuristic_ = params.heuristic_;
        i_heuristic_ = params.i_heuristic_;
    }

    /// @brief Destructor
    ~ParallelSearch() override = default;

    /// @brief Initialize the planner
    /// @param action_space_ptr The action space
    /// @param starts Vector of start states
    /// @param goals Vector of goal states
    virtual void initializePlanner(const std::shared_ptr<EdgeActionSpace>& action_space_ptr,
                                   const std::vector<StateType>& starts,
                                   const std::vector<StateType>& goals) = 0;

    /// @brief Initialize the planner
    /// @param action_space_ptr The action space
    /// @param start The start state
    /// @param goal The goal state
    virtual void initializePlanner(const std::shared_ptr<EdgeActionSpace>& action_space_ptr,
                                   const StateType& start, const StateType& goal) = 0;

    /// @brief plan a path
    /// @param path The path
    /// @return if the plan was successful or not
    virtual bool plan(std::vector<StateType>& path) = 0;

    /**Implemented**/

    virtual void resetPlanningData() override {
        // First dealing with raw pointers data structure
        state_open_->clear();
        edge_open_->clear();

        states_.clear();
        states_ = std::vector<std::shared_ptr<SearchState>>();

        edges_.clear();
        edges_ = std::vector<std::shared_ptr<SearchEdge>>();

        goals_.clear();
        goal_ = -1;
        stats_ = ParallelSearchPlannerStats();
        stats_.num_jobs_per_thread.resize(params_.num_threads_, 0);

        work_in_progress_->resize(params_.num_threads_ - 1, nullptr);
        std::vector<std::condition_variable> cv_vec(params_.num_threads_ - 1);
        cv_vec_.swap(cv_vec);
        std::vector<LockType> lock_vec(params_.num_threads_ - 1);
        lock_vec_.swap(lock_vec);
        work_futures_.clear();
        work_status_.resize(params_.num_threads_ - 1, 0);
        t_stamps_.resize(params_.num_threads_ - 1, std::chrono::steady_clock::now());
        terminate_ = false;
        plan_found_ = false;
        recheck_flag_ = true;
    }

    inline ParallelSearchPlannerStats reportStats() { return stats_; }

    inline ParallelSearchPlannerStats getStats() { return stats_; }

};  // class Parallel Search

}  // namespace ims
