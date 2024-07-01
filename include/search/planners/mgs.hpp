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
 * \file   mgs.hpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   Jan 01 2024
 */


#ifndef SEARCH_MGS_HPP_
#define SEARCH_MGS_HPP_

// standard includes
#include <functional>
// Standard includes
#include <algorithm>
#include <utility>

// project includes
#include <search/action_space/action_space_mgs.hpp>
//#include <search/common/queue_general.hpp>
#include <search/heuristics/base_heuristic.hpp>
#include <search/planners/planner.hpp>

namespace ims {

/// @class MGS Parameters
struct MGSParams : public PlannerParams {
    /// @brief Constructor
    explicit MGSParams(BaseHeuristic* heuristic) : PlannerParams(), heuristic_(heuristic) {}
    explicit MGSParams(BaseHeuristic* heuristic, int g_num) : PlannerParams(),
    heuristic_(heuristic), g_num_(g_num) {}

    /// @brief Destructor
    ~MGSParams() override = default;

    BaseHeuristic* heuristic_ = nullptr;

    /// @brief The number of graphs to be used in the MGS algorithm.
    int g_num_ = 2;
};

struct MGSPlannerStats : public PlannerStats {
    std::vector<StateType> root_states;
};



using ControllerFn = std::vector<ActionSequence>(*)(void* user, const std::shared_ptr<ActionSpace>& action_space_ptr);

enum class ControllerType {
    INVALID,
    GENERATOR,
    CONNECTOR
};

/// @class Controller
/// @brief A general struct for controller. It can be anything, from a simple analytical function to a neural network.
/// @note 1) It is important to define the controller function and the user data before using the controller.
/// @note 2) The controller function should return a vector of action sequences.
/// @example The controller function should be defined as follows:
/// @code inline std::vector<ActionSequence> invalidController(void* user, const std::shared_ptr<ActionSpace>& action_space_ptr) {
/// throw std::runtime_error("Invalid controller function.");
/// }
struct Controller {
    ControllerType type {ControllerType::INVALID}; // The type of the controller
    ControllerFn solver_fn {nullptr}; // The function to solve the state
    std::vector<double>* user_data {nullptr}; // User data for the function
//    TODO: Add constraints.
    std::shared_ptr<ActionSpace> as_ptr {nullptr};
    std::vector<ActionSequence> solve() const  {
        if (solver_fn == nullptr){
            throw std::runtime_error("Solver function is not set.");
        } else if (as_ptr == nullptr) {
            throw std::runtime_error("Action space is not set.");
        }
        return solver_fn(user_data, as_ptr);
    }

    /// @brief Destructor
//    ~Controller() {
//        delete user_data;
//    }
};

/// @class MGS class.
/// @brief A general search algorithm that uses heuristics and g values to find the optimal path
class MGS : public Planner {
private:

    /// @brief The search state.
    struct SearchState {
        int state_id = UNSET; // The id of the search state. If applicable, the search state would be mapped to a robot state with the same id.
        /// @brief Compare based on graph number
        int use_graph_ {-1};
        /// @brief closed list boolean
        bool in_closed {false};
        /// @brief The graph id that closed the state
        int graph_closed {-1};
        /// @brief The heuristic values for a given graph with respect to all other roots.
        std::shared_ptr<std::vector<double>> h_map = nullptr; // TODO: it's a huge hack. need to fix this to be dynamic based on the number of graphs

        void print() const {
            std::cout << "State: " << state_id << std::endl;
        }

        struct HeapData : public ::smpl::HeapElement {
            // graph id.
            int graph_id;
            /// @brief Pointer to the search state.
            SearchState* me;
            /// @brief The parent states
            std::shared_ptr<std::vector<int>> parent_id {nullptr};
            /// @brief edges in the graphs.
            std::shared_ptr<std::vector<std::pair<int, double>>> edges {nullptr};
            /// @brief The cost to come
            double g {INF_DOUBLE};
            /// @brief The heuristic to self root
            double h_self {INF_DOUBLE};
            /// @brief The heuristic values.
            double h {INF_DOUBLE};
            /// @brief The f values.
            double f {INF_DOUBLE};
            /// @brief open list boolean.
            bool is_open = false;
            /// @brief closed list booleans.
            bool is_closed = false;

            /// @brief set the state to open list (make sure it is not in closed list and if it is, update it)
            void setOpen() {
                is_open = true;
                is_closed = false;
            }

            /// @brief set the state to closed list (make sure it is not in open list and if it is, update it)
            /// @param graph_ The graph id that closed the state
            void setClosed() {
                is_closed = true;
                is_open = false;
            }
        };

        HeapData data_[1];
    };


    struct HeapCompare {
        bool operator()(const SearchState::HeapData& s1, const SearchState::HeapData& s2) const {
            if ((s1.f == s2.f) && (s1.g == s2.g))
                return (s1.me->state_id < s2.me->state_id);
            else if (s1.f == s2.f)
                // For tie breaking, we prefer the state with the larger g value as it is closer to the goal (lower h in the case of an informed search).
                return s1.g > s2.g;
            else
                return s1.f < s2.f;
        }
    };

    /// @brief Keeping track of states by their id.
    std::vector<SearchState*> states_;

    /// @brief The parameters.
    MGSParams params_;

    /// @brief The open lists
    using OpenList = ::smpl::IntrusiveHeap<SearchState::HeapData, HeapCompare>;
//    using OpenList = ::smpl::IntrusiveHeap<SearchState, SearchStateCompare>;
    OpenList* opens_ {nullptr};
//    std::unordered_map<int, OpenList> opens_;


    SearchState* start_ {nullptr};

    /// @brief The states with the maximum h_i value.
    std::unordered_map<int, SearchState*> max_h_states_;

    /// @brief The root states.
    std::unordered_map<int, SearchState*> roots_;

    /// @brief Graph for finding a path from start search to goal search (the vertex are the graph indices and the edges are undirected connections between the graphs)
    std::unordered_map<int, std::vector<int>> hl_graph_;

    std::vector<int> closed_graphs_;

    enum GRAPH_ID {
        GRAPH_START = 0,
        GRAPH_GOAL = 1
    };

    int goal_in_graph_ {GRAPH_GOAL};

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
    explicit MGS(const MGSParams& params);

    /// @brief Destructor
    ~MGS() override;

//    /// @brief Initialize the planner
//    /// @param action_space_ptr The action space
//    /// @param starts Vector of start states
//    /// @param goals Vector of goal states
//    virtual void initializePlanner(const std::shared_ptr<ActionSpaceMGS>& action_space_ptr,
//                                   const std::vector<StateType>& starts,
//                                   const std::vector<StateType>& goals);

    /// @brief Initialize the planner
    /// @param action_space_ptr The action space
    /// @param controllers The controllers
    /// @param start The start state
    /// @param goal The goal state
    void initializePlanner(const std::shared_ptr<ActionSpaceMGS>& action_space_ptr,
                           std::shared_ptr<std::vector<Controller>>& controllers,
                           const StateType& start, const StateType& goal);

    inline void initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
                                  const StateType& starts,
                                  const StateType& goals) override { throw std::runtime_error("MGS::initializePlanner() called with an ActionSpace that is not a ActionSpaceMGS."); }


    /// @brief plan a path
    /// @param path The path
    /// @return if the plan was successful or not
    bool plan(std::vector<StateType>& path) override;

    void resetPlanningData() override;

    inline MGSPlannerStats reportStats(){ return stats_;}
    inline MGSPlannerStats getStats(){ return stats_;}

protected:
    /// @brief
    /// @param state_id The id of the state.
    /// @param parent_id The id of the parent state.
    /// @param cost The cost associated with the search state. For example, in A*, this would be the f value.
    /// @param g_num The graph number
    virtual void setStateVals(int state_id, int parent_id, double cost, int g_num);

    /// @brief Compute the heuristic value of from state s to the goal state
    /// @param s The state
    /// @param g_num The graph number
    virtual double computeHeuristic(int state_id, size_t g_num);

    /// @brief Compute the heuristic value from state s1 to state s2
    /// @param s1 The state
    /// @param s2 The state
    virtual double computeHeuristic(int s1_id, int s2_id);

    /// @brief Compute the heuristic value from a state to all other roots
    /// @param s
    virtual void computeHeuristics(int s_id);

    /// @brief Expand the current state
    virtual void expand(int state_id, int g_num);

    /// @brief Try to connect the trees
    /// @param graph_curr The current graph id
    /// @param graph_other The other graph id
    /// @param state_id The state that was expanded in current graph
    /// @param connecting_path The path that connects the two graphs
    /// @return True if the graphs were connected, false otherwise
    virtual bool connect(int graph_curr, int graph_other, int state_id,
                         std::vector<int>& connecting_path,
                         std::vector<double>& costs);

    void reconstructPath(std::vector<StateType>& path) override;
    void reconstructPath(std::vector<StateType>& path, std::vector<double>& costs) override;

    bool isGoalState(int state_id) override;

    /// @brief Check if the goal condition is satisfied
    /// @return True if the goal condition is satisfied
    bool isGoalConditionSatisfied();


    /// @brief generate a random state within the ellipsoid defined based on the heuristic between start and goal
    /// @brief start The start state
    /// @brief goal The goal state
    /// @return
    int generateRandomState(const StateType& start, const StateType& goal);

    /// @brief Connect two graphs based on a state that was expanded in both graphs
    /// @param graph_id1 The first graph
    /// @param graph_id2 The second graph
    /// @param state_id The state that was expanded in both graphs
    /// @param potential_parent_id The potential parent of the state
    int connectGraphs(int graph_id1, int graph_id2, int state_id);


    void saveData();

    BaseHeuristic* heuristic_ = nullptr;

    // The action space.
    std::shared_ptr<ActionSpaceMGS> action_space_ptr_;

    // The controllers.
    std::shared_ptr<std::vector<Controller>> controllers_;

    // The stats.
    MGSPlannerStats stats_;

};  // class MGS

}  // namespace ims


#endif //SEARCH_MGS_HPP_
