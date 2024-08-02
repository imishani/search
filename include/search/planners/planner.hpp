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


#ifndef SEARCH_PLANNER_HPP
#define SEARCH_PLANNER_HPP

// standard includes
#include <memory>
#include <chrono>
// project includes
#include "search/action_space/action_space.hpp"
#include <search/common/intrusive_heap.h>
#include <search/common/types.hpp>
#include <search/common/utils.hpp>

namespace ims{
    /// @brief A super class for representing a state in a search algorithm. These states are abstracted away from the robot world, and do not include any information about the robot's configuration. The only interaction between the search states and the robot states happens via the action space, and specifically the action space's successor function. This function may take in a search state index, and return a vector of successor state indices, along with the cost of the action that takes the robot from the search state to the successor state. The action space and the planner are responsible for maintaining the mapping between the search states and the robot states. This is nominally done via two vectors of pointers to robot/search states, which are maintained by the planner and the action space. The indices should by synchronized between the two vectors.
    /// Each planner and each action space specialize the search state class and the robot state class to include information relevant to the domain and planning algorithm.
    /// NOTE(yoraish): in any algorithm implementation, the SearchState will be a private member of the planner class.
struct SearchState : public ::smpl::HeapElement {
        int state_id = UNSET; // The id of the search state. If applicable, the search state would be mapped to a robot state with the same id.

        /// @brief Pure virtual method to print state values
        virtual void print() = 0;
    };

    /// @class Planner Parameters abstract class
    struct PlannerParams{
        /// @brief Constructor
        PlannerParams() : time_limit_(60.0) {}

        /// @brief Destructor
        virtual ~PlannerParams() = default;

        /// @brief Add new parameter to the PlannerParams as an attribute
        /// @param param The parameter
        /// make it take any type of parameter
//        template<typename T>
//        void addParam(const T& param) {
//        }

        double time_limit_; // seconds
        bool verbose = false;
    };

    ///@brief Pure virtual base class planner interface
    /// It should include methods for initializing the problem, evaluating the cost of a state,
    /// generating successors, and checking if a state is a goal state.
    /// This interface should be implemented by all search problem instances.
    class Planner {
    public:

        ///@brief Constructor
        ///@param params The planner parameters based on PlannerParams struct
        explicit Planner(const PlannerParams& params): params_(params) {
        };

        ///@brief Destructor
        virtual ~Planner() = default;

        /// @brief Initialize the planner
        /// @param action_space_ptr The action space
        /// @param starts Vector of start states
        /// @param goals Vector of goal states
        virtual void initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
                                       const std::vector<StateType>& starts,
                                       const std::vector<StateType>& goals){
          if (starts.size() != 1 || goals.size() != 1){
              throw std::invalid_argument("Planner::initializePlanner: Currently, Planner only supports one start and one goal state");
          }
          else {
              initializePlanner(action_space_ptr, starts[0], goals[0]);
          }
        }

        /// @brief Initialize the planner
        /// @param action_space_ptr The action space
        /// @param start The start state
        /// @param goal The goal state
        virtual void initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
                                       const StateType& start, const StateType& goal){
            // Throw an error if this base class is called.
            // I did not want to make this a pure virtual function because I wanted to have the option to implement it
            // in the derived class with different action-space types.
            throw std::invalid_argument("Planner::initializePlanner: This function should be implemented in the derived class");
        };

        /// @brief start the timer
        void startTimer() { t_start_ = std::chrono::steady_clock::now(); }

        void getTimeFromStart(double &elapsed_time) {
            auto t_end = std::chrono::steady_clock::now();
            double scaler = 1e9;
            elapsed_time = (double)std::chrono::duration_cast<std::chrono::nanoseconds>(t_end - t_start_).count();
            elapsed_time /= scaler;
        }
        
        /// @brief utility function to collect runtime
        void stampTimer() { t_stamp_ = std::chrono::steady_clock::now(); }
        
        void getTimeFromStamp(double &elapsed_time) {
            auto t_end = std::chrono::steady_clock::now();
            double scaler = 1e9;
            elapsed_time = (double)std::chrono::duration_cast<std::chrono::nanoseconds>(t_end - t_stamp_).count();
            elapsed_time /= scaler;
        }

        double getTimeFromStamp() {
            auto t_end = std::chrono::steady_clock::now();
            double scaler = 1e9;
            double elapsed_time = (double)std::chrono::duration_cast<std::chrono::nanoseconds>(t_end - t_stamp_).count();
            return elapsed_time /= scaler;
        }

        bool isTimeOut() {
            double elapsed_time;
            getTimeFromStart(elapsed_time);
            return elapsed_time > params_.time_limit_;
        }

        PlannerStats reportStats () {
            return stats_;
        }
        PlannerStats getStats () {
            return stats_;
        }

        /// @brief plan
        /// @param path The path
        /// @return if the plan was successful or not
        virtual bool plan(std::vector<StateType>& path) = 0;
        
        /// @brief Reset all the member variables in the planner.
        virtual void resetPlanningData() = 0;

    protected:

        /// @brief Reconstruct the path
        virtual void reconstructPath(std::vector<StateType>& path) = 0;

        /// @brief Reconstruct the path and also get the transition costs.
        /// @param path The path to be populated
        /// @param costs The costs to be populated. Cost at index i is the cost of the transition from state i to state i+1. Thus, the cost at the goal state is zero as there is no transition from the goal state.
        virtual void reconstructPath(std::vector<StateType>& path, std::vector<double>& transition_costs) = 0;

        /// @brief Check if the current state is the goal state
        /// @return if the current state is the goal state or not
        virtual bool isGoalState(int s_id) = 0;

        int goal_{};
        std::vector<int> goals_;
        PlannerParams params_;
        std::chrono::time_point<std::chrono::steady_clock> t_start_;
        std::chrono::time_point<std::chrono::steady_clock> t_stamp_;
        PlannerStats stats_;
        std::shared_ptr<ActionSpace> action_space_ptr_;

    };
}

#endif //SEARCH_PLANNER_HPP
