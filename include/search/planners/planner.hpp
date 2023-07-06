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
#include <search/common/state.hpp>
#include <search/common/action_space.hpp>

// Always initialize the static member variable
//int ims::State::id_counter = 0;

namespace ims{

    /// @class Planner Parameters abstract class
    struct PlannerParams{
        /// @brief Constructor
        PlannerParams() : time_limit_(1000.0) {}

        /// @brief Destructor
        virtual ~PlannerParams() = default;

        /// @brief Add new parameter to the PlannerParams as an attribute
        /// @param param The parameter
        /// make it take any type of parameter
//        template<typename T>
//        void addParam(const T& param) {
//        }

        double time_limit_; // seconds
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
//            start_ = nullptr;
            goal_ = nullptr;
        };

        ///@brief Destructor
        virtual ~Planner() = default;

        /// @brief Initialize the planner
        /// @param actionSpacePtr The action space
        /// @param starts Vector of start states
        /// @param goals Vector of goal states
        virtual void initializePlanner(const std::shared_ptr<ActionSpace>& actionSpacePtr,
                                       const std::vector<StateType>& starts,
                                       const std::vector<StateType>& goals){
          if (starts.size() != 1 || goals.size() != 1){
              throw std::invalid_argument("Planner::initializePlanner: Currently, Planner only supports one start and one goal state");
          }
          else {
              initializePlanner(actionSpacePtr, starts[0], goals[0]);
          }
        }

        /// @brief Initialize the planner
        /// @param actionSpacePtr The action space
        /// @param start The start state
        /// @param goal The goal state
        virtual void initializePlanner(const std::shared_ptr<ActionSpace>& actionSpacePtr,
                                       const StateType& start, const StateType& goal) = 0;

        /// @brief start the timer
        void startTimer() { t_start_ = std::chrono::steady_clock::now(); }

        void getTimeFromStart(double &elapsed_time) {
            auto t_end = std::chrono::steady_clock::now();
            double scaler = 10e9;
            elapsed_time = (double)std::chrono::duration_cast<std::chrono::nanoseconds>(t_end - t_start_).count();
            elapsed_time /= scaler;
        }

        bool isTimeOut() {
            double elapsed_time;
            getTimeFromStart(elapsed_time);
            return elapsed_time > params_.time_limit_;
        }

        PlannerStats reportStats () {
            return stats_;
        }

        /// @brief plan
        /// @param path The path
        /// @return if the plan was successful or not
        virtual bool plan(std::vector<State*>& path) = 0;


    protected:

        /// @brief Reconstruct the path
        virtual void reconstructPath(std::vector<State*>& path) = 0;

        /// @brief Check if the current state is the goal state
        /// @return if the current state is the goal state or not
        virtual bool isGoalState(const State& s) = 0;

        State* goal_;
        std::vector<int> goals_;
        PlannerParams params_;
        std::chrono::time_point<std::chrono::steady_clock> t_start_;
        PlannerStats stats_;
        std::shared_ptr<ActionSpace> action_space_ptr_;


    };
}


#endif //SEARCH_PLANNER_HPP
