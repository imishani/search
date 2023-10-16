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
 * \file   sampling_based_planner.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   October 15 2023
*/
#pragma once

// standard includes
#include <memory>
#include <chrono>

// Project includes
#include "search/action_space/action_space.hpp"
#include <search/common/intrusive_heap.h>
#include <search/common/types.hpp>
#include <search/planners/planner.hpp>


namespace ims{

    /// @class SamplingBasedPlanner Parameters abstract class
    struct SamplingBasedPlannerParams : virtual public PlannerParams{
        /// @brief Constructor
        SamplingBasedPlannerParams() : time_limit_(1000.0), PlannerParams(){}

        /// @brief Destructor
        virtual ~SamplingBasedPlannerParams() = default;

        double time_limit_; // seconds
        bool verbose = false;
    };

    ///@brief Pure virtual base class planner interface
    /// It should include methods for initializing the problem, evaluating the cost of a state,
    /// generating successors, and checking if a state is a goal state.
    /// This interface should be implemented by all search problem instances.
    class SamplingBasedPlanner : public Planner {
    public:

        ///@brief Constructor
        ///@param params The planner parameters based on SamplingBasedPlannerParams struct
        explicit SamplingBasedPlanner(const SamplingBasedPlannerParams& params): params_(params), Planner(params){}

        ///@brief Destructor
        virtual ~SamplingBasedPlanner() = default;

        /// @brief Initialize the planner
        /// @param action_space_ptr The action space
        /// @param starts Vector of start states
        /// @param goals Vector of goal states
        virtual void initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
                                       const std::vector<StateType>& starts,
                                       const std::vector<StateType>& goals){
          if (starts.size() != 1 || goals.size() != 1){
              throw std::invalid_argument("SamplingBasedPlanner::initializePlanner: Currently, SamplingBasedPlanner only supports one start and one goal state");
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
                                       const StateType& start, const StateType& goal) = 0;

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
        virtual void reconstructPath(std::vector<StateType>& path, std::vector<double>& costs) = 0;

        /// @brief Check if the current state is the goal state
        /// @return if the current state is the goal state or not
        virtual bool isGoalState(int s_id) = 0;

        /// @brief Sample a new configuration, get its new id.
        virtual bool getNewSample(int& sample_id) = 0;

        SamplingBasedPlannerParams params_;

        /// @brief The search state vector.
        std::vector<SearchState*> states_;
    };
}

