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
 * \file   multi_agent_planner.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   Sept 14 2023
*/


#ifndef SEARCH_MULTI_AGENT_PLANNER_HPP
#define SEARCH_MULTI_AGENT_PLANNER_HPP

// Standard includes.
#include <memory>
#include <chrono>

// Project includes.
#include "search/action_space/action_space.hpp"
#include <search/common/intrusive_heap.h>
#include <search/common/types.hpp>
#include <search/planners/planner.hpp>


namespace ims{

    ///@brief Pure virtual base class planner interface
    /// It should include methods for initializing the problem, evaluating the cost of a state,
    /// generating successors, and checking if a state is a goal state.
    /// This interface should be implemented by all search problem instances.
    class MultiAgentPlanner {
    public:

        ///@brief Constructor
        ///@param params The planner parameters based on PlannerParams struct
        explicit MultiAgentPlanner(const PlannerParams& params): params_(params) {
        };

        ///@brief Destructor
        virtual ~MultiAgentPlanner() = default;

        /// @brief Initialize the planner
        /// @param action_space_ptr The action space
        /// @param starts Vector of start states
        /// @param goals Vector of goal states
        virtual void initializePlanner(std::vector<std::shared_ptr<ActionSpace>>& action_space_ptrs,
                                       const std::vector<StateType>& starts,
                                       const std::vector<StateType>& goals) = 0;

        /// @brief start the timer
        void startTimer() { t_start_ = std::chrono::steady_clock::now(); }

        void getTimeFromStart(double &elapsed_time) {
            auto t_end = std::chrono::steady_clock::now();
            double scaler = 1e9;
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
        virtual bool plan(MultiAgentPaths& paths) = 0;
        
        /// @brief Reset all the member variables in the planner.
        virtual void resetPlanningData() = 0;

    protected:

        std::vector<StateType> goals_;
        std::vector<StateType> starts_;
        PlannerParams params_;
        std::chrono::time_point<std::chrono::steady_clock> t_start_;
        PlannerStats stats_;
        std::vector<std::shared_ptr<ActionSpace>> action_space_ptrs_;

    };
}

#endif //SEARCH_MULTI_AGENT_PLANNER_HPP
