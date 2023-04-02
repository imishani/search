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
#include "common/state.hpp"
#include "common/actionSpace.hpp"

// Always initialize the static member variable
//int ims::state::id_counter = 0;

namespace ims{

    /// @class Planner Parameters abstract class
    struct PlannerParams{
        /// @brief Constructor
        PlannerParams() : m_timeLimit(1000.0) {}

        /// @brief Destructor
        virtual ~PlannerParams() = default;

        double m_timeLimit; // seconds
    };

    ///@brief Pure virtual base class planner interface
    /// It should include methods for initializing the problem, evaluating the cost of a state,
    /// generating successors, and checking if a state is a goal state.
    /// This interface should be implemented by all search problem instances.
    class Planner {
    public:

        ///@brief Constructor
        ///@param params The planner parameters based on PlannerParams struct
        explicit Planner(const PlannerParams& params): m_params(params) {
            m_start = nullptr;
            m_goal = nullptr;
        };

        ///@brief Destructor
        virtual ~Planner() = default;

        /// @brief Initialize the planner
        /// @param actionSpacePtr The action space
        /// @param start The start state
        /// @param goal The goal state
        virtual void initializePlanner(const std::shared_ptr<actionSpace>& actionSpacePtr,
                                       const stateType& start, const stateType& goal) = 0;

        /// Setters
        /// @brief Set the start state
        /// @param start The start state
        void setStartState(state& start) {
            m_start = &start;
            m_start->setParent(-1);
            m_actionSpacePtr->m_states.push_back(m_start);
            m_actionSpacePtr->m_state_to_id[m_start] = m_start->getStateId();
        }

        /// @brief Set the goal state
        /// @param goal The goal state
        void setGoalState(state& goal) {
            m_goal = &goal;
            m_goal->setParent(-2);
            m_actionSpacePtr->m_states.push_back(m_goal);
            m_actionSpacePtr->m_state_to_id[m_goal] = m_goal->getStateId();
        }

        /// @brief start the timer
        void startTimer() { t_start_ = std::chrono::steady_clock::now(); }

        void getTimeFromStart(double &elapsed_time) {
            auto t_end = std::chrono::steady_clock::now();
            elapsed_time = 1e-9*std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start_).count();
        }

        bool isTimeOut() {
            double elapsed_time;
            getTimeFromStart(elapsed_time);
            return elapsed_time > m_params.m_timeLimit;
        }

        /// @brief plan
        /// @param path The path
        /// @return if the plan was successful or not
        virtual bool plan(std::vector<state*>& path) = 0;

        using openList =  smpl::intrusive_heap<state, stateCompare>;
        openList m_open;


    protected:

        /// @brief Expand the current state
        virtual void expand(state* state_) = 0;

        /// @brief Reconstruct the path
        virtual void reconstructPath(std::vector<state*>& path) = 0;

        /// @brief Check if the current state is the goal state
        /// @return if the current state is the goal state or not
        virtual bool isGoalState(const state& s) = 0;

        state* m_start;
        state* m_goal;
        PlannerParams m_params;
        std::chrono::time_point<std::chrono::steady_clock> t_start_;
        plannerStats m_stats;
        std::shared_ptr<actionSpace> m_actionSpacePtr;

    };



}


#endif //SEARCH_PLANNER_HPP
