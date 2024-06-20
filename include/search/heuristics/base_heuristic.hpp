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
 * \file   astar.hpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   4/13/23
*/

#ifndef SEARCH_BASEHEURISTIC_HPP
#define SEARCH_BASEHEURISTIC_HPP

// Project includes.
#include "search/common/types.hpp"
#include "search/common/goal_constraint.hpp"


namespace ims{
    /// @class BaseHeuristic class.
    /// @brief The base heuristic class
    class BaseHeuristic {
    public:
        /// @brief Constructor
        BaseHeuristic() = default;

        /// @brief Destructor
        virtual ~BaseHeuristic()= default;

        /// @brief This function should be implemented by the user and is the function that calculates the heuristic
        /// @param s1 The first state
        /// @param s2 The second state
        /// @param dist The distance between the two states (output, pass by reference)
        /// @return The heuristic value
        virtual bool getHeuristic(const StateType& s1, const StateType& s2, double& dist) = 0;

        /// @brief An option to calculate the heuristic from a single state to the goal state
        /// @param s The state
        /// @param dist The distance between the two states (output, pass by reference)
        /// @return The heuristic value
        virtual bool getHeuristic(const StateType& s, double& dist) = 0;

        /// @brief Set goal constraint
        /// @param goal_constraint The goal constraint
        virtual void setGoalConstraint(GoalConstraint& goal_constraint) {
            goal_constraint_ = goal_constraint;
        }

        /// @brief Set the start state
        /// @param start The start state
        /// @note You have to set the start state if you want to use bfs heuristic
        virtual void setStart(const StateType& start) {
            start_ = start;
        }

        GoalConstraint goal_constraint_;
        StateType start_;

    };


    /// @brief The base heuristic class for single goal heuristics
    class BaseSingleGoalHeuristic : public BaseHeuristic {
    public:

        /// @brief This function should be implemented by the user and is the function that calculates the heuristic
        /// @param s1 The first state
        /// @param s2 The second state
        /// @param dist The distance between the two states (output, pass by reference)
        /// @return The heuristic value
        bool getHeuristic(const StateType& s1, const StateType& s2, double& dist) override = 0;

        /// @brief An option to calculate the heuristic from a single state to the goal state
        /// @param s The state
        /// @param dist The distance between the two states (output, pass by reference)
        /// @return The heuristic value
        bool getHeuristic(const StateType& s, double& dist) override {
            if (goal_.empty()) {
                throw std::runtime_error("Goal state is not set");
            }
            return getHeuristic(s, goal_, dist);
        }

        void setGoalConstraint(GoalConstraint& goal_constraint) override{
            if (goal_constraint.type != SINGLE_SEARCH_STATE_GOAL && goal_constraint.type != SINGLE_SEARCH_STATE_MAPPED_GOAL) {
                throw std::runtime_error("Goal type is not single search state goal");
            }
            if (goal_constraint.type == SINGLE_SEARCH_STATE_GOAL) {
                goal_ = goal_constraint.action_space_ptr->getRobotState(*static_cast<int*>(goal_constraint.check_goal_user))->state;
            } else {
                goal_ = goal_constraint.action_space_ptr->getRobotState(*static_cast<int*>(goal_constraint.check_goal_user))->state_mapped;
            }
            BaseHeuristic::setGoalConstraint(goal_constraint);
        }
        /// @brief Set the goal state
        /// @param goal The goal state
        /// @note You have to set the goal state if you want to use the getHeuristic(const std::shared_ptr<state> s) function
        virtual void setGoal(const StateType& goal) {
            goal_ = goal;
        }

        StateType goal_;

    };

    /// @class ExperienceHeuristicBase
    /// @brief The base class for the experience heuristic
    class EGraphHeuristicBase : public BaseHeuristic {
    public:

        /// @brief get all state ids on the E-graph that has the same heuristic value as the given state
        /// @param s_id The state id
        /// @param state_ids The state ids (output, pass by reference)
        virtual void getEquivalentStates(int s_id, std::vector<int>& state_ids) = 0;

        /// @brief get all state ids of the shortcut states on the E-graph which are available from input state
        /// @param s_id The state id
        /// @param state_ids The state ids (output, pass by reference)
        virtual void getShortcutSuccessors(int s_id, std::vector<int>& state_ids) = 0;

    };

}


#endif //SEARCH_BASEHEURISTIC_HPP
