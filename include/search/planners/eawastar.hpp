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
 * \file   eawastar.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   September 5 2023
*/

#ifndef SEARCH_EAWASTAR_HPP
#define SEARCH_EAWASTAR_HPP

// standard includes
#include <functional>
// Standard includes
#include <utility>
#include <algorithm>
// project includes
#include <search/planners/best_first_search.hpp>
#include <search/planners/wastar.hpp>
#include <search/common/experiences.hpp>
#include <search/action_space/experience_accelerated_constrained_action_space.hpp>

namespace ims{

    /// @class AStarParams class.
    /// @brief The parameters for the AStar algorithm
    struct EAwAStarUniformCostParams : public wAStarParams{

        /// @brief Constructor
        /// @param heuristic The heuristic function. Passing the default heuristic function will result in a uniform cost search
        explicit EAwAStarUniformCostParams(BaseHeuristic* heuristic,
                              double epsilon) : wAStarParams(heuristic, epsilon) {
            this->epsilon = epsilon;
        }

        /// @brief Destructor
        ~EAwAStarUniformCostParams() override = default;

        double epsilon;

    };

    /// @class EAwAStarUniformCost class. Weighted A* algorithm
    /// @brief A weighted A* algorithm implementation. This algorithm is a modification of the A* algorithm that
    /// uses inflation of the heuristic function to find a solution with a cost that is within a factor of epsilon
    /// of the optimal solution (epsilon-suboptimality).
    class EAwAStarUniformCost : public wAStar{

    private:


    public:
        /// @brief Constructor
        /// @param params The parameters
        explicit EAwAStarUniformCost(const EAwAStarUniformCostParams &params);

        /// @brief Destructor
        ~EAwAStarUniformCost() override;

        /// @brief Initialize the planner
        /// @param action_space_ptr The action space
        /// @param starts Vector of start states
        /// @param goals Vector of goal states
        void initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
                               const std::vector<StateType>& starts,
                               const std::vector<StateType>& goals) override;

        /// @brief Initialize the planner
        /// @param action_space_ptr The action space
        /// @param start The start state
        /// @param goal The goal state
        void initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
                               const StateType& start, const StateType& goal) override;

        /// @brief Add a connected subpath to the open list.
        /// @param states The states of the subpath
        /// @note This function assumes that each state in the subpath can have its parent set to the previous state.
        void addValidSubpathToOpenList(const std::vector<std::vector<int>> & seq_paths_state_ids,
                                                              const std::vector<std::vector<double>> & seq_paths_transition_costs);

    protected:

        /// @brief A pointer to the action space. We need this action space to be (at least) experience-accelerateable in order to have access to the action space method querying for valid experience subpaths.
        std::shared_ptr<ExperienceAcceleratedConstrainedActionSpace> action_space_ptr_;

        void expand(int state_id) override;

        /// @brief  Asks the action space for valid experience subpaths from the given state and adds them to the open list. Validity of subpaths is determined by the action space. It could be validity with respect to constraints, or validity with respect to other factors in the environment. In EAwA* and EACBS, we only keep collision-free experiences and therefore only check for validity with respect to constraints. 
        /// @param state_id 
        void experienceAccelerateOpenList(int state_id);

        EAwAStarUniformCostParams params_;

    };

}


#endif //SEARCH_EAWASTAR_HPP
