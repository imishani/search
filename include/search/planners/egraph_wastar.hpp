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
 * \file   egraph_wastar.hpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   8/7/23
*/

#ifndef SEARCH_SEARCH_INCLUDE_SEARCH_PLANNERS_EGRAPH_WASTAR_HPP_
#define SEARCH_SEARCH_INCLUDE_SEARCH_PLANNERS_EGRAPH_WASTAR_HPP_

// standard includes
#include <functional>
#include <utility>
#include <algorithm>
// project includes
#include "search/planners/wastar.hpp"
#include "search/action_space/egraph_action_space.hpp"

namespace ims{

/// @class AStarParams class.
/// @brief The parameters for the AStar algorithm
struct ExperienceWAStarParams : public wAStarParams{

    /// @brief Constructor
    /// @param heuristic The heuristic function. Passing the default heuristic function will result in a uniform cost search
    ExperienceWAStarParams(EGraphHeuristicBase* heuristic,
                                    double epsilon,
                                    double egraph_epsilon,
                                    const std::string& experiences_dir) : wAStarParams(heuristic, epsilon) {
        this->egraph_epsilon = egraph_epsilon;
        this->egraph_heuristic = heuristic;
        this->experiences_dir = experiences_dir;
    }

    /// @brief Destructor
    ~ExperienceWAStarParams() override = default;

    double egraph_epsilon; // epsilon for the egraph
    EGraphHeuristicBase* egraph_heuristic;
    std::string experiences_dir;
};

/// @class wAStar class. Weighted A* algorithm
/// @brief A weighted A* algorithm implementation. This algorithm is a modification of the A* algorithm that
/// uses inflation of the heuristic function to find a solution with a cost that is within a factor of epsilon
/// of the optimal solution (epsilon-suboptimality).
class ExperienceWAstar : public wAStar {

public:
    /// @brief Constructor
    /// @param params The parameters
    explicit ExperienceWAstar(const ExperienceWAStarParams &params);

    /// @brief Initialize the planner
    /// @param action_space_ptr The action space
    /// @param starts Vector of start states
    /// @param goals Vector of goal states
    void initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
                           const std::vector<StateType>& starts,
                           const std::vector<StateType>& goals) override;
//
    /// @brief Initialize the planner
    /// @param action_space_ptr The action space
    /// @param start The start state
    /// @param goal The goal state
    void initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
                           const StateType& start, const StateType& goal) override;

    /// @brief plan a path
    /// @param path The path
    /// @return if the plan was successful or not
    bool plan(std::vector<StateType> &path) override;


protected:

    void extractPath(const std::vector<int> &search_path, PathType &path);

    void reconstructPath(std::vector<StateType> &path) override;

    void expand(int state_id) override;

    void syncStatesCreated();

    ExperienceWAStarParams params_;

    std::shared_ptr<EGraphActionSpace> egraph_action_space_ptr_ = nullptr;

};

}

#endif //SEARCH_SEARCH_INCLUDE_SEARCH_PLANNERS_EGRAPH_WASTAR_HPP_
