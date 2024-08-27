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
 * \file   qpase.hpp
 * \author Hanlan Yang (yanghanlan666@gmail.com)
 * \date   August 17 2024
 */

#pragma once

// standard includes
#include <functional>
// Standard includes
#include <algorithm>
#include <future>
#include <utility>

// project includes
#include <search/planners/parallel_search/epase.hpp>
#include <search/planners/parallel_search/parallel_search.hpp>

namespace ims {

/// @class Edge-based Parallel A* with Slow Evaluation with different edge priorities (qPASE) Search class.
/// @brief This is an algorithm that utilizes multi-threading to speed up the search process. The key idea is to evaluate independent edges
class Qpase : public Epase {
protected:
    /*FUNCTIONS*/

    /// @brief helper function to expand a proxy edge
    void expandProxy(std::shared_ptr<SearchEdge> Edge, int thread_id) override;

public:
    /// @brief Constructor
    /// @param params The parameters
    explicit Qpase(const ParallelSearchParams& params);

    /// @brief Destructor
    ~Qpase();

    /// @brief Initialize the planner
    /// @param action_space_ptr The action space
    /// @param starts Vector of start states
    /// @param goals Vector of goal states
    void initializePlanner(const std::shared_ptr<EdgeActionSpace>& action_space_ptr,
                           const std::vector<StateType>& starts,
                           const std::vector<StateType>& goals) override;

    /// @brief Initialize the planner
    /// @param action_space_ptr The action space
    /// @param start The start state
    /// @param goal The goal state
    void initializePlanner(const std::shared_ptr<EdgeActionSpace>& action_space_ptr,
                           const StateType& start, const StateType& goal) override;

};  // class edge-based Parallel A* with Slow Evaluation (ePASE) Search

}  // namespace ims