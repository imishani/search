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
 * \file   epase.hpp
 * \author Hanlan Yang (yanghanlan666@gmail.com)
 * \date   July 19 2024
 */

#pragma once

// standard includes
#include <functional>
// Standard includes
#include <algorithm>
#include <future>
#include <utility>

// project includes
#include <search/planners/parallel_search/parallel_search.hpp>

namespace ims {

/// @class Edge-based Parallel A* with Slow Evaluation (ePASE) Search class.
/// @brief This is an algorithm that utilizes multi-threading to speed up the search process. The key idea is to evaluate independent edges
class Epase : public ParallelSearch {
protected:
    /*FUNCTIONS*/

    /// @brief independency check
    bool independenceCheck(int edge_id, const boost::any& popped_vec) override;

    /// @brief helper function to check if there's any edge is being work in progress
    inline bool noWorkInProgress() const {
        return std::all_of(work_in_progress_->begin(),
                           work_in_progress_->end(),
                           [](std::shared_ptr<ims::SearchState> Edge) { return Edge == nullptr; });
    }

    /// @brief loop function for worker threads
    void workerLoop(int thread_id) override;

    /// @brief helper function to expand a proxy edge
    virtual void expandProxy(std::shared_ptr<SearchEdge> Edge, int thread_id);

    /// @brief main function to expand an edge
    void expand(std::shared_ptr<SearchEdge> Edge, int thread_id);

public:
    /// @brief Constructor
    /// @param params The parameters
    explicit Epase(const ParallelSearchParams& params);

    /// @brief Destructor
    ~Epase();

    /// @brief Initialize the planner
    /// @param action_space_ptr The action space
    /// @param starts Vector of start states
    /// @param goals Vector of goal states
    virtual void initializePlanner(const std::shared_ptr<EdgeActionSpace>& action_space_ptr,
                                   const std::vector<StateType>& starts,
                                   const std::vector<StateType>& goals);

    /// @brief Initialize the planner
    /// @param action_space_ptr The action space
    /// @param start The start state
    /// @param goal The goal state
    virtual void initializePlanner(const std::shared_ptr<EdgeActionSpace>& action_space_ptr,
                                   const StateType& start, const StateType& goal);

    /// @brief plan a path
    /// @param path The path
    /// @return if the plan was successful or not
    bool plan(std::vector<StateType>& path);

};  // class edge-based Parallel A* with Slow Evaluation (ePASE) Search

}  // namespace ims