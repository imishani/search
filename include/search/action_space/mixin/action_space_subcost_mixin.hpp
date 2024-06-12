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
 * \file   action_space_subcost_mixin.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   Oct 05 2023
 */

#pragma once

// standard includes
#include <functional>
#include <algorithm>
#include <utility>

// project includes
#include <search/action_space/action_space.hpp>
#include <search/heuristics/base_heuristic.hpp>
#include <search/planners/planner.hpp>

namespace ims {

/// @brief A trait class (mixin) for allowing an ActionSpace to return a subcost in addition to a cost for each successor transition. This "subcost" is an additional cost of an edge transition. As an example, in ECBS, this is the number of conflicts that would be created with other agents when traversing this edge.
class ActionSpaceSubcostMixin {
public:
    /// @brief Constructor
    explicit ActionSpaceSubcostMixin() = default;

    /// @brief Destructor
    ~ActionSpaceSubcostMixin() = default;

    /// @brief Overrides the getSuccessors method of ActionSpace to return a subcost in addition to a cost for each successor transition.
    /// @details This method is used to get the successors of a given state.
    /// @param state The state for which we want to get the successors.
    /// @param successors The vector of successors to be filled.
    /// @param costs The vector of costs to be filled.
    /// @param subcosts The vector of subcosts to be filled.
    virtual bool getSuccessors(int curr_state_ind,
                                   std::vector<int>& successors,
                                   std::vector<double>& costs, 
                                   std::vector<double> &subcosts) = 0;
    /// @brief Compute the subcost associated with a particular timed transition. Notice that states have time in their last element.
    virtual void getTransitionSubcost(const StateType& state_val_from, const StateType& state_val_to, double & subcost) = 0;
};

}  // namespace ims

