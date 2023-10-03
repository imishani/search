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
 * \file   constrained_search.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   July 10 2023
 */

#ifndef SEARCH_CONSTRAINEDSEARCH_HPP
#define SEARCH_CONSTRAINEDSEARCH_HPP

// standard includes
#include <functional>
#include <algorithm>
#include <utility>

// project includes
#include "action_space.hpp"
#include "search/heuristics/base_heuristic.hpp"
#include "search/planners/planner.hpp"
#include "search/common/constraints.hpp"
#include "search/common/conflicts.hpp"
#include "action_space_constrainable_mixin.hpp"

namespace ims {

/// @brief Base class for ActionSpaces with constraints.
/// @details This is an actions space extended to be "Constrainable" using a mixin.
class ConstrainedActionSpace : public ActionSpace, public ActionSpaceConstrainableMixin{
public:
    /// @brief Constructor
    explicit ConstrainedActionSpace(): ActionSpace(), ActionSpaceConstrainableMixin() {
        std::cout << "ConstrainedActionSpace: Constructor" << std::endl;
        constraints_collective_ptr_ = std::make_shared<ConstraintsCollective>();
    }

    /// @brief Destructor
    ~ConstrainedActionSpace() = default;

    /// @brief If a given configuration is valid with respect to the constraints. 
    virtual bool isSatisfyingConstraints(const StateType& state, const StateType& next_state) = 0;

};

}  // namespace ims

#endif  // SEARCH_CONSTRAINEDSEARCH_HPP
