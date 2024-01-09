/*
 * Copyright (C) 2024, Itamar Mishani
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
 * \file   action_space_msg.hpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   Jan 01 2024
 */

#pragma once

// standard includes
#include <functional>
#include <algorithm>
#include <utility>
#include <random>

// project includes
#include <search/action_space/action_space.hpp>
#include <search/heuristics/base_heuristic.hpp>
#include <search/planners/planner.hpp>

namespace ims {

class ActionSpaceMGS: virtual public ActionSpace {
public:

    std::shared_ptr<ActionType> action_type_;

    /// @brief Constructor
    /// @param action_type The type of the action space
    explicit ActionSpaceMGS(const std::shared_ptr<ActionType>& action_type) :
                ActionSpace() {
        action_type_ = action_type;
    }

    /// @brief Generate random state within ellipsoidal region
    /// @param s1 The first state
    /// @param s2 The second state
    /// @param dist The distance between two states (based on heuristic)
    /// @param ransom_state The random state
    /// @return True if the random state is generated successfully
    virtual bool generateRandomState(const StateType& s1,
                                     const StateType& s2,
                                     double dist,
                                     StateType& random_state) {
        // maximum attempts to generate a random state
        int max_attempts = 1000;
        int attempts = 0;
        while (attempts < max_attempts){
            // get the random state
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> dis(0, 1);
            ////////// Ellipsoid ////////// TODO: Fix
            double major_axis = dist;
            double minor_axis = dist * dis(gen);
            double ratio = minor_axis / major_axis;
            StateType diff(s1.size());
            for (size_t i {0}; i < s1.size(); ++i){
                diff[i] = s2[i] - s1[i];
            }
            random_state.resize(s1.size());
            for (size_t i {0}; i < s1.size(); ++i){
//                double minor_axis = dist * dis(gen);
//                double ratio = minor_axis / major_axis;
                random_state[i] = s1[i] + ratio * diff[i];
                random_state[i] = std::round(random_state[i] / action_type_->state_discretization_[i]) * action_type_->state_discretization_[i];
            }
            // check if the state is valid
            if (isStateValid(random_state)){
                return true;
            } else {
                attempts++;
            }
        }
        return false;
    }

};

}  // namespace ims

