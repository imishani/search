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
 * \file   subcost_constrained_action_space.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   Oct 05 2023
 */
#pragma once

// standard includes
#include <functional>
#include <algorithm>
#include <utility>

// Project includes.
#include "experience_accelerated_constrained_action_space.hpp"
#include "subcost_action_space.hpp"

namespace ims {

/// @brief Base class for ActionSpaces with constraints.
/// @details This is an actions space extended to be "Constrainable" using a mixin.
class SubcostExperienceAcceleratedConstrainedActionSpace : public ExperienceAcceleratedConstrainedActionSpace{//, public SubcostActionSpace{ 
public:
    /// @brief Constructor
    explicit SubcostExperienceAcceleratedConstrainedActionSpace(): ExperienceAcceleratedConstrainedActionSpace(){};//, SubcostActionSpace() {};

    /// @brief Destructor
    ~SubcostExperienceAcceleratedConstrainedActionSpace() = default;

};

}  // namespace ims

