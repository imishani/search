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
 * \file   action_space_experience_acceleratable_mixin.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   Sepetember 5 2023
 */

#ifndef SEARCH_ACTIONSPACEEXPERIENCEACCELERATABLEMIXIN_HPP
#define SEARCH_ACTIONSPACEEXPERIENCEACCELERATABLEMIXIN_HPP

// standard includes
#include <functional>
#include <algorithm>
#include <utility>

// project includes
#include <search/action_space/action_space.hpp>
#include <search/heuristics/base_heuristic.hpp>
#include <search/planners/planner.hpp>
#include <search/common/experiences.hpp>

namespace ims {

/// @brief A trait class (mixin) for allowing an ActionSpace to be experience-accelerated.
/// @details This class is used extend an ActionSpace with the ability to use past experience to accelerate any search algorithm that uses it. This is done by storing path segments in the action space and reusing some of them (valid subpaths) when they are asked-for by the search.
class ActionSpaceExperienceAcceleratableMixin {
public:
    /// @brief Constructor
    explicit ActionSpaceExperienceAcceleratableMixin() = default;

    /// @brief Destructor
    ~ActionSpaceExperienceAcceleratableMixin() = default;

    /// @brief Set the experiences. The ExperiencesCollective object includes some path segments that an agent has already carried out in the environment and may or may not be applicable in a new world configuration.
    void setExperiencesCollective(const std::shared_ptr<ExperiencesCollective>& expriences_collective) {
        experiences_collective_ptr_ = expriences_collective;
    }

    void addPathExperienceToExperiencesCollective(const std::shared_ptr<PathExperience>& experience) {
        experiences_collective_ptr_->addPathExperience(experience);
    }

    void addTimedPathExperienceToExperiencesCollective(const std::shared_ptr<PathExperience>& experience) {
        PathType path_wo_time = experience->getPath(); // A copy of the path that will be modified to remove the time component.
        for (auto& state : path_wo_time) {
            state.pop_back();
        }
        std::shared_ptr<PathExperience> experience_wo_time = std::make_shared<PathExperience>(path_wo_time, experience->getPathTransitionCosts());
        experiences_collective_ptr_->addPathExperience(experience_wo_time);
    }

    /// @brief Clear the experiences.
    void clearPathExperiences() { experiences_collective_ptr_->clear(); }

    /// @brief Set the experiences context.
    /// @param context The experiences context to set.
    void setExperiencesContext(const std::shared_ptr<ExperiencesContext>& context) {
        experiences_collective_ptr_->setContext(context);
    }

    /// @brief Get subpaths that are valid for the given state. These are all the suffixes of the path experiences that include the given state, starting from this state. 
    virtual void getValidExperienceSubpathsFromState(int state_id, std::vector<std::vector<int>>& subpaths, std::vector<std::vector<double>>& subpath_transition_costs) = 0;

    /// @brief Get successors with experience acceleration.
    /// @param state_id The state id.
    /// @param successors The successors to be updated.
    /// @param costs The costs to be updated.
    virtual bool getSuccessorsExperienceAccelerated(int state_id,
                                                    std::vector<std::vector<int>>& successors,
                                                    std::vector<std::vector<double>>& costs) = 0;

    
    // Member variables.
    /// @brief The experiences.
    std::shared_ptr<ExperiencesCollective> experiences_collective_ptr_;
};

}  // namespace ims

#endif  // SEARCH_ACTIONSPACEEXPERIENCEACCELERATABLEMIXIN_HPP
