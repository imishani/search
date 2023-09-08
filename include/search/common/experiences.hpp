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
 * \file   experiences.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   July 13 2023
*/

#ifndef SEARCH_COMMON_EXPERIENCESS_HPP
#define SEARCH_COMMON_EXPERIENCESS_HPP

// standard includes
#include <functional>
#include <algorithm>
#include <utility>
#include <vector>
#include <memory>
#include <unordered_map>
#include <map>

// Project includes.
#include <search/common/types.hpp>

namespace ims {

enum class ExperienceType {
    UNSET = -1,
    PATH = 0,
};

/// @brief Base class for all search experiences.
struct ExperienceBase {
    /// @brief Constructor
    explicit ExperienceBase() = default;

    /// @brief Virtual destructor.
    virtual ~ExperienceBase() = default;

    /// @brief Get a string with information about the experience.
    virtual std::string toString() const = 0;

    /// @brief The type of the experience.
    ExperienceType type;
};

/// @brief path experience. We simply call this Experience, instead of PathExperience, since this is the most common type of experience when working in motion planning.
struct Experience : public ExperienceBase {
    /// @brief The path.
    PathType path_;

    /// @brief The path transition costs.
    std::vector<double> path_transition_costs_;

    /// @brief Constructor
    explicit Experience(const PathType& path, const std::vector<double>& path_transition_costs) : path_(path), path_transition_costs_(path_transition_costs), ExperienceBase() {
        type = ExperienceType::PATH;
    }

    /// @brief Virtual destructor.
    ~Experience() override = default;

    /// @brief Get a string with information about the experience.
    std::string toString() const override {
        std::string str = "Experience: ";
        for (const auto& state : path_) {
            str += "[";
            for (auto val : state) {
                str += std::to_string(val) + " ";
            }
            str += "]\n";
        }
        return str;
    }

    /// @brief Get the path.
    /// @return The path.
    const PathType& getPath() const {
        return path_;
    }

    /// @brief Get the path transition costs.
    /// @return The path transition costs.
    const std::vector<double>& getPathTransitionCosts() const {
        return path_transition_costs_;
    }
};

/// @brief Base class for all search experience contexts.
/// @details This class is used to store context variables that are required for experience checking and satisfaction. For example, in the case of PBS, the context for experience satisfaction is the paths of all the agents whose paths were already found. This class is used to store these paths, and is passed to the ConstrainedActionSpace via the setExperiencesContext method.
struct ExperiencesContext {
    /// @brief Constructor
    explicit ExperiencesContext() = default;

    /// @brief Virtual destructor.
    virtual ~ExperiencesContext() = default;

    /// @brief The type of the experience.
};

/// @brief Base class for all search experiences.
/// @details This class is used to store a set of experiences, and a context for experience satisfaction.
struct ExperiencesCollective {
    /// @brief Constructor
    explicit ExperiencesCollective() = default;

    /// @brief Virtual destructor.
    virtual ~ExperiencesCollective() = default;

    // Setters.
    /// @brief Set the experiences.
    /// @param experiences The experiences to set.
    void setExperiences(const std::vector<std::shared_ptr<Experience>>& experiences) {
        experiences_ptrs_ = experiences;

        // Populate the map betwee states and experiences.
        for (const auto& experience_ptr : experiences_ptrs_) {
            for (const auto& state : experience_ptr->getPath()) {
                state_to_experiences_ptrs_[state].push_back(experience_ptr);
            }
        }
    }

    void addExperience(const std::shared_ptr<Experience>& experience) {
        experiences_ptrs_.push_back(experience);

        // Point each of the states in the experience to this experience.
        for (const auto& state : experience->getPath()) {
            state_to_experiences_ptrs_[state].push_back(experience);
        }
    }

    void addExperiences(const std::vector<std::shared_ptr<Experience>>& experiences) {
        for (const std::shared_ptr<Experience>& experience_ptr : experiences) {
            addExperience(experience_ptr);
        }
    }

    void getSubExperiencesFromState(const StateType& state, std::vector<PathType>& experience_subpaths, std::vector<std::vector<double>>&experience_subpaths_transition_costs) const {

        // Find all the experiences that contain this state. Return if there are none.
        if (state_to_experiences_ptrs_.find(state) == state_to_experiences_ptrs_.end()) {
            return;
        }
        // return; /////////

        std::vector<std::shared_ptr<Experience>> all_experiences_with_state = state_to_experiences_ptrs_.at(state);


        // Remove all prefixes in the experiences that come before this state.
        for (const auto& experience_ptr : all_experiences_with_state) {
            // Find the index of the state in the experience.
            auto it = std::find(experience_ptr->getPath().begin(), experience_ptr->getPath().end(), state);
            int state_index = std::distance(experience_ptr->getPath().begin(), it);

            // Add the subpath to the experiences.
            experience_subpaths.push_back(PathType(experience_ptr->getPath().begin() + state_index, experience_ptr->getPath().end()));
            experience_subpaths_transition_costs.push_back(std::vector<double>(experience_ptr->getPathTransitionCosts().begin() + state_index, experience_ptr->getPathTransitionCosts().end()));
        }

    }

    /// @brief Set the experiences context.
    /// @param context The experiences context to set.
    void setContext(const std::shared_ptr<ExperiencesContext>& context) {
        context_ptr_ = context;
    }

    /// @brief Clear the experiences context.
    void clearContext() {
        context_ptr_.reset();
    }

    /// @brief Clear the experiences.
    void clear() {
        experiences_ptrs_.clear();
        state_to_experiences_ptrs_.clear();
    }

    // Getters.
    /// @brief Get the context pointer.
    /// @return The context pointer.
    std::shared_ptr<ExperiencesContext> getExperiencesContext() const {
        return context_ptr_;
    }

    /// @brief Get the experiences.
    /// @return The experiences.
    const std::vector<std::shared_ptr<Experience>>& getExperiences() const {
        return experiences_ptrs_;
    }

    /// String.
    /// @brief Get a string with information about the experience.
    std::string toString() const {
        std::string str = "ExperiencesCollective: \n";
        for (const auto& experience_ptr : experiences_ptrs_) {
            str += "    " + experience_ptr->toString() + "\n";
        }
        return str;
    }

private:
    /// @brief Map from a timestep to a set of experience pointers.
    std::map<StateType, std::vector<std::shared_ptr<Experience>>> state_to_experiences_ptrs_ = {};

    // Member Variables.
    /// @brief The type of the experience.
    std::shared_ptr<ExperiencesContext> context_ptr_ = std::make_shared<ExperiencesContext>();

    /// @brief The type of the experience.
    std::vector<std::shared_ptr<Experience>> experiences_ptrs_ = {};
};

/// @brief An object for mapping [agent_ids] to a set of experiences collective.
using MultiAgentExperiencesCollective = std::unordered_map<int, ExperiencesCollective>;

}  // namespace ims

#endif //SEARCH_COMMON_EXPERIENCESS_HPP
