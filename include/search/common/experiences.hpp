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
#include <unordered_set>
#include <map>
#include <set>

// Project includes.
#include <search/common/types.hpp>


// Define a custom hash function for a pair of vectors using Boost
namespace std {
    template <>
    struct hash< std::pair< StateType, StateType > > {
        using argument_type = std::pair<StateType, StateType>;
        using result_type = std::size_t;
        
        result_type operator()(const argument_type& p) const {
            size_t seed = 0;
            boost::hash_combine(seed, p.first);
            boost::hash_combine(seed, p.second);
            return seed;
        }
    };
}

namespace ims {

enum class ExperienceType {
    UNSET = -1,
    PATH = 0,
    TRANSITION = 1,
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
    ExperienceType type = ExperienceType::UNSET;
};

/// @brief path experience PathExperience. This is one of the most common type of experience when working in motion planning.
struct PathExperience : public ExperienceBase {
    /// @brief The path.
    PathType path_;

    /// @brief The path transition costs.
    std::vector<double> path_transition_costs_;

    /// @brief Constructor
    explicit PathExperience(PathType path, const std::vector<double>& path_transition_costs) : path_(std::move(path)), path_transition_costs_(path_transition_costs), ExperienceBase() {
        type = ExperienceType::PATH;
    }

    /// @brief Virtual destructor.
    ~PathExperience() override = default;

    /// @brief Get a string with information about the experience.
    std::string toString() const override {
        std::string str = "PathExperience: ";
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


/// @brief path experience PathExperience. This is one of the most common type of experience when working in motion planning.
struct TransitionExperience : public ExperienceBase {
    /// @brief The states involved in the transition. Note that these states are usually not timed.
    StateType state_from_;
    StateType state_to_;
    bool is_valid_ = true;

    /// @brief The path transition costs.
    double transition_cost_;

    /// @brief Constructor
    TransitionExperience(const StateType& state_from, const StateType& state_to, const bool is_valid) :
    state_from_(state_from), state_to_(state_to), is_valid_(is_valid), ExperienceBase() {
        type = ExperienceType::TRANSITION;
    }

    TransitionExperience(const std::pair<StateType, StateType>& state_pair, const bool is_valid) : state_from_(state_pair.first), state_to_(state_pair.second), is_valid_(is_valid), ExperienceBase() {
        type = ExperienceType::TRANSITION;
    }

    /// @brief Virtual destructor.
    ~TransitionExperience() override = default;

    /// @brief Get a string with information about the experience.
    std::string toString() const override {
        std::string str = "TransitionExperience: [";
        for (double val : state_from_) {
            str += std::to_string(val) + " ";
        }
        str += "] -> [";
        for (double val : state_to_) {
            str += std::to_string(val) + " ";
        }
        str += "]\n";
        return str;
    }

    /// @brief Get the involved states.
    /// @return 
    void getStates(StateType& state_from, StateType& state_to) const {
        state_from = state_from_;
        state_to = state_to_;
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
    void setPathExperiences(const std::vector<std::shared_ptr<PathExperience>>& experiences) {
        path_experiences_ptrs_ = experiences;

        // Populate the map betwee states and experiences: point each of the states in the experience to this experience. 
        for (const auto& experience_ptr : path_experiences_ptrs_) {
            // If a state appears multiple times in this experience, then skip it after the first time.
            std::set<StateType> states_seen_in_experience;
            for (const auto& state : experience_ptr->getPath()) {
                if (states_seen_in_experience.find(state) != states_seen_in_experience.end()) {
                    continue;
                }
                state_to_path_experiences_ptrs_[state].push_back(experience_ptr);
                states_seen_in_experience.insert(state);
            }
        }
    }

    void addPathExperience(const std::shared_ptr<PathExperience>& experience) {
        // TODO(yoraish) IMPORTANT: Check if this experience is new. Unsure how to do this well.

        path_experiences_ptrs_.push_back(experience);
        
        // Point each of the states in the experience to this experience.
        // Keep track of the states that have already been seen in this experience. This is to avoid adding the same experience multiple times for the same state.
        std::set<StateType> states_seen_in_experience;
        for (const auto& state : experience->getPath()) {
            if (states_seen_in_experience.find(state) != states_seen_in_experience.end()) {
                continue;
            }
            state_to_path_experiences_ptrs_[state].push_back(experience);
            states_seen_in_experience.insert(state);
        }
    }

    void addPathExperiences(const std::vector<std::shared_ptr<PathExperience>>& experiences) {
        for (const std::shared_ptr<PathExperience>& experience_ptr : experiences) {
            addPathExperience(experience_ptr);
        }
    }

    void addTimedPathExperience(const std::shared_ptr<PathExperience>& experience) {

        // A copy of the path that will be modified to remove the time component and remove loops.
        PathType path_wo_time;
        std::vector<double> path_transition_costs;
        for (size_t i{0}; i < experience->getPath().size() ; ++i){
            StateType state = experience->getPath()[i];
            state.pop_back();

            // If the current state is the same as the previous state, then skip it.
            if (i > 0 && state == path_wo_time.back()){
                continue;
            }
            path_wo_time.push_back(state);
            path_transition_costs.push_back(experience->getPathTransitionCosts()[i]);
        }
        path_transition_costs.back() = 0.0;

        std::shared_ptr<PathExperience> experience_wo_time = std::make_shared<PathExperience>(path_wo_time, experience->getPathTransitionCosts());
        addPathExperience(experience_wo_time);
    }

    /// @brief Get the experiences that contain this state. These are subexperiences since those that are returned all begin with the query state. If they started before it, then only their suffixes are returned.
    /// @param state The state to get the experiences for.
    /// @param experience_subpaths The vector of subpaths -- to be updated with the subpaths.
    /// @param experience_subpaths_transition_costs The vector of subpath transition costs -- to be updated with the subpath transition costs.
    void getSubExperiencesFromState(const StateType& state, std::vector<PathType>& experience_subpaths, std::vector<std::vector<double>>&experience_subpaths_transition_costs) const {

        // Find all the experiences that contain this state. Return if there are none.
        if (state_to_path_experiences_ptrs_.find(state) == state_to_path_experiences_ptrs_.end()) {
            return;
        }

        std::vector<std::shared_ptr<PathExperience>> all_experiences_with_state = state_to_path_experiences_ptrs_.at(state);

        // Remove all prefixes in the experiences that come before this state.
        for (const auto& experience_ptr : all_experiences_with_state) {
            // Find the index of the state in the experience.
            auto it = std::find(experience_ptr->getPath().begin(), experience_ptr->getPath().end(), state);
            int state_index = (int)std::distance(experience_ptr->getPath().begin(), it);

            // Add the subpath to the experiences.
            experience_subpaths.emplace_back(experience_ptr->getPath().begin() + state_index, experience_ptr->getPath().end());
            experience_subpaths_transition_costs.emplace_back(experience_ptr->getPathTransitionCosts().begin() + state_index, experience_ptr->getPathTransitionCosts().end());
        }
    }

    /// @brief Add a transition to the experiences collective.
    /// @param experience 
    void addTransitionExperience(const std::shared_ptr<TransitionExperience>& experience) {

        // Add this transition experience to the collection of transition experiences.
        StateType state_from;
        StateType state_to;
        experience->getStates(state_from, state_to);

        // Create a pair.
        std::pair<StateType, StateType> state_pair = std::make_pair(state_from, state_to);

        // Add/update the collection.
        state_pair_to_transition_experience_ptr_[state_pair] = experience;
    }

    bool isTransitionInExperiences(const std::pair<StateType, StateType>& state_pair) const {
        // Check if the pair is in the collection.
        return state_pair_to_transition_experience_ptr_.find(state_pair) != state_pair_to_transition_experience_ptr_.end();
    }

    bool isTransitionValid(const std::pair<StateType, StateType>& state_pair) const {
        // Check if the pair is in the collection.
        assert(isTransitionInExperiences(state_pair));

        // Check if the transition is valid.
        return state_pair_to_transition_experience_ptr_.at(state_pair)->is_valid_;
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
        path_experiences_ptrs_.clear();
        state_to_path_experiences_ptrs_.clear();
    }

    // Getters.
    /// @brief Get the context pointer.
    /// @return The context pointer.
    std::shared_ptr<ExperiencesContext> getExperiencesContext() const {
        return context_ptr_;
    }

    /// @brief Get the experiences.
    /// @return The experiences.
    const std::vector<std::shared_ptr<PathExperience>>& getExperiences() const {
        return path_experiences_ptrs_;
    }

    /// String.
    /// @brief Get a string with information about the experience.
    std::string toString() const {
        std::string str = "ExperiencesCollective: \n";
        for (const auto& experience_ptr : path_experiences_ptrs_) {
            str += "    " + experience_ptr->toString() + "\n";
        }
        return str;
    }

private:
    /// @brief Map from a timestep to a set of path experience pointers.
    std::unordered_map<StateType, std::vector<std::shared_ptr<PathExperience>>, StateTypeHash> state_to_path_experiences_ptrs_ = {};
    std::unordered_map<std::pair<StateType, StateType>, std::shared_ptr<TransitionExperience>> state_pair_to_transition_experience_ptr_ = {};

    // Member Variables.
    /// @brief The type of the experience.
    std::shared_ptr<ExperiencesContext> context_ptr_ = std::make_shared<ExperiencesContext>();

    /// @brief Keep track of all the path experiences that have been added.
    std::vector<std::shared_ptr<PathExperience>> path_experiences_ptrs_ = {};
    std::vector<std::shared_ptr<TransitionExperience>> transition_experiences_ptrs_ = {};
};

/// @brief An object for mapping [agent_ids] to a set of experiences collective.
using MultiAgentExperiencesCollective = std::unordered_map<int, ExperiencesCollective>;

}  // namespace ims

#endif //SEARCH_COMMON_EXPERIENCESS_HPP
