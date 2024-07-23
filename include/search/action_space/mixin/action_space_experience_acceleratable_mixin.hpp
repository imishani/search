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
    /// @brief Add a timed path experience to the experiences collective. This strips the states from their time dimension in all sequences of the path.
    /// \param experience
    void addTimedPathExperienceToExperiencesCollective(const std::shared_ptr<PathExperience>& experience) {
        SeqPathType seq_path_wo_time = experience->getSeqPath(); // A copy of the path that will be modified to remove the time component.
        for (auto& seq : seq_path_wo_time) {
            for (auto& state : seq) {
                state.pop_back();
            }
        }

        std::shared_ptr<PathExperience> experience_wo_time = std::make_shared<PathExperience>(seq_path_wo_time, experience->getSeqPathTransitionCosts());
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
    virtual void getValidExperienceSubpathsFromState(int state_id,
                                             std::vector<std::vector<std::vector<int>>>& experience_seq_subpaths,
                                             std::vector<std::vector<std::vector<double>>>& experience_seq_subpaths_transition_costs) = 0;

    /// @brief Get successors with experience acceleration.
    /// @param state_id The state id.
    /// @param successors The successors to be updated.
    /// @param costs The costs to be updated.
    virtual bool getSuccessorsExperienceAccelerated(int curr_state_ind,
                            std::vector<std::vector<int>>& seqs_state_ids,
                            std::vector<std::vector<double>> & seqs_transition_costs) = 0;

    virtual bool getSuccessorsExperienceAccelerated(int curr_state_ind,
                            std::vector<int> & successors,
                            std::vector<double> & costs) {
        std::vector<std::vector<int>> seqs_state_ids;
        std::vector<std::vector<double>> seqs_transition_costs;
        getSuccessorsExperienceAccelerated(curr_state_ind, seqs_state_ids, seqs_transition_costs);
        for (size_t i = 0; i < seqs_state_ids.size(); i++) {
            if (seqs_state_ids[i].size() != 2){
                std::cout << RED << "getSuccessors: The seqs_state_ids[i] should have only two elements (the parent and child states). Instead, it has " << seqs_state_ids[i].size() << " elements." << RESET << std::endl;
                std::cout << RED << "Edge state ids: " << seqs_state_ids[i] << RESET << std::endl;
                std::cout << RED << "GetSuccessors would have returned the edge state ids: [" << seqs_state_ids[i].front() << ", " << seqs_state_ids[i].back() << "], which would lose information." << RESET << std::endl;
                throw std::runtime_error("getSuccessors: The seqs_state_ids should have only two elements (the parent and child states).");
            }
            successors.push_back(seqs_state_ids[i].back());
            costs.push_back(std::accumulate(seqs_transition_costs[i].begin(), seqs_transition_costs[i].end(), 0.0));
        }
        return true;
    }

    
    // Member variables.
    /// @brief The experiences.
    std::shared_ptr<ExperiencesCollective> experiences_collective_ptr_;
};

}  // namespace ims

#endif  // SEARCH_ACTIONSPACEEXPERIENCEACCELERATABLEMIXIN_HPP
