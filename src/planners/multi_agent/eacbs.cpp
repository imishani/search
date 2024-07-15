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
 * \file   eacbs.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   07/07/23
 */

#include <search/planners/wastar.hpp>
#include <search/planners/multi_agent/eacbs.hpp>
#include <search/planners/multi_agent/cbs.hpp>
//
ims::EACBS::EACBS(const ims::EACBSParams& params) : params_(params), CBS(params) {}
//
//void ims::EACBS::initializePlanner(std::vector<std::shared_ptr<ConstrainedActionSpace>>& action_space_ptrs,
//                        const std::vector<StateType>& starts, const std::vector<StateType>& goals){
//
//    // Cast the action spaces to ExperienceAcceleratedConstrainedActionSpace.
//    std::vector<std::shared_ptr<ExperienceAcceleratedConstrainedActionSpace>> eac_action_space_ptrs;
//    for (auto& action_space_ptr : action_space_ptrs) {
//        eac_action_space_ptrs.push_back(std::dynamic_pointer_cast<ExperienceAcceleratedConstrainedActionSpace>(action_space_ptr));
//    }
//
//    // Check that all casts succeeded.
//    for (auto& action_space_ptr : eac_action_space_ptrs) {
//        if (action_space_ptr == nullptr) {
//            throw std::runtime_error("Could not cast action space to ExperienceAcceleratedConstrainedActionSpace");
//        }
//    }
//
//    // Call the other initializePlanner method.
//    initializePlanner(eac_action_space_ptrs, starts, goals);
//}
//
//void ims::EACBS::initializePlanner(std::vector<std::shared_ptr<ConstrainedActionSpace>>& action_space_ptrs, const std::vector<std::string>& agent_names, const std::vector<StateType>& starts, const std::vector<StateType>& goals){
//    // Cast the action spaces to ExperienceAcceleratedConstrainedActionSpace.
//    std::vector<std::shared_ptr<ExperienceAcceleratedConstrainedActionSpace>> eac_action_space_ptrs;
//    for (auto& action_space_ptr : action_space_ptrs) {
//        eac_action_space_ptrs.push_back(std::dynamic_pointer_cast<ExperienceAcceleratedConstrainedActionSpace>(action_space_ptr));
//    }
//
//    // Check that all casts succeeded.
//    for (auto& action_space_ptr : eac_action_space_ptrs) {
//        if (action_space_ptr == nullptr) {
//            throw std::runtime_error("Could not cast action space to ExperienceAcceleratedConstrainedActionSpace");
//        }
//    }
//
//    // Call the other initializePlanner method.
//    initializePlanner(eac_action_space_ptrs, agent_names, starts, goals);
//}
//
//void ims::EACBS::initializePlanner(std::vector<std::shared_ptr<ExperienceAcceleratedConstrainedActionSpace>>& action_space_ptrs,
//                                 const std::vector<StateType>& starts, const std::vector<StateType>& goals) {
//    // Store the action spaces. This must happen before checking for the validity of the start and end states.
//    agent_action_space_ptrs_ = action_space_ptrs;
//
//    // Check if the inputs are valid.
//    if (starts.size() != agent_action_space_ptrs_.size() || goals.size() != agent_action_space_ptrs_.size()) {
//        throw std::runtime_error("Start state vector size (" + std::to_string(starts.size()) + ") does not match the number of agents (" + std::to_string(agent_action_space_ptrs_.size()) + ")");
//    }
//
//    // Check start and end states for validity. Individually and between agents.
//    std::vector<std::shared_ptr<ims::ConstrainedActionSpace>> constrained_action_space_ptrs(agent_action_space_ptrs_.begin(), agent_action_space_ptrs_.end());
//    verifyStartAndGoalInputStates(starts, goals, constrained_action_space_ptrs);
//
//    // Store the starts and goals.
//    starts_ = starts;
//    goals_ = goals;
//
//    // Set the number of agents.
//    num_agents_ = (int)starts.size();
//
//    // Create all the low-level planners.
//    for (size_t i{0}; i < starts.size(); ++i) {
//        ims::EAwAStarUniformCostParams eawastar_params_(params_.low_level_heuristic_ptrs[i], params_.weight_low_level_heuristic);
//
//        agent_planner_ptrs_.push_back(std::make_shared<ims::EAwAStarUniformCost>(eawastar_params_));
//    }
//}
//
//void ims::EACBS::createRootInOpenList() {
//    // Generate a plan for each of the agents.
//    MultiAgentPaths initial_paths;
//    std::unordered_map<int, double> initial_paths_costs;
//    std::unordered_map<int, std::vector<double>> initial_paths_transition_costs;
//
//    for (size_t i{0}; i < num_agents_; ++i) {
//        std::vector<StateType> path;
//        agent_planner_ptrs_[i]->initializePlanner(agent_action_space_ptrs_[i], starts_[i], goals_[i]);
//        bool is_plan_success = agent_planner_ptrs_[i]->plan(path);
//
//        // Add the number of low level nodes to the counter.
//        stats_.bonus_stats["num_low_level_expanded"] += agent_planner_ptrs_[i]->stats_.num_expanded;
//
//        // If there is no path for this agent, then this is not a valid state. Do not add a new state to the open list.
//        if (!is_plan_success) {
//            std::cout << RED << "No path found for agent " << i << " in the initial planning phase." << RESET << std::endl;
//            stats_.cost = -1;
//            stats_.time = -1;
//            return;
//        }
//        // Fix the last path state to have a correct time and not -1.
//        path.back().back() = path.size() - 1;
//
//        // We use a map since down the line we may only store paths for some agents.
//        // initial_paths.insert(std::make_pair(i, path));
//        initial_paths[i] = path;
//
//        // Compute the cost of the path.
//        initial_paths_costs[i] = agent_planner_ptrs_[i]->stats_.cost;
//        initial_paths_transition_costs[i] = agent_planner_ptrs_[i]->stats_.transition_costs;
//    }
//
//
//    // Create the initial EACBS state to the open list. This planner does not interface with an action space, so it does not call the getOrCreateRobotState to retrieve a new-state index. But rather decides on a new index directly and creates a search-state index with the getOrCreateSearchState method. Additionally, there is no goal specification for EACBS, so we do not have a goal state.
//    int start_ind_ = 0;
//    auto start_ = getOrCreateSearchState(start_ind_);
//
//    // Set the initial EACBS state.
//    start_->parent_id = PARENT_TYPE(START);
//    start_->paths = initial_paths;
//    start_->paths_costs = initial_paths_costs;
//    start_->paths_transition_costs = initial_paths_transition_costs;
//
//    // Set the cost of the EACBSState start_.
//    start_->f = std::accumulate(initial_paths_costs.begin(), initial_paths_costs.end(), 0.0, [](double acc, const std::pair<int, double>& path_cost) { return acc + path_cost.second; });
//    start_->setOpen();
//
//    // Add the agent_names to the constraints collectives.
//    for (const auto& agent_id_and_constraints_collective : start_->constraints_collectives){
//        int agent_id = agent_id_and_constraints_collective.first;
//
//        start_->constraints_collectives.at(agent_id).getConstraintsContext()->agent_names = agent_names_;
//    }
//
//    // Push the initial EACBS state to the open list.
//    open_.push(start_);
//
//}
//
//void ims::EACBS::initializePlanner(std::vector<std::shared_ptr<ExperienceAcceleratedConstrainedActionSpace>>& action_space_ptrs, const std::vector<std::string> & agent_names, const std::vector<StateType>& starts, const std::vector<StateType>& goals){
//                        agent_names_ = agent_names;
//                        initializePlanner(action_space_ptrs, starts, goals);
//                        }
//
auto ims::EACBS::getSearchState(int state_id) -> ims::EACBS::SearchState* {
    assert(state_id < states_.size() && state_id >= 0);
    return states_[state_id];
}

auto ims::EACBS::getOrCreateSearchState(int state_id) -> ims::EACBS::SearchState* {
    if (state_id >= states_.size()) {
        states_.resize(state_id + 1, nullptr);
    }
    if (states_[state_id] == nullptr) {
        assert(state_id < states_.size() && state_id >= 0);
        states_[state_id] = new SearchState;
        states_[state_id]->state_id = state_id;
    }
    return states_[state_id];
}
//
//bool ims::EACBS::plan(MultiAgentPaths& paths) {
//    startTimer();
//    createRootInOpenList();
//    int iter{0};
//    while (!open_.empty() && !isTimeOut()) {
//        // Report progress every 100 iterations
//        if (iter % 10 == 0) {
//            std::cout << "EACBS CT open size: " << open_.size() << std::endl;
//        }
//
//        // Get the state of least cost.
//        auto state = open_.min();
//        open_.pop();
//
//        // Set the state to closed.
//        state->setClosed();
//
//        // Expand the state.
//        // Check for conflicts in this new state.
//        agent_action_space_ptrs_[0]->getPathsConflicts(std::make_shared<MultiAgentPaths>(state->paths),
//                                                       state->unresolved_conflicts,
//                                                       getConflictTypes(),
//                                                       1,
//                                                       agent_names_);
//
//        // Before we actually expand the state, we check if there is even a need to do so. If there are no conflicts, then this is a goal state. Set the goal state and return.
//        if (state->unresolved_conflicts.empty()) {
//            std::cout << "No conflicts found. This is a goal state." << std::endl;
//            goal_ = state->state_id;
//            getTimeFromStart(stats_.time);
//            stats_.cost = state->f;
//            paths = state->paths;
//            stats_.num_expanded = iter;
//            return true;
//        }
//
//        // Otherwise, expand the state.
//        expand(state->state_id);
//        ++iter;
//
//    }
//    getTimeFromStart(stats_.time);
//    return false;
//}
//
//void ims::EACBS::expand(int state_id) {
//    auto state = getSearchState(state_id);
//    std::vector<int> successors;
//
//    // First, convert all conflicts to pairs of (agent_id, constraint). In vanilla EACBS, there is only one conflict found from a set of paths (the first/random one), and that would yield two constraints. To allow for more flexibility, we do not restrict the data structure to only two constraints per conflict.
//    std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>> constraints = conflictsToConstraints(state->unresolved_conflicts);
//
//    // Second, iterate through the constraints, and for each one, create a new search state. The new search state is a copy of the previous search state, with the constraint added to the constraints collective of the agent.
//    // For each constraint, split the state into branches. Each branch will be a new state in the search tree.
//    for (auto& agent_id_constraints : constraints){
//
//        // The first element is the agent ID.
//        int agent_id = agent_id_constraints.first;
//
//        // The second element is a shared pointer to the constraint.
//        auto constraints_ptr = agent_id_constraints.second;
//
//        // Create a new search state. In this implementation ther is no check for whether the search state already exists (same starts, goals, and constraints), so we always create a new search state and push(...) it to the open list. Otherwise, we would check if the search state already exists, and if so, we would update(...) the open list heap.
//        // NOTE(yoraish): lock below for parallelization. Think of copying action-spaces and planners as well for each thread?
//        int new_state_id = (int)states_.size();
//        auto new_state = getOrCreateSearchState(new_state_id);
//        // NOTE(yoraish): lock above for parallelization.
//
//        // The new state is a copy of the previous state.
//        new_state->parent_id = state->state_id;
//        new_state->paths = state->paths;
//        new_state->paths_costs = state->paths_costs;
//        new_state->paths_transition_costs = state->paths_transition_costs;
//        new_state->f = state->f;
//        new_state->constraints_collectives = state->constraints_collectives;
//        new_state->experiences_collectives = state->experiences_collectives;
//        // NOTE(yoraish): we do not copy over the conflicts, since they will be recomputed in the new state. We could consider keeping a history of conflicts in the search state, with new conflicts being marked as such.
//
//        // Update the constraints collective to also include the new constraint.
//        new_state->constraints_collectives[agent_id].addConstraints(constraints_ptr);
//
//        // Update the action-space. Start with the constraints and their context.
//        std::shared_ptr<ConstraintsCollective> constraints_collective_ptr = std::make_shared<ConstraintsCollective>(new_state->constraints_collectives[agent_id]);
//        std::shared_ptr<ConstraintsContext> context_ptr = std::make_shared<ConstraintsContext>();
//        // context_ptr->agent_paths = new_state->paths;
//        context_ptr->agent_names = agent_names_;
//        constraints_collective_ptr->setContext(context_ptr);
//        agent_action_space_ptrs_[agent_id]->setConstraintsCollective(constraints_collective_ptr);
//
//
//
//        // Update the action-space with an experience yielded by the previous path of this agent. Notice that this is slightly different from what we do with constraints. With the constraints, we first update the constraints stored in the search state and then set this same constraints collective in the action space. With experiences though, we would like to (perhaps) reuse experiences from different CT nodes, so we directly add experiences to the action space, which is shared by all invocations be this agents' low-level planner across CT subtrees.
//        // NOTE(yoraish): if we do not want to share experiences across CT subtrees, then we should work in the same way as the constraints (keep track of experiences in the search state, and update (set) the action space with the experiences stored in the search state (not to add to them)).
//
//        switch (experience_reuse_type_) {
//            case ExperienceReuseType::NONE: {
//                // std::cout << "Experience reuse type is NONE. Not updating the experiences collective." << std::endl;
//                break;
//            }
//            case ExperienceReuseType::PREVIOUS_SOLUTION: {
//                // std::cout << "Experience reuse type is PREVIOUS_SOLUTION. Updating the experiences collective with the previous solution." << std::endl;
//                agent_action_space_ptrs_[agent_id]->clearPathExperiences();
//                agent_action_space_ptrs_[agent_id]->addTimedPathExperienceToExperiencesCollective(std::make_shared<PathExperience>(state->paths[agent_id], state->paths_transition_costs[agent_id]));
//                break;
//            }
//            case ExperienceReuseType::CT_BRANCH: {
//                // std::cout << "Experience reuse type is CT_BRANCH. Updating the experiences collective with the solution on branch." << std::endl;
//                new_state->experiences_collectives[agent_id].addTimedPathExperience(std::make_shared<PathExperience>(state->paths[agent_id], state->paths_transition_costs[agent_id]));
//
//                // Update the action-space with the updated experiences.
//                agent_action_space_ptrs_[agent_id]->setExperiencesCollective(std::make_shared<ExperiencesCollective>(new_state->experiences_collectives[agent_id]));
//                break;
//            }
//            case ExperienceReuseType::CT_GLOBAL: {
//                // std::cout << "Experience reuse type is CT_GLOBAL. Updating the experiences collective with all previous solutions." << std::endl;
//                agent_action_space_ptrs_[agent_id]->addTimedPathExperienceToExperiencesCollective(std::make_shared<PathExperience>(state->paths[agent_id], state->paths_transition_costs[agent_id]));
//                break;
//            }
//            default: {
//                throw std::runtime_error("Unknown experience reuse type.");
//            }
//        }
//
//        // Update the low-level planner for this agent.
//        agent_planner_ptrs_[agent_id]->initializePlanner(agent_action_space_ptrs_[agent_id], starts_[agent_id], goals_[agent_id]);
//
//        // Replan for this agent and update the stored path associated with it in the new state. Update the cost of the new state as well.
//        // The replanning will be using the updated constraints and experiences stored within the action space.
//        new_state->paths[agent_id].clear();
//        agent_planner_ptrs_[agent_id]->plan(new_state->paths[agent_id]);
//        new_state->paths_transition_costs[agent_id] = agent_planner_ptrs_[agent_id]->stats_.transition_costs;
//        new_state->paths_costs[agent_id] = agent_planner_ptrs_[agent_id]->stats_.cost;
//        new_state->f = std::accumulate(new_state->paths_costs.begin(), new_state->paths_costs.end(), 0.0, [](double acc, const std::pair<int, double>& path_cost) { return acc + path_cost.second; });
//
//        // Add the number of low level nodes to the counter.
//        stats_.bonus_stats["num_low_level_expanded"] += agent_planner_ptrs_[agent_id]->stats_.num_expanded;
//
//        // If there is no path for this agent, then this is not a valid state. Discard it.
//        if (new_state->paths[agent_id].empty()) {
//            delete new_state;
//            continue;
//        }
//
//        // The goal state returned is at time -1. We need to fix that.
//        new_state->paths[agent_id].back().back() = new_state->paths[agent_id].size() - 1;
//
//        // Push the new state to the open list.
//        open_.push(new_state);
//        new_state->setOpen();
//
//        // Delete the previous state but keep the entry in the states_ vector.
//        // state = nullptr;
//    }
//}
//
//void ims::EACBS::setStateVals(int state_id, int parent_id, double cost) {
//    auto state = getSearchState(state_id);
//    auto parent = getSearchState(parent_id);
//    state->parent_id = parent_id;
//}
//
//std::vector<std::pair<int, std::vector<std::shared_ptr<ims::Constraint>>>> ims::EACBS::conflictsToConstraints(const std::vector<std::shared_ptr<ims::Conflict>>& conflicts) {
//    std::vector<std::pair<int, std::vector<std::shared_ptr<ims::Constraint>>>> agent_constraints;
//
//    // Iterate through the conflicts and convert them to constraints.
//    for (auto& conflict_ptr : conflicts) {
//        // Create a new constraint given the conflict.
//        if (conflict_ptr->type == ConflictType::VERTEX) {
//            auto* vertex_conflict_ptr = dynamic_cast<VertexConflict*>(conflict_ptr.get());
//            // Check if the conversion succeeded.
//            if (vertex_conflict_ptr == nullptr) {
//                throw std::runtime_error("Conflict is a vertex conflict, but could not be converted to a VertexConflict.");
//            }
//
//            // For each affected agent (2, in EACBS), create a new constraint, and a search state for each as well.
//            ims::conflict_conversions::vertexConflictToVertexConstraints(vertex_conflict_ptr, agent_constraints);
//        }
//
//        // Otherwise, if the conflict is an edge conflict, add an edge constraint to each of the two affected agents.
//        else if (conflict_ptr->type == ConflictType::EDGE) {
//            auto* edge_conflict_ptr = dynamic_cast<EdgeConflict*>(conflict_ptr.get());
//
//            // Check if the conversion succeeded.
//            if (edge_conflict_ptr == nullptr) {
//                throw std::runtime_error("Conflict is an edge conflict, but could not be converted to an EdgeConflict.");
//            }
//            ims::conflict_conversions::edgeConflictToEdgeConstraints(edge_conflict_ptr, agent_constraints);
//        }
//
//        else {
//            throw std::runtime_error("Conflict type " + std::to_string(static_cast<int>(conflict_ptr->type)) + " is not supported by EACBS.");
//        }
//    }
//
//    return agent_constraints;
//}