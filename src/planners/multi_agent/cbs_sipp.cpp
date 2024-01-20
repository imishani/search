/*
 * Copyright (C) 2024, Yorai Shaoul
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
 * \file   cbs_sipp.cpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   2024-01-17
 */

#include <search/planners/sipp.hpp>
#include <search/planners/multi_agent/cbs_sipp.hpp>

ims::CBSSIPP::CBSSIPP(const ims::CBSSIPPParams& params) : params_(params), ims::CBS(params) {
}

void ims::CBSSIPP::initializePlanner(std::vector<std::shared_ptr<ConstrainedActionSpace>>& action_space_ptrs,
                                 const std::vector<StateType>& starts, const std::vector<StateType>& goals) {
    // Reset the open list. Do this by deleteing it and creating it again. TODO(yoraish): add `clear` method to our custom queues.
    open_->clear();
    
    // Store the action spaces. This must happen before checking for the validity of the start and end states.
    agent_action_space_ptrs_ = action_space_ptrs;

    // Check if the inputs are valid.
    if (starts.size() != agent_action_space_ptrs_.size() || goals.size() != agent_action_space_ptrs_.size()) {
        throw std::runtime_error("Start state vector size (" + std::to_string(starts.size()) + ") does not match the number of agents (" + std::to_string(agent_action_space_ptrs_.size()) + ")");
    }

    // Check start and end states for validity. Individually and between agents.
    verifyStartAndGoalInputStates(starts, goals);

    // Store the starts and goals.
    starts_ = starts;
    goals_ = goals;

    // Set the number of agents.
    num_agents_ = (int)starts.size();

    // Create all the low-level planners.
    for (size_t i{0}; i < starts.size(); ++i) {
        ims::SIPPParams sipp_params(params_.low_level_heuristic_ptrs[i], params_.weight_low_level_heuristic);
        
        agent_planner_ptrs_.push_back(std::make_shared<ims::SIPP>(sipp_params));
    }
}

void ims::CBSSIPP::createRootInOpenList(){
    // Generate a plan for each of the agents.
    MultiAgentPaths initial_paths;
    std::unordered_map<int, double> initial_paths_costs;
    std::unordered_map<int, std::vector<double>> initial_paths_transition_costs;
    std::unordered_map<int, double> initial_paths_lower_bounds;
    double initial_sum_of_path_cost_lower_bounds{0.0};

    for (size_t i{0}; i < num_agents_; ++i) {
        std::vector<StateType> path;
        agent_planner_ptrs_[i]->initializePlanner(agent_action_space_ptrs_[i], starts_[i], goals_[i]);
        bool is_plan_success = agent_planner_ptrs_[i]->plan(path);

        // Add the number of low level nodes to the counter.
        stats_.bonus_stats["num_low_level_expanded"] += agent_planner_ptrs_[i]->stats_.num_expanded;

        // If there is no path for this agent, then this is not a valid state. Do not add a new state to the open list.
        if (!is_plan_success) {
            std::cout << RED << "No path found for agent " << i << " in the initial planning phase." << RESET << std::endl;
            stats_.cost = -1;
            stats_.time = -1;
            return;
        }
        // Fix the last path state to have a correct time and not -1.
        path.back().back() = path.size() - 1;

        // We use a map since down the line we may only store paths for some agents.
        // initial_paths.insert(std::make_pair(i, path));
        initial_paths[i] = path;

        // Compute the cost of the path.
        initial_paths_costs[i] = agent_planner_ptrs_[i]->stats_.cost;
        initial_paths_transition_costs[i] = agent_planner_ptrs_[i]->stats_.transition_costs;

        // Compute the lower bound of the path.
        initial_paths_lower_bounds[i] = agent_planner_ptrs_[i]->stats_.cost;
        initial_sum_of_path_cost_lower_bounds += initial_paths_lower_bounds[i];
    }

    // Create the initial CBS state to the open list. This planner does not interface with an action space, so it does not call the getOrCreateRobotState to retrieve a new-state index. But rather decides on a new index directly and creates a search-state index with the getOrCreateSearchState method. Additionally, there is no goal specification for CBS, so we do not have a goal state.
    int start_ind_ = 0;
    auto start_ = getOrCreateSearchState(start_ind_);

    // Set the initial CBS state.
    start_->parent_id = PARENT_TYPE(START);
    start_->paths = initial_paths;
    start_->paths_costs = initial_paths_costs;
    start_->path_cost_lower_bounds = initial_paths_lower_bounds;
    start_->paths_transition_costs = initial_paths_transition_costs;

    // Set the cost of the CBSState start_.
    start_->f = std::accumulate(initial_paths_costs.begin(), initial_paths_costs.end(), 0.0, [](double acc, const std::pair<int, double>& path_cost) { return acc + path_cost.second; });
    start_->setOpen();

    // Add the agent_names to the constraints collectives.
    for (int agent_id = 0; agent_id < num_agents_; agent_id++){
        start_->constraints_collectives[agent_id] = agent_action_space_ptrs_[agent_id]->getConstraintsCollective();
        start_->constraints_collectives.at(agent_id).getConstraintsContextNonConst()->agent_names = agent_names_;
        // Add a pointer to the action space to the constraints collectives.
        start_->constraints_collectives.at(agent_id).getConstraintsContextNonConst()->action_space_ptr = agent_action_space_ptrs_[agent_id];
        std::cout << "Parent state has " << start_->constraints_collectives[agent_id].getNumSafeIntervals() << " safe intervals." << std::endl;

    }


    // Push the initial CBS state to the open list.
    open_->push(start_);
}


void ims::CBSSIPP::expand(int state_id) {
    auto state = getSearchState(state_id);
    std::vector<int> successors;
    std::vector<double> costs;

    // First, convert all conflicts to pairs of (agent_id, constraint). In vanilla CBS, there is only one conflict found from a set of paths (the first/random one), and that would yield two constraints. To allow for more flexibility, we do not restrict the data structure to only two constraints per conflict.
    std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>> constraints = conflictsToConstraints(state->unresolved_conflicts);

    // Second, iterate through the constraints, and for each one, create a new search state. The new search state is a copy of the previous search state, with the constraint added to the constraints collective of the agent.
    // For each constraint, split the state into branches. Each branch will be a new state in the search tree.
    for (auto& agent_id_constraint : constraints){

        // The first element is the agent ID.
        int agent_id = agent_id_constraint.first;

        // The second element is a shared pointer to the constraint.
        auto constraint_ptr = agent_id_constraint.second;

        // Create a new search state. In this implementation ther is no check for whether the search state already exists (same starts, goals, and constraints), so we always create a new search state and push(...) it to the open list. Otherwise, we would check if the search state already exists, and if so, we would update(...) the open list heap.
        // NOTE(yoraish): lock below for parallelization. Think of copying action-spaces and planners as well for each thread?
        int new_state_id = (int)states_.size();
        auto new_state = getOrCreateSearchState(new_state_id);
        // NOTE(yoraish): lock above for parallelization.

        // The new state is a copy of the previous state.
        new_state->parent_id = state->state_id;
        new_state->paths = state->paths;
        new_state->paths_costs = state->paths_costs;
        new_state->paths_transition_costs = state->paths_transition_costs;
        new_state->f = state->f;
        new_state->path_cost_lower_bounds = state->path_cost_lower_bounds;
        new_state->constraints_collectives = state->constraints_collectives;
        // NOTE(yoraish): we do not copy over the conflicts, since they will be recomputed in the new state. We could consider keeping a history of conflicts in the search state, with new conflicts being marked as such.

        // Update the constraints collective to also include the new constraint.
        new_state->constraints_collectives[agent_id].addConstraints(constraint_ptr);

        // Update the action-space. Start with the constraints and their context.
        std::shared_ptr<ConstraintsCollective> constraints_collective_ptr = std::make_shared<ConstraintsCollective>(new_state->constraints_collectives[agent_id]);
        std::shared_ptr<ConstraintsContext> context_ptr = std::make_shared<ConstraintsContext>();
        // context_ptr->agent_paths = new_state->paths;
        context_ptr->agent_names = agent_names_;
        context_ptr->action_space_ptr = agent_action_space_ptrs_[agent_id];
        constraints_collective_ptr->setContext(context_ptr);
        agent_action_space_ptrs_[agent_id]->setConstraintsCollective(constraints_collective_ptr);

        // Update the low-level planner for this agent.
        agent_planner_ptrs_[agent_id]->initializePlanner(agent_action_space_ptrs_[agent_id], starts_[agent_id], goals_[agent_id]);

        // Replan for this agent and update the stored path associated with it in the new state. Update the cost of the new state as well.
        new_state->paths[agent_id].clear();
        agent_planner_ptrs_[agent_id]->plan(new_state->paths[agent_id]);
        new_state->paths_transition_costs[agent_id] = agent_planner_ptrs_[agent_id]->stats_.transition_costs;
        new_state->paths_costs[agent_id] = agent_planner_ptrs_[agent_id]->stats_.cost;
        new_state->f = std::accumulate(new_state->paths_costs.begin(), new_state->paths_costs.end(), 0.0, [](double acc, const std::pair<int, double>& path_cost) { return acc + path_cost.second; });
        new_state->path_cost_lower_bounds[agent_id] = agent_planner_ptrs_[agent_id]->stats_.cost;
        new_state->sum_of_path_cost_lower_bounds = std::accumulate(new_state->path_cost_lower_bounds.begin(), new_state->path_cost_lower_bounds.end(), 0.0, [](double acc, const std::pair<int, double>& path_cost) { return acc + path_cost.second; });

        // Add the number of low level nodes to the counter.
        stats_.bonus_stats["num_low_level_expanded"] += agent_planner_ptrs_[agent_id]->stats_.num_expanded;

        // Add a random number between zero and one to f.
        // new_state->f += (double)rand() / RAND_MAX; // Uncomment for nitro boost.

        // If there is no path for this agent, then this is not a valid state. Discard it.
        if (new_state->paths[agent_id].empty()) {
            delete new_state;
            continue;
        }

        // The goal state returned is at time -1. We need to fix that.
        new_state->paths[agent_id].back().back() = new_state->paths[agent_id].size() - 1;

        // Push the new state to the open list.
        open_->push(new_state);
        new_state->setOpen();

        // Delete the previous state but keep the entry in the states_ vector.
        // state = nullptr;
    }
}