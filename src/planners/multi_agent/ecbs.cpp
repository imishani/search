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
 * \file   ecbs.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   07/08/22
 */

#include <search/planners/multi_agent/ecbs.hpp>

ims::ECBS::ECBS(const ims::ECBSParams& params) : params_(params), CBS(params) {
    // Create the open list.
    open_ = new FocalAndAnchorQueueWrapper<SearchState, ECBSOpenCompare, ECBSFocalCompare>();

    // Create a stats field for the low-level planner nodes created.
    stats_.bonus_stats["num_low_level_expanded"] = 0;
}

void ims::ECBS::initializePlanner(std::vector<std::shared_ptr<SubcostConstrainedActionSpace>>& action_space_ptrs,
                                 const std::vector<StateType>& starts, const std::vector<StateType>& goals) {

    // Create the open list. This list is created in the constructor and reset here.
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
        if (params_.low_level_heuristic_ptrs.size() != starts.size()) {
            throw std::runtime_error("Number of low-level heuristics (" + std::to_string(params_.low_level_heuristic_ptrs.size()) + ") does not match the number of agents (" + std::to_string(starts.size()) + ")");
        }
        ims::FocalwAStarParams focal_wastar_params_(params_.low_level_heuristic_ptrs[i], params_.low_level_focal_suboptimality ,params_.weight_low_level_heuristic);
        agent_planner_ptrs_.push_back(std::make_shared<ims::FocalwAStar>(focal_wastar_params_));
    }
}
                                 
void ims::ECBS::createRootInOpenList() {
    // Generate a plan for each of the agents.
    MultiAgentPaths initial_paths;
    std::unordered_map<int, double> initial_paths_costs;
    std::unordered_map<int, std::vector<double>> initial_paths_transition_costs;
    std::unordered_map<int, double> initial_paths_lower_bounds;
    for (size_t i{0}; i < num_agents_; ++i) {
        std::vector<StateType> path;
        agent_planner_ptrs_[i]->initializePlanner(agent_action_space_ptrs_[i], starts_[i], goals_[i]);
        bool is_plan_success = agent_planner_ptrs_[i]->plan(path);

        // Add the number of low level nodes to the counter.
        stats_.bonus_stats["num_low_level_expanded"] += agent_planner_ptrs_[i]->getStats().num_expanded;

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
        // initial_paths_costs.insert(std::make_pair(i, agent_planner_ptrs_[i]->stats_.cost));
        initial_paths_costs[i] = agent_planner_ptrs_[i]->getStats().cost;
        initial_paths_transition_costs[i] = agent_planner_ptrs_[i]->getStats().transition_costs;
        initial_paths_lower_bounds[i] = agent_planner_ptrs_[i]->getStats().lower_bound;
    }

    // Create the initial CBS state to the open list. This planner does not interface with an action space, so it does not call the getOrCreateRobotState to retrieve a new-state index. But rather decides on a new index directly and creates a search-state index with the getOrCreateSearchState method. Additionally, there is no goal specification for CBS, so we do not have a goal state.
    int start_ind_ = 0;
    auto start_ = getOrCreateSearchState(start_ind_);

    // Set the initial CBS state.
    start_->parent_id = PARENT_TYPE(START);
    start_->paths = initial_paths;
    start_->paths_costs = initial_paths_costs;
    start_->paths_transition_costs = initial_paths_transition_costs;

    // Get conflicts within the paths.
    // Get any conflicts between the newly computed paths.
    // NOTE(yoraish):  that this could be checked in any of the action_spaces, since they must all operate on the same scene. This is funky though, since the action_space is not aware of the other agents. Maybe this should be done in the ECBS class, and then passed to the action_space.
    agent_action_space_ptrs_[0]->getPathsConflicts(std::make_shared<MultiAgentPaths>(start_->paths), 
                                                    start_->unresolved_conflicts, 
                                                    getConflictTypes(),
                                                    -1, // TODO(yoraish): get all the conflicts.
                                                    agent_names_);

    // Set the cost of the CBSState start_.
    double start_soc = std::accumulate(initial_paths_costs.begin(), initial_paths_costs.end(), 0.0, [](double acc, const std::pair<int, double>& path_cost) { return acc + path_cost.second; });
    int start_num_conflicts = start_->unresolved_conflicts.size();
    start_->f = start_soc;
    start_->sum_of_costs = start_soc;
    start_->sum_of_path_cost_lower_bounds = std::accumulate(initial_paths_lower_bounds.begin(), initial_paths_lower_bounds.end(), 0.0, [](double acc, const std::pair<int, double>& path_cost) { return acc + path_cost.second; });
    start_->setOpen();

    // Push the initial CBS state to the open list.
    open_->push(start_);
    // Required to push into focal queue
    // double lower_bound = open_->getLowerBound();
    // open_->updateWithBound(params_.high_level_focal_suboptimality * lower_bound);

    // Show the initial paths.
    std::cout << "Initial paths:" << std::endl;
    for (auto& path : start_->paths) {
        std::cout << "Agent " << path.first << ": \n";
        for (auto state : path.second) {
            std::cout << "    [";
            for (auto val : state) {
                std::cout << val << ", ";
            }
            std::cout << "], \n";
        }
        std::cout << std::endl;
    }
}

void ims::ECBS::initializePlanner(std::vector<std::shared_ptr<SubcostConstrainedActionSpace>>& action_space_ptrs, const std::vector<std::string> & agent_names, const std::vector<StateType>& starts, const std::vector<StateType>& goals){
                        agent_names_ = agent_names;
                        initializePlanner(action_space_ptrs, starts, goals);
                        }

bool ims::ECBS::plan(MultiAgentPaths& paths) {
    startTimer();
    
    // Create the root node in the open list.
    createRootInOpenList();
    if (open_->empty()) {
        std::cout << "No path found." << std::endl;
        return false;
    }
    
    int iter{0};
    double lower_bound = open_->getLowerBound();
    open_->updateWithBound(params_.high_level_focal_suboptimality * lower_bound);

    while (!open_->empty() && !isTimeOut()) {
        // Report progress every 100 iterations
        if (iter % 1000 == 0) {
            std::cout << "ECBS CT open size: " << open_->size() << std::endl;
        }

        // Get the state of least cost.
        auto state = open_->min();
        open_->pop();

        // Set the state to closed.
        state->setClosed();

        // Expand the state. This requires a check for conflicts (done right below), a branch if there are conflicts (converted to constraints, done in expand()), and a replan for each branch in light of the new constraints (also done in expand()). If no conflicts were found, then the state is a goal state, is set in goals_, and we return.

        // Before we actually expand the state, we check if there is even a need to do so. If there are no conflicts, then this is a goal state. Set the goal state and return.
        if (state->unresolved_conflicts.empty()) {
            std::cout << "No conflicts found. This is a goal state." << std::endl;
            goal_ = state->state_id;
            getTimeFromStart(stats_.time);
            stats_.cost = state->f;
            paths = state->paths;
            stats_.num_expanded = iter;
            stats_.suboptimality = params_.high_level_focal_suboptimality;
            return true;
        }

        // Otherwise, expand the state.
        expand(state->state_id);
        ++iter;

        double lower_bound = open_->getLowerBound();
        open_->updateWithBound(params_.high_level_focal_suboptimality * lower_bound);
    }
    getTimeFromStart(stats_.time);
    return false;
}

void ims::ECBS::expand(int state_id) {
    auto state = getSearchState(state_id);
    std::vector<int> successors;
    std::vector<double> costs;

    // First, convert all conflicts to pairs of (agent_id, constraint). In vanilla ECBS, there is only one conflict found from a set of paths (the first/random one), and that would yield two constraints. To allow for more flexibility, we do not restrict the data structure to only two constraints per conflict.

    // Despite asking for many conflicts, we only convert the first one to constraints.
    std::vector<std::shared_ptr<Conflict>> conflicts_to_convert{state->unresolved_conflicts.begin(), state->unresolved_conflicts.begin() + 1};
    std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>> constraints = conflictsToConstraints(conflicts_to_convert);

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
        //LB-FIX  
        new_state->path_cost_lower_bounds = state->path_cost_lower_bounds;
        new_state->f = state->f;
        new_state->constraints_collectives = state->constraints_collectives;

        // Remove prior information for the agent that is being replanned-for. This is important for the constraints context, such that it only includes context from other agents.
        new_state->paths[agent_id].clear();
        new_state->paths_costs[agent_id] = 0.0;
        new_state->paths_transition_costs[agent_id].clear();

        // Update the constraints collective to also include the new constraint.
        new_state->constraints_collectives[agent_id].addConstraints(constraint_ptr);

        // Update the action-space with the constraints and their context (the paths of the other agents).
        std::shared_ptr<ConstraintsCollective> constraints_collective_ptr = std::make_shared<ConstraintsCollective>(new_state->constraints_collectives[agent_id]);
        std::shared_ptr<ConstraintsContext> context_ptr = std::make_shared<ConstraintsContext>();
        context_ptr->agent_paths = new_state->paths;
        context_ptr->agent_names = agent_names_;
        constraints_collective_ptr->setContext(context_ptr);
        agent_action_space_ptrs_[agent_id]->setConstraintsCollective(constraints_collective_ptr);

        // Update the low-level planner for this agent.
        agent_planner_ptrs_[agent_id]->initializePlanner(agent_action_space_ptrs_[agent_id], starts_[agent_id], goals_[agent_id]);

        // Replan for this agent and update the stored path associated with it in the new state. Update the cost of the new state as well.
        agent_planner_ptrs_[agent_id]->plan(new_state->paths[agent_id]);
        new_state->paths_transition_costs[agent_id] = agent_planner_ptrs_[agent_id]->getStats().transition_costs;
        new_state->paths_costs[agent_id] = agent_planner_ptrs_[agent_id]->getStats().cost;
        new_state->path_cost_lower_bounds[agent_id] = agent_planner_ptrs_[agent_id]->getStats().lower_bound;

        // Add the number of low level nodes to the counter.
        stats_.bonus_stats["num_low_level_expanded"] += agent_planner_ptrs_[agent_id]->getStats().num_expanded;

        // If there is no path for this agent, then this is not a valid state. Discard it.
        if (new_state->paths[agent_id].empty()) {
            delete new_state;
            continue;
        }

        // The goal state returned is at time -1. We need to fix that and set its time element (last value) to the size of the path.
        new_state->paths[agent_id].back().back() = new_state->paths[agent_id].size() - 1;
        // Get the sum of costs for the new state.
        double new_state_soc = std::accumulate(new_state->paths_costs.begin(), new_state->paths_costs.end(), 0.0, [](double acc, const std::pair<int, double>& path_cost) { return acc + path_cost.second; });
        double new_state_lb = std::accumulate(new_state->path_cost_lower_bounds.begin(), new_state->path_cost_lower_bounds.end(), 0.0, [](double acc, const std::pair<int, double>& path_cost) { return acc + path_cost.second; });

        // Get any conflicts between the newly computed paths.
        // NOTE(yoraish):  that this could be checked in any of the action_spaces, since they must all operate on the same scene. This is funky though, since the action_space is not aware of the other agents. Maybe this should be done in the ECBS class, and then passed to the action_space.
        agent_action_space_ptrs_[0]->getPathsConflicts(std::make_shared<MultiAgentPaths>(new_state->paths), 
                                                       new_state->unresolved_conflicts, 
                                                       getConflictTypes(),
                                                       -1, // TODO(yoraish): get all the conflicts.
                                                       agent_names_);

        new_state->f = new_state_soc;
        new_state->sum_of_costs = new_state_soc;
        new_state->sum_of_path_cost_lower_bounds = new_state_lb;
        // The goal state returned is at time -1. We need to fix that and set its time element (last value) to the size of the path.
        new_state->paths[agent_id].back().back() = new_state->paths[agent_id].size() - 1;

        // Push the new state to the open list.
        open_->push(new_state);
        new_state->setOpen();

        // Delete the previous state but keep the entry in the states_ vector.
        // state = nullptr;
    }
}


void ims::ECBS::verifyStartAndGoalInputStates(const std::vector<StateType>& starts, const std::vector<StateType>& goals) {
    // Check all goals have starts.
    if (starts.size() != goals.size()) {
        throw std::runtime_error("Start state vector size (" + std::to_string(starts.size()) + ") does not match the goal state vector size (" + std::to_string(goals.size()) + ")");
    }
    // Check if the start and goal states are valid w.r.t time. All starts are t=0 and all goals are t=-1.
    for (size_t i{0}; i < starts.size(); ++i) {
        if (starts[i].back() != 0) {
            throw std::runtime_error("Start state for agent " + std::to_string(i) + " is not at time 0");
        }
        if (goals[i].back() != -1) {
            throw std::runtime_error("Goal state for agent " + std::to_string(i) + " is not at time -1");
        }
    }

    // Check if the start and goal states are valid. For each agent.
    for (size_t i{0}; i < starts.size(); ++i) {
        if (!agent_action_space_ptrs_[i]->isStateValid(starts[i])) {
            throw std::runtime_error("Start state for agent " + std::to_string(i) + " is not valid");
        }
    }

    for (size_t i{0}; i < goals.size(); ++i) {
        if (!agent_action_space_ptrs_[i]->isStateValid(goals[i])) {
            throw std::runtime_error("Goal state for agent " + std::to_string(i) + " is not valid");
        }
    }
}