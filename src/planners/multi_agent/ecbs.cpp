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

    // Check start and end states for validity. Individually and between agents. Up-cast to use the method.
    std::vector<std::shared_ptr<ims::ConstrainedActionSpace>> constrained_action_space_ptrs(agent_action_space_ptrs_.begin(), agent_action_space_ptrs_.end());
    verifyStartAndGoalInputStates(starts, goals, constrained_action_space_ptrs);

    // Store the starts and goals.
    starts_ = starts;
    goals_ = goals;

    // Set the number of agents.
    num_agents_ = (int)starts.size();

    // Create all the low-level planners.
    createLowLevelPlanners();
}
                                 
void ims::ECBS::createRootInOpenList() {
    // Generate a plan for each of the agents.
    MultiAgentPaths initial_paths;
    std::unordered_map<int, double> initial_paths_costs;
    std::unordered_map<int, std::vector<double>> initial_paths_transition_costs;
    std::unordered_map<int, double> initial_paths_lower_bounds;
    for (size_t i{0}; i < num_agents_; ++i) {
        if (params_.is_root_trick) {
            // Root Trick.
            // Add the previous paths as context to the action space.
            std::shared_ptr<ConstraintsContext> context_ptr = std::make_shared<ConstraintsContext>();
            context_ptr->agent_paths = initial_paths;
            context_ptr->agent_names = agent_names_;
            agent_action_space_ptrs_[i]->constraints_collective_ptr_->setContext(context_ptr);
        }
        std::vector<StateType> path;
        FocalSearchPlannerStats low_level_plan_stats;
        bool is_plan_success = initializeAndPlanLowLevel((int)i, path, low_level_plan_stats);
        // Add the number of low level nodes to the counter.
        stats_.bonus_stats["num_low_level_expanded"] += low_level_plan_stats.num_expanded;
        // If there is no path for this agent, then this is not a valid state. Do not add a new state to the open list.
        if (!is_plan_success) {
            std::cout << RED << "No path found for agent " << i << " in the initial planning phase." << RESET << std::endl;
            stats_.cost = -1;
            stats_.time = -1;
            return;
        }
        // Fix the last path state to have a correct time and not -1.
        path.back().back() = (int)path.size() - 1;

        // We use a map since down the line we may only store paths for some agents.
        // initial_paths.insert(std::make_pair(i, path));
        initial_paths[i] = path;

        // Compute the cost of the path.
        // initial_paths_costs.insert(std::make_pair(i, agent_planner_ptrs_[i]->stats_.cost));
        initial_paths_costs[i] = low_level_plan_stats.cost;
        initial_paths_transition_costs[i] = low_level_plan_stats.transition_costs;
        initial_paths_lower_bounds[i] = low_level_plan_stats.lower_bound;
    }

    // Create the initial CBS state to the open list. This planner does not interface with an action space, so it does not call the getOrCreateRobotState to retrieve a new-state index. But rather decides on a new index directly and creates a search-state index with the getOrCreateSearchState method. Additionally, there is no goal specification for CBS, so we do not have a goal state.
    int start_ind_ = 0;
    auto start_ = getOrCreateSearchState(start_ind_);

    // Set the initial CBS state.
    start_->parent_id = PARENT_TYPE(START);
    start_->paths = std::make_shared<MultiAgentPaths>(initial_paths);
    start_->paths_costs = std::make_shared<std::unordered_map<int, double>>(initial_paths_costs);
    start_->paths_transition_costs = std::make_shared<std::unordered_map<int, std::vector<double>>>(initial_paths_transition_costs);

    // Get conflicts within the paths.
    // Get any conflicts between the newly computed paths.
    // NOTE(yoraish):  that this could be checked in any of the action_spaces, since they must all operate on the same scene. This is funky though, since the action_space is not aware of the other agents. Maybe this should be done in the ECBS class, and then passed to the action_space.
    agent_action_space_ptrs_[0]->getPathsConflicts(start_->paths,
                                                    *start_->unresolved_conflicts,
                                                    getConflictTypes(),
                                                    -1, // TODO(yoraish): get all the conflicts.
                                                    agent_names_);

    // Set the cost of the CBSState start_.
    double start_soc = std::accumulate(initial_paths_costs.begin(), initial_paths_costs.end(), 0.0, [](double acc, const std::pair<int, double>& path_cost) { return acc + path_cost.second; });
    int start_num_conflicts = start_->unresolved_conflicts->size();
    start_->f = start_soc;
    start_->sum_of_costs = start_soc;
    start_->sum_of_path_cost_lower_bounds = std::accumulate(initial_paths_lower_bounds.begin(), initial_paths_lower_bounds.end(), 0.0, [](double acc, const std::pair<int, double>& path_cost) { return acc + path_cost.second; });
    start_->setOpen();

    // Add the agent_names to the constraints collectives. And add constraints collectives to the search state.
    for (int agent_id = 0; agent_id < num_agents_; agent_id++){
        start_->constraints_collectives->emplace(agent_id, agent_action_space_ptrs_[agent_id]->getConstraintsCollective());
        start_->constraints_collectives->at(agent_id).getConstraintsContextNonConst()->agent_names = agent_names_;
        // Add a pointer to the action space to the constraints collectives.
        start_->constraints_collectives->at(agent_id).getConstraintsContextNonConst()->action_space_ptr = agent_action_space_ptrs_[agent_id];
        std::cout << "Parent state has " << start_->constraints_collectives->at(agent_id).getNumSafeIntervals() << " safe intervals." << std::endl;
    }


    // Push the initial CBS state to the open list.
    open_->push(start_);

    // Show the initial paths.
    std::cout << "Initial paths:" << std::endl;
    for (auto& path : *start_->paths) {
        std::cout << "Agent " << path.first << ": \n";
        for (const auto& state : path.second) {
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
        if (state->unresolved_conflicts->empty()) {
            std::cout << "No conflicts found. This is a goal state." << std::endl;
            goal_ = state->state_id;
            getTimeFromStart(stats_.time);
            stats_.cost = state->f;
            paths = *state->paths;
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
    // Keep track of the newly created child nodes. Those will be added to OPEN later.
    std::vector<SearchState*> child_search_states;

    auto state = getSearchState(state_id);
    std::vector<int> successors;
    std::vector<double> costs;

    if (params_.verbose){
        std::cout << YELLOW << "====================" << RESET << std::endl;
        std::cout << "Expanding state " << state_id << std::endl;
        std::cout << "====================" << std::endl;
    }

    // First, convert all conflicts to pairs of (agent_id, constraint). In vanilla ECBS, there is only one conflict found from a set of paths (the first/random one), and that would yield two constraints. To allow for more flexibility, we do not restrict the data structure to only two constraints per conflict.

    // Despite asking for many conflicts, we only convert the first one to constraints.
    std::vector<std::shared_ptr<Conflict>> conflicts_to_convert{state->unresolved_conflicts->begin(), state->unresolved_conflicts->begin() + 1};
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
        createSearchStateCopy(state, new_state);

        // Remove prior information for the agent that is being replanned-for. This is important for the constraints context, such that it only includes context from other agents.
        new_state->paths->at(agent_id).clear();
        new_state->paths_costs->insert_or_assign(agent_id, 0.0);
        new_state->paths_transition_costs->at(agent_id).clear();

        // Update the constraints collective to also include the new constraint.
        new_state->constraints_collectives->at(agent_id).addConstraints(constraint_ptr);

        // Update the action-space with the constraints and their context (the paths of the other agents).
        std::shared_ptr<ConstraintsCollective> constraints_collective_ptr = std::make_shared<ConstraintsCollective>(new_state->constraints_collectives->at(agent_id));
        std::shared_ptr<ConstraintsContext> context_ptr = std::make_shared<ConstraintsContext>();
        context_ptr->agent_paths = *new_state->paths;
        context_ptr->agent_names = agent_names_;
        context_ptr->action_space_ptr = agent_action_space_ptrs_[agent_id];
        constraints_collective_ptr->setContext(context_ptr);
        agent_action_space_ptrs_[agent_id]->setConstraintsCollective(constraints_collective_ptr);

        // Replan for this agent.
        FocalSearchPlannerStats low_level_plan_stats;
        bool is_replan_success = initializeAndPlanLowLevel(agent_id, new_state->paths->at(agent_id), low_level_plan_stats);
        if (!is_replan_success){
            std::cout << RED << "No path found for agent " << agent_id << " in the replanning phase." << RESET << std::endl;
            delete new_state;
            continue;
        }
        // Update the stats.
        new_state->paths_transition_costs->insert_or_assign(agent_id, low_level_plan_stats.transition_costs);
        new_state->paths_costs->insert_or_assign(agent_id, low_level_plan_stats.cost);
        new_state->path_cost_lower_bounds->insert_or_assign(agent_id, low_level_plan_stats.lower_bound);

        // Add the number of low level nodes to the counter.
        stats_.bonus_stats["num_low_level_expanded"] += low_level_plan_stats.num_expanded;

        // If there is no path for this agent, then this is not a valid state. Discard it.
        if (new_state->paths->at(agent_id).empty()) {
            delete new_state;
            continue;
        }

        // The goal state returned is at time -1. We need to fix that and set its time element (last value) to the size of the path.
        new_state->paths->at(agent_id).back().back() = new_state->paths->at(agent_id).size() - 1;
        // Get the sum of costs for the new state.
        double new_state_soc = std::accumulate(new_state->paths_costs->begin(), new_state->paths_costs->end(), 0.0, [](double acc, const std::pair<int, double>& path_cost) { return acc + path_cost.second; });
        double new_state_lb = std::accumulate(new_state->path_cost_lower_bounds->begin(), new_state->path_cost_lower_bounds->end(), 0.0, [](double acc, const std::pair<int, double>& path_cost) { return acc + path_cost.second; });

        // Get any conflicts between the newly computed paths.
        // NOTE(yoraish):  that this could be checked in any of the action_spaces, since they must all operate on the same scene. This is funky though, since the action_space is not aware of the other agents. Maybe this should be done in the ECBS class, and then passed to the action_space.
        agent_action_space_ptrs_[0]->getPathsConflicts(new_state->paths,
                                                       *new_state->unresolved_conflicts,
                                                       getConflictTypes(),
                                                       -1, // TODO(yoraish): get all the conflicts.
                                                       agent_names_);
        double prev_soc = state->sum_of_costs;
        int prev_num_conflicts = (int)state->unresolved_conflicts->size();
        int new_state_num_conflicts = (int)new_state->unresolved_conflicts->size();
        std::cout << "-(SOC, Conflicts): (" << prev_soc << ", " << prev_num_conflicts << ") -> " << "(" << new_state_soc << ", " << new_state_num_conflicts << ")" << std::endl;

        new_state->f = new_state_soc;
        new_state->sum_of_costs = new_state_soc;
        new_state->sum_of_path_cost_lower_bounds = new_state_lb;
        // The goal state returned is at time -1. We need to fix that and set its time element (last value) to the size of the path.
        new_state->paths->at(agent_id).back().back() = new_state->paths->at(agent_id).size() - 1;

        // Push the new state to the child search states.
        child_search_states.push_back(new_state);
        stats_.num_generated++;

        // If we are allowed to bypass conflicts, then check if the new state is
        // 1. Of equal or lower cost, and
        // 2. Has fewer conflicts.
        // If so, then we discard all other child states created and only push this one with the parent's constraints.
        if (params_.is_bypassing_conflicts) {
            if (new_state_soc <= prev_soc && new_state_num_conflicts < prev_num_conflicts) {
                std::cout << CYAN << "Bypassing conflicts." << RESET << std::endl;
                // Discard all other child states. These are all but the last one.
                for (size_t i{0}; i < child_search_states.size() - 1; ++i) {
                    delete child_search_states[i];
                }
                // Clear the child search states vector and push the last one.
                child_search_states.clear();
                // Set the parent's constraints to the new state's constraints.
                new_state->constraints_collectives = state->constraints_collectives;
                child_search_states.push_back(new_state);
                break;
            }
        }
    }
    // Push the child search states to the open list.
    for (auto& child_search_state : child_search_states) {
        open_->push(child_search_state);
        child_search_state->setOpen();
    }
}

void ims::ECBS::createLowLevelPlanners() {
    int num_agents = (int)agent_action_space_ptrs_.size();
    assert(num_agents != 0);
    for (size_t i{0}; i < num_agents; ++i) {
        if (params_.low_level_heuristic_ptrs.size() != num_agents) {
            throw std::runtime_error("Number of low-level heuristics (" + std::to_string(params_.low_level_heuristic_ptrs.size()) + ") does not match the number of agents (" + std::to_string(num_agents) + ")");
        }

        // Set up the weighted A* planners.
        ims::FocalwAStarParams focal_wastar_params_(params_.low_level_heuristic_ptrs[i],
                                                    params_.low_level_focal_suboptimality,
                                                    params_.weight_low_level_heuristic);
        focal_wastar_params_.time_limit_ = params_.time_limit_;
        agent_planner_ptrs_.push_back(std::make_shared<ims::FocalwAStar>(focal_wastar_params_));
    }
}

bool ims::ECBS::initializeAndPlanLowLevel(int agent_id, PathType &path,
                                          ims::FocalSearchPlannerStats &stats) {
    agent_planner_ptrs_.at(agent_id)->initializePlanner(agent_action_space_ptrs_.at(agent_id),
                                                 starts_.at(agent_id),
                                                 goals_.at(agent_id));
    bool is_plan_success = agent_planner_ptrs_[agent_id]->plan(path);
    stats = agent_planner_ptrs_[agent_id]->getStats();
    return is_plan_success;
}

// ==============================
// ECBSSIPP.
// ==============================

void ims::ECBSSIPP::createLowLevelPlanners() {
    int num_agents = (int)agent_action_space_ptrs_.size();
    assert(num_agents != 0);
    for (size_t i{0}; i < num_agents; ++i) {
        if (params_.low_level_heuristic_ptrs.size() != num_agents) {
            throw std::runtime_error("Number of low-level heuristics (" + std::to_string(params_.low_level_heuristic_ptrs.size()) + ") does not match the number of agents (" + std::to_string(num_agents) + ")");
        }
        ims::FocalwSIPPParams focal_wsipp_params_(params_.low_level_heuristic_ptrs[i],
                                                  params_.low_level_focal_suboptimality,
                                                  params_.weight_low_level_heuristic);
        focal_wsipp_params_.verbose = params_.verbose;
        focal_wsipp_params_.time_limit_ = params_.time_limit_;
        agent_planner_ptrs_.push_back(std::make_shared<ims::FocalwSIPP>(focal_wsipp_params_));
    }
}

bool ims::ECBSSIPP::initializeAndPlanLowLevel(int agent_id, PathType &path,
                                              ims::FocalSearchPlannerStats &stats) {
    agent_planner_ptrs_.at(agent_id)->initializePlanner(agent_action_space_ptrs_.at(agent_id),
                                                 starts_.at(agent_id),
                                                 goals_.at(agent_id));
    bool is_plan_success = agent_planner_ptrs_[agent_id]->plan(path);
    stats = agent_planner_ptrs_[agent_id]->getStats();
    return is_plan_success;
}