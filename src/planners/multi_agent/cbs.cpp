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
 * \file   cbs.cpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   07/07/23
 */

#include <search/planners/astar.hpp>
#include <search/planners/multi_agent/cbs.hpp>
#include <chrono>

// Global for cbs or not.
bool IS_CONFLICT_CREATION_CBS = true;

ims::CBSBase::CBSBase(const CBSParams& params): BestFirstSearch(params) {}

void ims::CBSBase::verifyStartAndGoalInputStates(const std::vector<StateType>& starts, const std::vector<StateType>& goals, const std::vector<std::shared_ptr<ims::ConstrainedActionSpace>>& action_space_ptrs) {
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
        if (!action_space_ptrs[i]->isStateValid(starts[i])) {
            throw std::runtime_error("Start state for agent " + std::to_string(i) + " is not valid");
        }
    }

    for (size_t i{0}; i < goals.size(); ++i) {
        if (!action_space_ptrs[i]->isStateValid(goals[i])) {
            throw std::runtime_error("Goal state for agent " + std::to_string(i) + " is not valid");
        }
    }
}

ims::CBS::CBS(const ims::CBSParams& params) : params_(params), CBSBase(params) {
    open_ = new SimpleQueue<SearchState, SearchStateCompare>();

    // Set the conflict types.
    conflict_types_ = getMapKeys(params_.conflict_type_to_constraint_types);
}

void ims::CBS::initializePlanner(std::vector<std::shared_ptr<ConstrainedActionSpace>>& action_space_ptrs,
                                 const std::vector<StateType>& starts, const std::vector<StateType>& goals) {
    // Reset the open list. Do this by deleteing it and creating it again. TODO(yoraish): add `clear` method to our custom queues.
    open_->clear();
    
    // Store the action spaces. This must happen before checking for the validity of the start and end states.
    agent_action_space_ptrs_ = action_space_ptrs;

    // Check if the inputs are valid.
    if (starts.size() != agent_action_space_ptrs_.size() || goals.size() != agent_action_space_ptrs_.size()) {
        throw std::runtime_error("Start state vector size (" + std::to_string(starts.size()) + ") does not match the number of agents (" + std::to_string(agent_action_space_ptrs_.size()) + ")");
    }

    verifyStartAndGoalInputStates(starts, goals, agent_action_space_ptrs_);

    // Store the starts and goals.
    starts_ = starts;
    goals_ = goals;

    // Set the number of agents.
    num_agents_ = (int)starts.size();

    // Create all the low-level planners.
    createLowLevelPlanners();
}

void ims::CBS::createRootInOpenList(){
    // Generate a plan for each of the agents.
    MultiAgentPaths initial_paths;
    std::unordered_map<int, double> initial_paths_costs;
    std::unordered_map<int, std::vector<double>> initial_paths_transition_costs;
    std::unordered_map<int, double> initial_paths_lower_bounds;
    double initial_sum_of_path_cost_lower_bounds{0.0};

    for (size_t i{0}; i < num_agents_; ++i) {
        std::vector<StateType> path;
        PlannerStats stats_low_level;
        bool is_plan_success = initializeAndPlanLowLevel((int)i, path, stats_low_level);

        // Add the number of low level nodes to the counter.
        stats_.bonus_stats["num_low_level_expanded"] += stats_low_level.num_expanded;

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
        initial_paths_costs[i] = stats_low_level.cost;
        initial_paths_transition_costs[i] = stats_low_level.transition_costs;

        // Compute the lower bound of the path.
        initial_paths_lower_bounds[i] = stats_low_level.cost;
        initial_sum_of_path_cost_lower_bounds += initial_paths_lower_bounds[i];
    }

    // Create the initial CBS state to the open list. This planner does not interface with an action space, so it does not call the getOrCreateRobotState to retrieve a new-state index. But rather decides on a new index directly and creates a search-state index with the getOrCreateSearchState method. Additionally, there is no goal specification for CBS, so we do not have a goal state.
    int start_ind_ = 0;
    auto start_ = getOrCreateSearchState(start_ind_);

    // Set the initial CBS state.
    start_->parent_id = PARENT_TYPE(START);
    start_->paths = std::make_shared<MultiAgentPaths>(initial_paths);
    start_->paths_costs = std::make_shared<std::unordered_map<int, double>>(initial_paths_costs);
    start_->path_cost_lower_bounds = std::make_shared<std::unordered_map<int, double>>(initial_paths_lower_bounds);
    start_->paths_transition_costs = std::make_shared<std::unordered_map<int, std::vector<double>>>(initial_paths_transition_costs);

    // Set the cost of the CBSState start_.
    start_->f = std::accumulate(initial_paths_costs.begin(), initial_paths_costs.end(), 0.0, [](double acc, const std::pair<int, double>& path_cost) { return acc + path_cost.second; });
    start_->setOpen();

    // >>> REMOVE REMOVE REMOVE
    if (!IS_CONFLICT_CREATION_CBS){
        // Check for conflicts in this new state.
        agent_action_space_ptrs_[0]->getPathsConflicts(start_->paths,
                                                        *start_->unresolved_conflicts,
                                                        getConflictTypes()   ,
                                                        1, 
                                                        agent_names_);
    }
    // <<< REMOVE REMOVE REMOVE

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

void ims::CBS::initializePlanner(std::vector<std::shared_ptr<ConstrainedActionSpace>>& action_space_ptrs, const std::vector<std::string> & agent_names, const std::vector<StateType>& starts, const std::vector<StateType>& goals){
                        agent_names_ = agent_names;
                        initializePlanner(action_space_ptrs, starts, goals);
                        }

auto ims::CBS::getSearchState(int state_id) -> ims::CBS::SearchState* {
    assert(state_id < states_.size() && state_id >= 0);
    return states_[state_id];
}

auto ims::CBS::getOrCreateSearchState(int state_id) -> ims::CBS::SearchState* {
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

bool ims::CBS::plan(MultiAgentPaths& paths) {
    startTimer();
    createRootInOpenList();
    int iter{0};
    while (!open_->empty() && !isTimeOut()) {
        // Report progress every 100 iterations
        if (iter % 100 == 0) {
            std::cout << "CBS CT open size: " << open_->size() << std::endl;
        }

        // Get the state of least cost.
        auto state = open_->min();
        open_->pop();

        // Set the state to closed.
        state->setClosed();

        // Expand the state. This requires a check for conflicts (done right below), a branch if there are conflicts (converted to constraints, done in expand()), and a replan for each branch in light of the new constraints (also done in expand()). If no conflicts were found, then the state is a goal state, is set in goals_, and we return.
        // NOTE(yoraish):  that this could be checked in any of the action_spaces, since they must all operate on the same scene. This is funky though, since the action_space is not aware of the other agents. Maybe this should be done in the CBS class, and then passed to the action_space.
        // >>> KEEP KEEP KEEP
        if (IS_CONFLICT_CREATION_CBS){
            static int get_paths_conflicts_counter = 0;
            static int sum_of_get_path_conflict_time = 0;
            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            agent_action_space_ptrs_[0]->getPathsConflicts(state->paths,
                                                        *state->unresolved_conflicts,
                                                        getConflictTypes()   ,
                                                        1,
                                                        agent_names_);
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            ++get_paths_conflicts_counter;
            sum_of_get_path_conflict_time += (int)std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
            // std::cout << "getPathsConflicts called " << get_paths_conflicts_counter << " times. Took " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << " microseconds, summing to " << sum_of_get_path_conflict_time/1000000.0 << " seconds" << std::endl;
        }
        // <<< KEEP KEEP KEEP

        // Before we actually expand the state, we check if there is even a need to do so. If there are no conflicts, then this is a goal state. Set the goal state and return.
        if (state->unresolved_conflicts->empty()) {
            std::cout << "No conflicts found. This is a goal state." << std::endl;
            goal_ = state->state_id;
            getTimeFromStart(stats_.time);
            stats_.cost = state->f;
            paths = *state->paths;
            stats_.num_expanded = iter;
            return true;
        }

        // Otherwise, expand the state.
        expand(state->state_id);
        ++iter;

    }
    getTimeFromStart(stats_.time);
    return false;
}

void ims::CBS::expand(int state_id) {
    auto state = getSearchState(state_id);
    std::vector<int> successors;
    std::vector<double> costs;

    // First, convert all conflicts to pairs of (agent_id, constraint). In vanilla CBS, there is only one conflict found from a set of paths (the first/random one), and that would yield two constraints. To allow for more flexibility, we do not restrict the data structure to only two constraints per conflict.
    std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>> constraints = conflictsToConstraints(*state->unresolved_conflicts);
    // Second, iterate through the constraints, and for each one, create a new search state. The new search state is a copy of the previous search state, with the constraint added to the constraints collective of the agent.
    // For each constraint, split the state into branches. Each branch will be a new state in the search tree.
    for (auto& agent_id_constraint : constraints){

        // The first element is the agent ID.
        int agent_id = agent_id_constraint.first;

        // The second element is a shared pointer to the constraint.
        auto constraint_ptr = agent_id_constraint.second;

        // Create a new search state. In this implementation ther is no check for whether the search state already exists (same starts, goals, and constraints), so we always create a new search state and push(...) it to the open list. Otherwise, we would check if the search state already exists, and if so, we would update(...) the open list heap.
        int new_state_id = (int)states_.size();
        auto new_state = getOrCreateSearchState(new_state_id);
        // The new state is a copy of the previous state.
        createSearchStateCopy(state, new_state);
        // NOTE(yoraish): we do not copy over the conflicts, since they will be recomputed in the new state. We could consider keeping a history of conflicts in the search state, with new conflicts being marked as such.

        new_state->paths->at(agent_id).clear();
        new_state->paths_costs->insert_or_assign(agent_id, 0.0);
        new_state->paths_transition_costs->at(agent_id).clear();

        // Update the constraints collective to also include the new constraint.
        new_state->constraints_collectives->at(agent_id).addConstraints(constraint_ptr);

        // Update the action-space. Start with the constraints and their context.
        std::shared_ptr<ConstraintsCollective> constraints_collective_ptr =
                std::make_shared<ConstraintsCollective>(new_state->constraints_collectives->at(agent_id));
        std::shared_ptr<ConstraintsContext> context_ptr = std::make_shared<ConstraintsContext>();
        // context_ptr->agent_paths = new_state->paths;
        context_ptr->action_space_ptr = agent_action_space_ptrs_[agent_id];
        context_ptr->agent_names = agent_names_;
        constraints_collective_ptr->setContext(context_ptr);
        agent_action_space_ptrs_[agent_id]->setConstraintsCollective(constraints_collective_ptr);

        // Replan for this agent and update the stored path associated with it in the new state. Update the cost of the new state as well.
        PlannerStats stats_low_level;
        initializeAndPlanLowLevel(agent_id, new_state->paths->at(agent_id), stats_low_level);
        new_state->paths_transition_costs->at(agent_id) = stats_low_level.transition_costs;
        new_state->paths_costs->at(agent_id) = stats_low_level.cost;
        new_state->f = std::accumulate(new_state->paths_costs->begin(), new_state->paths_costs->end(), 0.0, [](double acc, const std::pair<int, double>& path_cost) { return acc + path_cost.second; });
        new_state->path_cost_lower_bounds->at(agent_id) = stats_low_level.cost;
        new_state->sum_of_path_cost_lower_bounds = std::accumulate(new_state->path_cost_lower_bounds->begin(), new_state->path_cost_lower_bounds->end(), 0.0, [](double acc, const std::pair<int, double>& path_cost) { return acc + path_cost.second; });

        // Add the number of low level nodes to the counter.
        stats_.bonus_stats["num_low_level_expanded"] += stats_low_level.num_expanded;

        // Add a random number between zero and one to f.
        // new_state->f += (double)rand() / RAND_MAX; // Uncomment for nitro boost.

        // If there is no path for this agent, then this is not a valid state. Discard it.
        if (new_state->paths->at(agent_id).empty()) {
//            std::cout << RED << "No path found for agent " << agent_id << " in the expansion phase." << RESET << std::endl;
            delete new_state;
            continue;
        }

        // The goal state returned is at time -1. We need to fix that.
        new_state->paths->at(agent_id).back().back() = new_state->paths->at(agent_id).size() - 1;

        // >>> REMOVE REMOVE REMOVE
        if (!IS_CONFLICT_CREATION_CBS){
            // Check for conflicts in this new state.
            static int get_paths_conflicts_counter = 0;
            static int sum_of_get_path_conflict_time = 0;
            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            get_paths_conflicts_counter++;
            agent_action_space_ptrs_[0]->getPathsConflicts(new_state->paths,
                                                           *new_state->unresolved_conflicts,
                                                           getConflictTypes()   ,
                                                           1,
                                                           agent_names_);
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            ++get_paths_conflicts_counter;
            sum_of_get_path_conflict_time += (int)std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
            // std::cout << "getPathsConflicts called " << get_paths_conflicts_counter << " times. Took " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << " microseconds, summing to " << sum_of_get_path_conflict_time/1000000.0 << " seconds" << std::endl;
        }
        // <<< REMOVE REMOVE REMOVE

        // Push the new state to the open list.
        open_->push(new_state);
        new_state->setOpen();
        stats_.num_generated++;

        // Delete the previous state but keep the entry in the states_ vector.
        // state = nullptr;
    }
}

void ims::CBS::setStateVals(int state_id, int parent_id, double cost) {
    auto state = getSearchState(state_id);
    auto parent = getSearchState(parent_id);
    state->parent_id = parent_id;
}

std::vector<std::pair<int, std::vector<std::shared_ptr<ims::Constraint>>>> ims::CBS::conflictsToConstraints(const std::vector<std::shared_ptr<ims::Conflict>>& conflicts) {
    std::vector<std::pair<int, std::vector<std::shared_ptr<ims::Constraint>>>> agent_constraints;
    // Create a converter object.
    ims::conflict_conversions::ConflictsToConstraintsConverter conflict_converter;
    // Add some context.
    ims::conflict_conversions::ConflictConversionContext context;
    context.agent_names = agent_names_;
    conflict_converter.setContext(context);

    // Iterate through the conflicts and convert them to constraints.
    for (auto& conflict_ptr : conflicts) {
        // Create a new constraint given the conflict. Check through the conflict types and convert them to constraints.
        for (ConstraintType constraint_type : params_.conflict_type_to_constraint_types.at(conflict_ptr->type)){
            conflict_converter.convertConflictToConstraints(conflict_ptr, constraint_type, agent_constraints);
        }
    }

    return agent_constraints;
}

void ims::CBS::createLowLevelPlanners(){
    assert(!agent_action_space_ptrs_.empty());
    for (size_t i{0}; i < agent_action_space_ptrs_.size(); ++i) {
        ims::wAStarParams wastar_params_(params_.low_level_heuristic_ptrs[i], params_.weight_low_level_heuristic);
        agent_planner_ptrs_.push_back(std::make_shared<ims::wAStar>(wastar_params_));
    }
}

bool ims::CBS::initializeAndPlanLowLevel(int agent_id, std::vector<StateType>& path, PlannerStats& stats){
    // Initialize the low-level planner.
    agent_planner_ptrs_[agent_id]->initializePlanner(agent_action_space_ptrs_[agent_id],
                                                     starts_[agent_id],
                                                     goals_[agent_id]);

    // Plan with the low-level planner.
    bool is_plan_success = agent_planner_ptrs_[agent_id]->plan(path);

    // Update the stats.
    stats = agent_planner_ptrs_[agent_id]->getStats();

    return is_plan_success;
}

void ims::CBS::createSearchStateCopy(const ims::CBSBase::SearchState *state, ims::CBSBase::SearchState *new_state)  {
    new_state->parent_id = state->state_id;
    new_state->paths = std::make_shared<MultiAgentPaths>(*state->paths);
    new_state->paths_costs = std::make_shared<std::unordered_map<int, double>>(*state->paths_costs);
    new_state->paths_transition_costs = std::make_shared<std::unordered_map<int, std::vector<double>>>(*state->paths_transition_costs);
    new_state->f = state->f;
    new_state->path_cost_lower_bounds = std::make_shared<std::unordered_map<int, double>>(*state->path_cost_lower_bounds);
    new_state->constraints_collectives = std::make_shared<std::unordered_map<int, ConstraintsCollective>>(*state->constraints_collectives);
}
