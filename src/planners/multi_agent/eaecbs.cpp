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
 * \file   eaecbs.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   11/09/2023
 */

#include <search/planners/wastar.hpp>
#include <search/planners/multi_agent/eaecbs.hpp>
#include <search/planners/multi_agent/eacbs.hpp>

ims::EAECBS::EAECBS(const ims::EAECBSParams& params) : params_(params), ECBS(params) {
    open_ = new FocalAndAnchorQueueWrapper<SearchState, EACBSOpenCompare, EACBSFocalCompare>();
}


void ims::EAECBS::initializePlanner(std::vector<std::shared_ptr<SubcostConstrainedActionSpace>>& action_space_ptrs,
                        const std::vector<StateType>& starts, const std::vector<StateType>& goals){
    
    // Cast the action spaces to SubcostExperienceAcceleratedConstrainedActionSpace.
    std::vector<std::shared_ptr<SubcostExperienceAcceleratedConstrainedActionSpace>> eac_action_space_ptrs;
    for (auto& action_space_ptr : action_space_ptrs) {
        eac_action_space_ptrs.push_back(std::dynamic_pointer_cast<SubcostExperienceAcceleratedConstrainedActionSpace>(action_space_ptr));
    }

    // Check that all casts succeeded.
    for (auto& action_space_ptr : eac_action_space_ptrs) {
        if (action_space_ptr == nullptr) {
            throw std::runtime_error("Could not cast action space to SubcostExperienceAcceleratedConstrainedActionSpace");
        }
    }

    // Call the other initializePlanner method.
    initializePlanner(eac_action_space_ptrs, starts, goals);
}

void ims::EAECBS::initializePlanner(std::vector<std::shared_ptr<SubcostConstrainedActionSpace>>& action_space_ptrs, const std::vector<std::string>& agent_names, const std::vector<StateType>& starts, const std::vector<StateType>& goals){
    // Cast the action spaces to SubcostExperienceAcceleratedConstrainedActionSpace.
    std::vector<std::shared_ptr<SubcostExperienceAcceleratedConstrainedActionSpace>> eac_action_space_ptrs;
    for (auto& action_space_ptr : action_space_ptrs) {
        eac_action_space_ptrs.push_back(std::dynamic_pointer_cast<SubcostExperienceAcceleratedConstrainedActionSpace>(action_space_ptr));
    }

    // Check that all casts succeeded.
    for (auto& action_space_ptr : eac_action_space_ptrs) {
        if (action_space_ptr == nullptr) {
            throw std::runtime_error("Could not cast action space to SubcostExperienceAcceleratedConstrainedActionSpace");
        }
    }

    // Call the other initializePlanner method.
    initializePlanner(eac_action_space_ptrs, agent_names, starts, goals);
}

void ims::EAECBS::initializePlanner(std::vector<std::shared_ptr<SubcostExperienceAcceleratedConstrainedActionSpace>>& action_space_ptrs, const std::vector<std::string> & agent_names, const std::vector<StateType>& starts, const std::vector<StateType>& goals){
                        agent_names_ = agent_names;
                        EAECBS::initializePlanner(action_space_ptrs, starts, goals);
                        }


void ims::EAECBS::initializePlanner(std::vector<std::shared_ptr<SubcostExperienceAcceleratedConstrainedActionSpace>>& action_space_ptrs,
                                 const std::vector<StateType>& starts, const std::vector<StateType>& goals) {
    // Store the action spaces. This must happen before checking for the validity of the start and end states.
    agent_action_space_ptrs_ = action_space_ptrs;

    // Check if the inputs are valid.
    if (starts.size() != agent_action_space_ptrs_.size() || goals.size() != agent_action_space_ptrs_.size()) {
        throw std::runtime_error("Start state vector size (" + std::to_string(starts.size()) + ") does not match the number of agents (" + std::to_string(agent_action_space_ptrs_.size()) + ")");
    }

    // Check start and end states for validity. Individually and between agents.
    std::vector<std::shared_ptr<ims::ConstrainedActionSpace>> constrained_action_space_ptrs(agent_action_space_ptrs_.begin(), agent_action_space_ptrs_.end());
    verifyStartAndGoalInputStates(starts, goals, constrained_action_space_ptrs);

    // Store the starts and goals.
    starts_ = starts;
    goals_ = goals;

    // Set the number of agents.
    num_agents_ = (int)starts.size();

    // Create all the low-level planners.
    for (size_t i{0}; i < starts.size(); ++i) {
        ims::FocalEAwAStarUniformCostParams eawastar_params_(params_.low_level_heuristic_ptrs[i], params_.low_level_focal_suboptimality, params_.weight_low_level_heuristic);
        
        agent_planner_ptrs_.push_back(std::make_shared<ims::FocalEAwAStarUniformCost>(eawastar_params_));
    }
}

void ims::EAECBS::createRootInOpenList() {
    // Generate a plan for each of the agents.
    MultiAgentPaths initial_paths;
    std::unordered_map<int, double> initial_paths_costs;
    std::unordered_map<int, std::vector<double>> initial_paths_transition_costs;
    std::unordered_map<int, double> initial_paths_lower_bounds;
    //LB-FIX 
    double initial_sum_of_path_cost_lower_bounds{0.0};

    for (size_t i{0}; i < num_agents_; ++i) {
        // Root trick.
        // Add the previous paths as context to the action space.
//        std::shared_ptr<ConstraintsContext> context_ptr = std::make_shared<ConstraintsContext>();
//        context_ptr->agent_paths = initial_paths;
//        context_ptr->agent_names = agent_names_;
//        agent_action_space_ptrs_[i]->constraints_collective_ptr_->setContext(context_ptr);

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
        initial_paths_costs[i] = agent_planner_ptrs_[i]->getStats().cost;
        initial_paths_transition_costs[i] = agent_planner_ptrs_[i]->getStats().transition_costs;
        initial_paths_lower_bounds[i] = agent_planner_ptrs_[i]->getStats().lower_bound;
        //LB-FIX 
        initial_sum_of_path_cost_lower_bounds += initial_paths_lower_bounds[i];
    }

    // Report that the initial paths were found.
    std::cout << "Initial paths found." << std::endl;


    // Create the initial EACBS state to the open list. This planner does not interface with an action space, so it does not call the getOrCreateRobotState to retrieve a new-state index. But rather decides on a new index directly and creates a search-state index with the getOrCreateSearchState method. Additionally, there is no goal specification for EACBS, so we do not have a goal state.
    int start_ind_ = 0;
    auto start_ = getOrCreateSearchState(start_ind_);

    // Set the initial EACBS state.
    start_->parent_id = PARENT_TYPE(START);
    start_->paths = initial_paths;
    start_->paths_costs = initial_paths_costs;
    //LB-FIX 
    start_->path_cost_lower_bounds = initial_paths_lower_bounds;
    //LB-FIX 
    start_->sum_of_path_cost_lower_bounds = initial_sum_of_path_cost_lower_bounds;
    start_->paths_transition_costs = initial_paths_transition_costs;

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

    // Push the initial EACBS state to the open list.
    open_->push(start_);
}

bool ims::EAECBS::plan(MultiAgentPaths& paths) {
    startTimer();
    int iter{0};
    createRootInOpenList();
    if (open_->empty()) {
        std::cout << "No path found." << std::endl;
        return false;
    }
    
    double lower_bound = open_->getLowerBound();
    open_->updateWithBound(params_.high_level_focal_suboptimality * lower_bound);

    while (!open_->empty() && !isTimeOut()) {
        // Report progress every 100 iterations
        if (iter % 1000 == 0) {
            std::cout << "EAECBS CT open size: " << open_->size() << std::endl;
        }

        // Get the state of least cost.
        auto state = open_->min();
        open_->pop();

        // Set the state to closed.
        state->setClosed();

        std::cout << "Expanded CT node with soc " << state->sum_of_costs << " and " << state->unresolved_conflicts.size() << " conflicts." << std::endl;

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
        std::cout << "* FOCAL accepting states with f value at most " << params_.high_level_focal_suboptimality * lower_bound << std::endl;
    }
    getTimeFromStart(stats_.time);
    return false;
}

auto ims::EAECBS::getSearchState(int state_id) -> ims::EAECBS::SearchState* {
    assert(state_id < states_.size() && state_id >= 0);
    return states_[state_id];
}

auto ims::EAECBS::getOrCreateSearchState(int state_id) -> ims::EAECBS::SearchState* {
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

void ims::EAECBS::expand(int state_id) {
    auto state = getSearchState(state_id);
    std::vector<int> successors;
    std::vector<double> costs;

    // First, convert all conflicts to pairs of (agent_id, constraint). In vanilla ECBS, there is only one conflict found from a set of paths (the first/random one), and that would yield two constraints. To allow for more flexibility, we do not restrict the data structure to only two constraints per conflict.

    // Despite asking for many conflicts, we only convert the first one to constraints.
    std::vector<std::shared_ptr<Conflict>> conflicts_to_convert{state->unresolved_conflicts.begin(), state->unresolved_conflicts.begin() + 1};

    // std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>> constraints = conflictsToConstraints(state->conflicts);
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
        new_state->f = state->f;
        new_state->sum_of_costs = state->sum_of_costs;
        new_state->path_cost_lower_bounds = state->path_cost_lower_bounds; 
        new_state->constraints_collectives = state->constraints_collectives;
        new_state->experiences_collectives = state->experiences_collectives;
        // NOTE(yoraish): we do not copy over the conflicts, since they will be recomputed in the new state. We could consider keeping a history of conflicts in the search state, with new conflicts being marked as such.

        // Update the constraints collective to also include the new constraint.
        new_state->constraints_collectives[agent_id].addConstraints(constraint_ptr);
        
        // Update the action-space. Start with the constraints and their context.
        std::shared_ptr<ConstraintsCollective> constraints_collective_ptr = std::make_shared<ConstraintsCollective>(new_state->constraints_collectives[agent_id]);
        std::shared_ptr<ConstraintsContext> context_ptr = std::make_shared<ConstraintsContext>();
        context_ptr->agent_paths = new_state->paths;
        context_ptr->agent_names = agent_names_;
        constraints_collective_ptr->setContext(context_ptr);
        agent_action_space_ptrs_[agent_id]->setConstraintsCollective(constraints_collective_ptr);

        /////////////////////////////////////////////
        // Update the experiences collective.
        /////////////////////////////////////////////
        switch (experience_reuse_type_) {
            case ExperienceReuseType::NONE: {
                // std::cout << "Experience reuse type is NONE. Not updating the experiences collective." << std::endl;
                break;
            }
            case ExperienceReuseType::PREVIOUS_SOLUTION: {
                // std::cout << "Experience reuse type is PREVIOUS_SOLUTION. Updating the experiences collective with the previous solution." << std::endl;
                agent_action_space_ptrs_[agent_id]->clearPathExperiences();
                agent_action_space_ptrs_[agent_id]->addTimedPathExperienceToExperiencesCollective(
                        std::make_shared<PathExperience>(state->paths[agent_id],
                                                         state->paths_transition_costs[agent_id]));
                break;
            }
            case ExperienceReuseType::CT_BRANCH: {
                // std::cout << "Experience reuse type is CT_BRANCH. Updating the experiences collective with the solution on branch." << std::endl;
                new_state->experiences_collectives[agent_id].addTimedPathExperience(std::make_shared<PathExperience>(state->paths[agent_id], state->paths_transition_costs[agent_id]));

                // Update the action-space with the updated experiences.
                agent_action_space_ptrs_[agent_id]->setExperiencesCollective(std::make_shared<ExperiencesCollective>(new_state->experiences_collectives[agent_id]));
                break;
            }
            case ExperienceReuseType::CT_GLOBAL: {
                // std::cout << "Experience reuse type is CT_GLOBAL. Updating the experiences collective with all previous solutions." << std::endl;
                agent_action_space_ptrs_[agent_id]->addTimedPathExperienceToExperiencesCollective(std::make_shared<PathExperience>(state->paths[agent_id], state->paths_transition_costs[agent_id]));
                break;
            }
            default: {
                throw std::runtime_error("Unknown experience reuse type.");
            }
        }

        // Update the low-level planner for this agent.
        agent_planner_ptrs_[agent_id]->initializePlanner(agent_action_space_ptrs_[agent_id], starts_[agent_id], goals_[agent_id]);

        // Replan for this agent and update the stored path associated with it in the new state. Update the cost of the new state as well.
        new_state->paths[agent_id].clear();
        agent_planner_ptrs_[agent_id]->plan(new_state->paths[agent_id]);
        new_state->paths_transition_costs[agent_id] = agent_planner_ptrs_[agent_id]->getStats().transition_costs;
        new_state->paths_costs[agent_id] = agent_planner_ptrs_[agent_id]->getStats().cost;
        new_state->path_cost_lower_bounds[agent_id] = agent_planner_ptrs_[agent_id]->getStats().lower_bound;

        // Add the number of low level nodes to the counter.
        stats_.bonus_stats["num_low_level_expanded"] += agent_planner_ptrs_[agent_id]->getStats().num_expanded;

        // Get the sum of costs for the new state.
        double new_state_soc = std::accumulate(new_state->paths_costs.begin(), new_state->paths_costs.end(), 0.0, [](double acc, const std::pair<int, double>& path_cost) { return acc + path_cost.second; });
        new_state->sum_of_costs = new_state_soc;
        double new_state_lb = std::accumulate(new_state->path_cost_lower_bounds.begin(), new_state->path_cost_lower_bounds.end(), 0.0, [](double acc, const std::pair<int, double>& path_cost) { return acc + path_cost.second; });

        // If there is no path for this agent, then this is not a valid state. Discard it.
        if (new_state->paths[agent_id].empty()) {
            delete new_state;
            continue;
        }

        // The goal state returned is at time -1. We need to fix that and set its time element (last value) to the size of the path.
        new_state->paths[agent_id].back().back() = new_state->paths[agent_id].size() - 1;

        // Get any conflicts between the newly computed paths.
        // NOTE(yoraish):  that this could be checked in any of the action_spaces, since they must all operate on the same scene. This is funky though, since the action_space is not aware of the other agents. Maybe this should be done in the ECBS class, and then passed to the action_space.
        agent_action_space_ptrs_[0]->getPathsConflicts(std::make_shared<MultiAgentPaths>(new_state->paths), 
                                                       new_state->unresolved_conflicts, 
                                                       getConflictTypes(),
                                                       -1, // TODO(yoraish): get all the conflicts.
                                                       agent_names_);

        std::cout << "New state soc: " << new_state_soc << std::endl;
        std::cout << "New state num conflicts: " << new_state->unresolved_conflicts.size() << std::endl;

        new_state->f = new_state_soc;
        new_state->sum_of_path_cost_lower_bounds = new_state_lb;

        // Add a random number between zero and one to f.
        // new_state->f += (double)rand() / RAND_MAX; // Uncomment for nitro boost.

        // Push the new state to the open list.
        open_->push(new_state);
        new_state->setOpen();
        stats_.num_generated++;

        // Delete the previous state but keep the entry in the states_ vector.
        // state = nullptr;
    }
}

