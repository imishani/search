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
 * \file   prioritized_planning.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   Sept 14 2023
 */

#include <search/planners/astar.hpp>
#include <search/planners/multi_agent/prioritized_planning.hpp>


ims::PrioritizedPlanning::PrioritizedPlanning(const ims::PrioritizedPlanningParams& params) : params_(params), MultiAgentPlanner(params){}

void ims::PrioritizedPlanning::initializePlanner(std::vector<std::shared_ptr<ConstrainedActionSpace>>& action_space_ptrs,
                                const std::vector<StateType>& starts, const std::vector<StateType>& goals) {
    // We do not have agent names, so assign them.
    std::vector<std::string> agent_names;
    for (int i{0}; i < starts.size(); ++i) {
        agent_names.push_back("agent" + std::to_string(i));
    }
    initializePlanner(action_space_ptrs, agent_names, starts, goals);
    }

void ims::PrioritizedPlanning::initializePlanner(std::vector<std::shared_ptr<ConstrainedActionSpace>>& action_space_ptrs, const std::vector<std::string> & agent_names, const std::vector<StateType>& starts, const std::vector<StateType>& goals){
    agent_names_ = agent_names;

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
        ims::wAStarParams wastar_params_(params_.low_level_heuristic_ptrs[i], params_.weight_low_level_heuristic);
        
        agent_planner_ptrs_.push_back(std::make_shared<ims::wAStar>(wastar_params_));
    }

    // Check if there is a priority ordering passed in the params. If not, create a random one.
    if (params_.agent_priority_ordering.empty()) {
        // Create a random priority ordering.
        for (int i{0}; i < num_agents_; ++i) {
            params_.agent_priority_ordering.push_back(i);
        }
        std::random_shuffle(params_.agent_priority_ordering.begin(), params_.agent_priority_ordering.end());
    }
}


bool ims::PrioritizedPlanning::plan(MultiAgentPaths& paths) {
    startTimer();

    // For each agent in the priority ordering, plan a path that respects the paths of the agents with higher priority.
    // Any computed path is stored in the `paths` variable, and downstream (lower-priority) agents must respect these paths. These will be specified to the single-agent planners via a constraint-context: asking each agent to avoid all other higher-priority agents at all timesteps.
    stats_.cost = 0;
    stats_.bonus_stats["num_low_level_expanded"] = 0;

    std::shared_ptr<ConstraintsCollective> constraints_collective_ptr = std::make_shared<ConstraintsCollective>();
    std::shared_ptr<ConstraintsContext> constraints_context_ptr = std::make_shared<ConstraintsContext>();

    for (int agent_id : params_.agent_priority_ordering) {
        // Get the start and goal states for this agent.
        StateType start = starts_[agent_id];
        StateType goal = goals_[agent_id];

        // Create a new action space for this agent.
        auto action_space_ptr = agent_action_space_ptrs_[agent_id];
        
        // Set the constraints on this agent.
        action_space_ptr->setConstraintsCollective(constraints_collective_ptr);
        // action_space_ptr->setConstraintsContext(constraints_context_ptr);

        // Create a new planner for this agent.
        auto planner_ptr = agent_planner_ptrs_[agent_id];

        // Initialize the planner.
        planner_ptr->initializePlanner(action_space_ptr, start, goal);

        // Plan a path.
        std::vector<StateType> path;
        bool plan_success = planner_ptr->plan(path);
        stats_.cost += planner_ptr->reportStats().cost;
        stats_.bonus_stats["num_low_level_expanded"] += planner_ptr->reportStats().num_expanded;

        if (!plan_success) {
            // If the planning failed, return false.
            std::cout << RED << "Planning failed for agent " << agent_id << ", priority " << std::distance(params_.agent_priority_ordering.begin(), std::find(params_.agent_priority_ordering.begin(), params_.agent_priority_ordering.end(), agent_id)) << ". Prioritized planning failed." << RESET << std::endl;
            getTimeFromStart(stats_.time);
            paths.clear();
            return false;
        }

        // Fix the last path state to have a correct time and not -1.
        path.back().back() = path.size() - 1;

        // Add the path to the paths.
        paths[agent_id] = path;

        // Set the constraints on this agent. These constraints are all avoidance constraints: avoiding all other planned (higher-priority) agents. The current agent a_l gets a constraint for avoiding another agent a_h at all times when a_h is planned. An extra constraint is added for avoiding the goal of a_h at "all times." This constraint is required to keep a_l from colliding with a_h at a_h's goal after a_h has gotten there, and while doing it not to extend the time of the last constraint on a_l such that it could accept a goal state for a_l at a time after all other agents have stopped planning.
        // Add the newly planned path to the constraints context.
        constraints_context_ptr->agent_paths[agent_id] = path;
        constraints_collective_ptr->setContext(constraints_context_ptr);

        // Add a constraint for avoiding the agent at all times. Reminder: if we want the next planned agent to be disallowed from terminating the search before all other previous agent have terminated their motions, we need to specify a latest constraint time directly. This time is often used by planners to determine if a goal state can be found.
        std::shared_ptr<VertexAvoidanceConstraint> vertex_avoid_constraint_ptr = std::make_shared<VertexAvoidanceConstraint>(agent_id, -1, agent_names_[agent_id]);
        std::shared_ptr<EdgeAvoidanceConstraint> edge_avoid_constraint_ptr = std::make_shared<EdgeAvoidanceConstraint>(agent_id, -1, -1, agent_names_[agent_id]);

        // Uniquely in PP, if the constraints collective already has a vertex-avoid and/or edge-avoid constraints for this infinite time-interval, simply modify those to also include this agent.
        bool found_vertex_avoid_constraint = false;
        bool found_edge_avoid_constraint = false;
        for (auto& constraint_ptr : constraints_collective_ptr->getConstraints()) {
            if (constraint_ptr->type == ConstraintType::VERTEX_AVOIDANCE) {
                auto found_vertex_avoid_constraint_ptr = std::dynamic_pointer_cast<VertexAvoidanceConstraint>(constraint_ptr);
                if (found_vertex_avoid_constraint_ptr->getTimeInterval().first == -1 && found_vertex_avoid_constraint_ptr->getTimeInterval().second == -1) {
                    found_vertex_avoid_constraint_ptr->agent_ids_to_avoid.push_back(agent_id);
                    found_vertex_avoid_constraint_ptr->agent_names_to_avoid.push_back(agent_names_[agent_id]);
                    found_vertex_avoid_constraint = true;
                }
            }
            else if (constraint_ptr->type == ConstraintType::EDGE_AVOIDANCE) {
                auto found_edge_avoid_constraint_ptr = std::dynamic_pointer_cast<EdgeAvoidanceConstraint>(constraint_ptr);
                if (edge_avoid_constraint_ptr->getTimeInterval().first == -1 && found_edge_avoid_constraint_ptr->getTimeInterval().second == -1) {
                    found_edge_avoid_constraint_ptr->agent_ids_to_avoid.push_back(agent_id);
                    found_edge_avoid_constraint_ptr->agent_names_to_avoid.push_back(agent_names_[agent_id]);
                    found_edge_avoid_constraint = true;
                }
            }
        }

        // If there were no updates in the constraints collective, then set the constraints.
        if (!found_vertex_avoid_constraint) {
            constraints_collective_ptr->addConstraint(vertex_avoid_constraint_ptr);
        }
        if (!found_edge_avoid_constraint) {
            constraints_collective_ptr->addConstraint(edge_avoid_constraint_ptr);
        }

        std::pair<int, int> interval = edge_avoid_constraint_ptr->getTimeInterval();        
        constraints_collective_ptr->setLastConstraintTimeToAtLeast(path.size() - 1);

        if (isTimeOut()){
            std::cout << RED << "Time out for agent " << agent_id << ", priority " << std::distance(params_.agent_priority_ordering.begin(), std::find(params_.agent_priority_ordering.begin(), params_.agent_priority_ordering.end(), agent_id)) << ". Prioritized planning failed." << RESET << std::endl;
            getTimeFromStart(stats_.time);
            paths.clear();
            return false;
        }
    }
        
    getTimeFromStart(stats_.time);
    
    if (paths.size() == num_agents_ && !isTimeOut()) {
        return true;
    }
    else {
        return false;
    }
}

void ims::PrioritizedPlanning::padPathsToMaxLength(MultiAgentPaths& paths) {
    // Pad all paths to the same length. Do this by adding the last state of the path to the end of the path (the state is identical, so time may be repeated).
    int max_path_length = (int)std::max_element(paths.begin(), paths.end(), [](const std::pair<int, std::vector<StateType>>& a, const std::pair<int, std::vector<StateType>>& b) { return a.second.size() < b.second.size(); })->second.size();

    // Pad all paths to the same length.
    for (auto& path : paths) {
        int agent_id = path.first;
        int path_length = (int)path.second.size();
        for (int i{0}; i < max_path_length - path_length; ++i) {
            // The last state.
            StateType last_state = path.second.back();
            // Increment time by 1.
            last_state.back() += 1;
            path.second.push_back(last_state);
        }
    }
}

void ims::PrioritizedPlanning::verifyStartAndGoalInputStates(const std::vector<StateType>& starts, const std::vector<StateType>& goals) {
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

void ims::PrioritizedPlanning::resetPlanningData() {
    // Reset the low-level planners.
    for (auto& planner_ptr : agent_planner_ptrs_) {
        planner_ptr->resetPlanningData();
    }
    // Reset the action spaces.
    for (auto& action_space_ptr : agent_action_space_ptrs_) {
        action_space_ptr->resetPlanningData();
    }
    // Reset the starts and goals.
    starts_.clear();
    goals_.clear();
    // Reset the agent names.
    agent_names_.clear();
    // Reset the priority ordering.
    params_.agent_priority_ordering.clear();
    // Reset the number of agents.
    num_agents_ = 0;
}