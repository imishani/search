/*
 * Copyright (C) 2023, Itamar Mishani
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
 * \file   astar.hpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   3/30/23
*/

#include <search/planners/dijkstra.hpp>


ims::Dijkstra::Dijkstra(const DijkstraParams &params) : AStar(params) {}


void ims::Dijkstra::initializePolicyPlanner(const std::shared_ptr<ActionSpace> &action_space_ptr,
                                            const StateType &start_state) {

    // Space pointer.
    action_space_ptr_ = action_space_ptr;

    // Clear both.
    action_space_ptr_->resetPlanningData();
    resetPlanningData();

    // check if start is valid
    if (!action_space_ptr_->isStateValid(start_state)){
        std::cout << "Start state is not valid" << std::endl;
        throw std::runtime_error("Start state is not valid");
    }
    int start_ind_ = action_space_ptr_->getOrCreateRobotState(start_state);
    auto start_ = getOrCreateSearchState(start_ind_);
    goal_ = start_ind_;

    start_->parent_id = PARENT_TYPE(START);
    // cast the heuristic
    heuristic_->setStart(const_cast<StateType &>(start_state));
    // Evaluate the start state
    start_->g = 0;
    start_->h = computeHeuristic(start_ind_);
    start_->f = start_->g + params_.epsilon*start_->h;
    start_->setOpen();

    open_.push(start_);
}


bool ims::Dijkstra::exhaustPlan() {
    startTimer();
    int iter {0};
    while (!open_.empty()){
        auto state  = open_.min();
        open_.pop();
        expand(state->state_id);
        iter++;
    }
    if (isTimeOut()){
        std::cout << "Time out!" << std::endl;
        return false;
    }
    else{
        std::cout << "Open got empty!" << std::endl;
        // report stats
        getTimeFromStart(stats_.time);
        std::cout << "Time: " << stats_.time << std::endl;
        return true;
    }

}


ims::Policy ims::Dijkstra::getPolicy(int state_id) {
    Policy policy;
    policy.state = action_space_ptr_->getRobotState(state_id)->state;
    policy.goal_state = action_space_ptr_->getRobotState(goal_)->state;
    std::vector<int> successors;
    std::vector<double> costs;
    action_space_ptr_->getSuccessors(state_id, successors, costs);
    double max_cost = std::numeric_limits<double>::max();
    for (int i {0}; i < successors.size(); i++){
        int successor_id = successors[i];
        double cost = costs[i];
        auto successor = getOrCreateSearchState(successor_id);
        if (successor->g + cost < max_cost){
            max_cost = successor->g + cost;
            policy.next_state = action_space_ptr_->getRobotState(successor_id)->state;
            policy.action.resize(policy.next_state.size());
            for (int j = 0; j < policy.next_state.size(); j++){
                policy.action[j] = policy.next_state[j] - policy.state[j];
            }
            policy.cost = cost;
            policy.q_value = successor->g; // TODO: It is a little fucked up. I should define a new planner named value iteration or something
        }
    }
    return policy;
}


std::vector<ims::Policy> ims::Dijkstra::getSuccessorPolicies(const int state_id) {
    const StateType search_state = action_space_ptr_->getRobotState(state_id)->state;
    const StateType goal_state = action_space_ptr_->getRobotState(goal_)->state;
    std::vector<Policy> policies;
    std::vector<int> successors;
    std::vector<double> costs;
    action_space_ptr_->getSuccessors(state_id, successors, costs);
    for (int i {0}; i < successors.size(); i++) {
        Policy policy;
        policy.state = search_state;
        policy.goal_state = goal_state;

        policy.next_state = action_space_ptr_->getRobotState(successors[i])->state;
        policy.action.resize(policy.next_state.size());
        for (int j = 0; j < policy.next_state.size(); j++){
            policy.action[j] = policy.next_state[j] - policy.state[j];
        }
        policy.cost = costs[i];
        policy.q_value = getOrCreateSearchState(successors[i])->g;
        policies.push_back(policy);
    }
    return policies;
}


