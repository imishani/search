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

void ims::CBSSIPP::createLowLevelPlanners() {
    int num_agents = (int)agent_action_space_ptrs_.size();
    assert(num_agents != 0);
    for (size_t i{0}; i < num_agents; ++i) {
        if (params_.low_level_heuristic_ptrs.size() != num_agents) {
            throw std::runtime_error("Number of low-level heuristics (" + std::to_string(params_.low_level_heuristic_ptrs.size()) + ") does not match the number of agents (" + std::to_string(num_agents) + ")");
        }
        ims::SIPPParams sipp_params_(params_.low_level_heuristic_ptrs[i],
                                                  params_.weight_low_level_heuristic);
        sipp_params_.verbose = params_.verbose;
        sipp_params_.time_limit_ = params_.time_limit_;
        agent_planner_ptrs_.push_back(std::make_shared<ims::SIPP>(sipp_params_));
    }
}

bool ims::CBSSIPP::initializeAndPlanLowLevel(int agent_id, PathType &path,
                                              PlannerStats &stats) {
    agent_planner_ptrs_.at(agent_id)->initializePlanner(agent_action_space_ptrs_.at(agent_id),
                                                 starts_.at(agent_id),
                                                 goals_.at(agent_id));
    bool is_plan_success = agent_planner_ptrs_[agent_id]->plan(path);
    stats = agent_planner_ptrs_[agent_id]->getStats();
    return is_plan_success;
}