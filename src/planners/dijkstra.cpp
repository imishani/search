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

std::vector<std::pair<StateType, double>> ims::Dijkstra::getAllGValues() {
    std::vector<std::pair<StateType, double>> g_values;
    for (auto &state : states_){
        // Get the robot state.
        auto robot_state = action_space_ptr_->getRobotState(state->state_id);
        g_values.emplace_back(robot_state->state, state->g);
    }
    return g_values;
}

