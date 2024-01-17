/*
 * Copyright (C) 2024, Rishi Veerapaneni
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
 * \file   custom_grid_heuristic.hpp
 * \author Rishi V
 * \date   1/16/2024
 */

#pragma once

#include "standard_heuristics.hpp"
#include <unordered_map>
#include "search/planners/dijkstra.hpp"

#include <boost/functional/hash.hpp>
#include <tuple>

namespace std
{

template<typename... T>
struct hash<tuple<T...>>
{
    size_t operator()(tuple<T...> const& arg) const noexcept
    {
        return boost::hash_value(arg);
    }
};

}

namespace ims {

struct CustomGridHeuristic : public BaseHeuristic {
    /// @brief The data structure keeping all the heuristic values.
    std::vector<std::vector<int>> heuristic_values_;
    // std::unordered_map<StateType, int> heuristic_map_;
    std::unordered_map<std::tuple<int, int>, int> heuristic_map_;

    /// @brief Constructor. Passing in an action space of the relevant agent, the map itself (scene-interface), alongside with its size (x, y).
    CustomGridHeuristic(std::shared_ptr<ActionSpace> action_space_ptr, const std::vector<StateType>& goals): BaseHeuristic() {
        // Try out running exhaustive dijkstra on the grids.
        // Set up the planner.
        ims::DijkstraParams params_dijk;
        params_dijk.time_limit_ = 10; // Run for 10 seconds
        ims::Dijkstra planner_dijk(params_dijk);

        // Set all the starts. These are the given goals.
        planner_dijk.initializePlanner(action_space_ptr, /*starts=*/ goals, /*goals=*/ goals);

        // Request an exhaustive plan to get the g-values of all the states w.r.t. the goals.
        bool success_dijk = planner_dijk.exhaustPlan();
        double used_planning_time_dijk = std::ceil(planner_dijk.reportStats().time);

        // Get all the g-values.
        std::vector<std::pair<StateType, double>> g_values = planner_dijk.getAllGValues();

        // std::vector<std::vector<int>> heuristic_grid(x_size, std::vector<int>(y_size, INF_INT));
        for (const std::pair<StateType, double> &g_value : g_values){
            // convert to tuple and insert into map
            std::tuple<int, int> state_tuple = std::make_tuple(g_value.first[0], g_value.first[1]);
            heuristic_map_[state_tuple] = (int)g_value.second;
            // heuristic_map_[g_value.first] = (int)g_value.second;
            // heuristic_grid[g_value.first[0] -1][g_value.first[1] -1] = (int)g_value.second;
        }
    }

    bool getHeuristic(const StateType& s1,
                      double& dist) override {
        // TODO(yoraish): if we want to incorporate this into search, then make sure to have this work with arbitrary state types.
        // if (s1[0] > heuristic_values_.size() || s1[1] > heuristic_values_[0].size()) {
        //     std::cout << "Error: The state is out of bounds!" << std::endl;
        //     return false;
        // }

        // find s1 in the map
        std::tuple<int, int> state_tuple = std::make_tuple(s1[0], s1[1]);
        if (heuristic_map_.find(state_tuple) == heuristic_map_.end()) {
            std::cout << "Error: The state is not in the map!" << std::endl;
            return false;
        }
        dist = heuristic_map_[state_tuple];

        // dist = heuristic_values_[s1[0]-1][s1[1]-1];
        // dist = heuristic_values_[s1[0]][s1[1]];
        return true;
    }

    bool getHeuristic(const StateType& s1, const StateType& s2,
                     double& dist) override {
        std::cout << "Error: NOT SUPPORTED FOR THIS HEURISTIC!" << std::endl;
        return false;
        // check id the states are the same size
        // if (s1.size() != s2.size()) {
        //     std::cout << "Error: The states are not the same size!" << std::endl;
        //     return false;
        // } else {
        //     dist = heuristic_values_[s1[0]][s1[1]];
        //     return true;
        // }
    }

};



}