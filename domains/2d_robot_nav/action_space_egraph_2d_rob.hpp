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
 * \file   action_space_egraph_2d_rob.hpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   8/6/23
*/


#ifndef SEARCH_SEARCH_DOMAINS_2D_ROBOT_NAV_ACTION_SPACE_EGRAPH_2D_ROB_HPP_
#define SEARCH_SEARCH_DOMAINS_2D_ROBOT_NAV_ACTION_SPACE_EGRAPH_2D_ROB_HPP_

// standard includes
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
// package includes
#include "action_space_2d_rob.hpp"
#include "search/common/experience_graph.hpp"
#include "search/common/intrusive_heap.h"

/// @class ActionSpaceEGraph2DRob - actionSpace2DRob with ExperienceGraph
/// @brief this class implements the ActionSpace2DRob interface with ExperienceGraph support for the 2D Robot Navigation domain
/// using double inheritance of two classes: ActionSpace2DRob and ExperienceGraphActionSpace
class ActionSpaceEGraph2DRob : public actionSpace2dRob,
                               public ims::ExperienceGraphActionSpace {
    public:

    /// @brief constructor
    /// @param env - scene interface
    /// @param actions_ptr - pointer to action type
    ActionSpaceEGraph2DRob(const scene2DRob& env, const actionType2dRob& actions_ptr) :
            actionSpace2dRob(env, actions_ptr) {}

    /// @brief interpolate between two states
    /// @param state_1 - first state
    /// @param state_2 - second state
    /// @param path - the interpolated path
    /// @resolution - resolution of the interpolation
    void interpolatePath(const StateType& state_1, const StateType& state_2,std::vector<StateType>& path,
                         double resolution = 1) {
        path.clear();
        path.push_back(state_1);
        double dist = std::sqrt(std::pow(state_1[0] - state_2[0], 2) + std::pow(state_1[1] - state_2[1], 2));
        int num_steps = std::ceil(dist / resolution);
        double step_size = dist / num_steps;
        for (int i = 1; i < num_steps; ++i) {
            StateType state = state_1;
            double ratio = i * step_size / dist;
            state[0] = state_1[0] + ratio * (state_2[0] - state_1[0]);
            state[1] = state_1[1] + ratio * (state_2[1] - state_1[1]);
            // discretize the state
            state[0] = std::round(state[0] / actions_->state_discretization_[0]) * actions_->state_discretization_[0];
            state[1] = std::round(state[1] / actions_->state_discretization_[1]) * actions_->state_discretization_[1];
            path.push_back(state);
        }
        path.push_back(state_2);
    }

    /// @brief check if state to state action is valid
    /// @param state_1 - first state
    /// @param state_2 - second state
    /// @return true if the action is valid, false otherwise
    bool isStateToStateValid(const StateType& state_1, const StateType& state_2) {
        std::vector<StateType> path;
        interpolatePath(state_1, state_2, path);
        return actionSpace2dRob::isPathValid(path);
    }


    /// @{ override functions from ExperienceGraphActionSpace
    bool loadEGraph(const std::string& path) override {
        // The path needs to be a directory containing the experience files
        // check if path is a directory
        boost::filesystem::path p(path);
        if (!boost::filesystem::is_directory(p)) {
            std::cout << RED << "[ERROR]: Path in loadEGraph is not a directory" << RESET << std::endl;
            return false;
        }

        // loop through all files in the directory and parse them
        for (auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(p), {})) {
            if (entry.path().extension() == ".csv") {
                std::vector<StateType> egraph_states;
                if (!parseEGraphFile(entry.path().string(), egraph_states) || egraph_states.empty()) {
                    continue;
                }
                auto& prev_state = egraph_states.front();
                auto pid = egraph_.insert_node(prev_state);
                state_to_nodes_map_[prev_state].push_back(pid);

                int entry_s_id = getOrCreateRobotState(prev_state);

                // map the state id to the node id in the experience graph
                egraph_state_ids_.resize(pid + 1, -1);
                egraph_state_ids_[pid] = entry_s_id;
                state_id_to_egraph_node_id_map_[entry_s_id] = pid;

                std::vector<StateType> edge_data;
                for (size_t i = 1; i < egraph_states.size(); ++i) {
                    auto& curr_state = egraph_states[i];
                    StateType cs = curr_state;
                    if (curr_state != prev_state) { // TODO: check if its fine
                        auto cid = egraph_.insert_node(curr_state);
                        state_to_nodes_map_[curr_state].push_back(cid);

                        int curr_s_id = getOrCreateRobotState(curr_state);

                        // map the state id to the node id in the experience graph
                        egraph_state_ids_.resize(cid + 1, -1);
                        egraph_state_ids_[cid] = curr_s_id;
                        state_id_to_egraph_node_id_map_[curr_s_id] = cid;

                        // add edge
                        egraph_.insert_edge(pid, cid, edge_data);
                        pid = cid;
                        prev_state = cs;
                    } else {
                        edge_data.push_back(curr_state);
                    }
                }
            }
        }
        return true;
    }

    void getEGraphNodes(int state_id,
                        std::vector<ims::smpl::ExperienceGraph::node_id> &nodes) override {
        auto it = state_id_to_egraph_node_id_map_.find(state_id);
        if (it != state_id_to_egraph_node_id_map_.end()) {
            nodes.push_back(it->second);
        }
    }

    bool shortcut(int first_id, int second_id, int& cost) override {
        auto* state_1 = getRobotHashEntry(first_id);
        auto* state_2 = getRobotHashEntry(second_id);

        if (state_1 == nullptr || state_2 == nullptr) {
            return false;
        }
        cost = 1;
        return true;
    }

    bool checkShortcutTransition(int first_id,
                                 int second_id,
                                 PathType& trans_path) override {
        auto prev_nit = std::find(egraph_state_ids_.begin(), egraph_state_ids_.end(), first_id);
        auto curr_nit = std::find(egraph_state_ids_.begin(), egraph_state_ids_.end(), second_id);
        if (prev_nit != egraph_state_ids_.end() &&
            curr_nit != egraph_state_ids_.end()) {
            ims::smpl::ExperienceGraph::node_id prev_nid = std::distance(egraph_state_ids_.begin(), prev_nit);
            ims::smpl::ExperienceGraph::node_id curr_nid = std::distance(egraph_state_ids_.begin(), curr_nit);
            std::vector<ims::smpl::ExperienceGraph::node_id> node_path;
            bool found = findShortestExperienceGraphPath(prev_nid, curr_nid, node_path);
            if (found){
                for (ims::smpl::ExperienceGraph::node_id n : node_path){
                    int s_id = egraph_state_ids_[n];
                    auto* entry = getRobotHashEntry(s_id);
                    assert(entry);
                    trans_path.push_back(entry->state);
                }
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }

    bool snap(int first_id, int second_id, int& cost) override {
        auto* state_1 = getRobotHashEntry(first_id);
        auto* state_2 = getRobotHashEntry(second_id);

        if (state_1 == nullptr || state_2 == nullptr) {
            return false;
        }

        if (isStateToStateValid(state_1->state, state_2->state)) {
            cost = 1;
            return true;
        } else {
            return false;
        }
    }

    bool checkSnapTransition(int first_id,
                             int second_id,
                             PathType& trans_path) override {
        int cost;
        if (snap(first_id, second_id, cost)){
            auto* entry = getRobotHashEntry(second_id);
            assert(entry);
            trans_path.push_back(entry->state);
            return true;
        } else {
            return false;
        }
    }

    const std::shared_ptr<ims::smpl::ExperienceGraph> getExperienceGraph() const override {
        return std::make_shared<ims::smpl::ExperienceGraph>(egraph_);
    }

    std::shared_ptr<ims::smpl::ExperienceGraph> getExperienceGraph() override {
        return std::make_shared<ims::smpl::ExperienceGraph>(egraph_);
    }

    int getStateID(ims::smpl::ExperienceGraph::node_id n) const override {
        if (n < egraph_state_ids_.size()) {
            return egraph_state_ids_[n];
        } else {
            return -1;
        }
    }

private:

    // hash map from state to node id in the experience graph
    typedef hash_map<StateType,
                     std::vector<ims::smpl::ExperienceGraph::node_id>,
                     StateTypeHash> StateToEGraphNodesMap;

    StateToEGraphNodesMap state_to_nodes_map_;
    hash_map<int, ims::smpl::ExperienceGraph::node_id> state_id_to_egraph_node_id_map_;

    ims::smpl::ExperienceGraph egraph_;

    std::vector<int> egraph_state_ids_;

    /// @brief Find the shortest experience path between two states
    /// @param start_node - start node id
    /// @param goal_node - goal node id
    /// @param path - vector of states in the shortest experience path
    /// @return true if path was found, false otherwise
    bool findShortestExperienceGraphPath(
        ims::smpl::ExperienceGraph::node_id start_node,
        ims::smpl::ExperienceGraph::node_id goal_node,
        std::vector<ims::smpl::ExperienceGraph::node_id>& path)
    {
        struct ExperienceGraphSearchNode : smpl::HeapElement
        {
            int g;
            bool closed;
            ExperienceGraphSearchNode* bp;
            ExperienceGraphSearchNode() :
                g(std::numeric_limits<int>::max()),
                closed(false),
                bp(nullptr)
            { }
        };

        struct NodeCompare
        {
            bool operator()(
                const ExperienceGraphSearchNode& a,
                const ExperienceGraphSearchNode& b)
            {
                return a.g < b.g;
            }
        };

        typedef smpl::IntrusiveHeap<ExperienceGraphSearchNode, NodeCompare> heap_type;

        std::vector<ExperienceGraphSearchNode> search_nodes(egraph_.num_nodes());

        heap_type open;

        search_nodes[start_node].g = 0;
        open.push(&search_nodes[start_node]);
        int exp_count = 0;
        while (!open.empty()) {
            ++exp_count;
            ExperienceGraphSearchNode* min = open.min();
            open.pop();
            min->closed = true;

            if (min == &search_nodes[goal_node]) {
                std::cout << RED << "[ERROR]: Found shortest experience graph path" << RESET << std::endl;
                ExperienceGraphSearchNode* ps = nullptr;
                for (ExperienceGraphSearchNode* s = &search_nodes[goal_node];
                     s; s = s->bp)
                {
                    if (s != ps) {
                        path.push_back(std::distance(search_nodes.data(), s));
                        ps = s;
                    } else {
                        std::cout << RED << "[ERROR]: Cycle detected!" << RESET << std::endl;
                    }
                }
                std::reverse(path.begin(), path.end());
                return true;
            }

            ims::smpl::ExperienceGraph::node_id n = std::distance(search_nodes.data(), min);
            auto adj = egraph_.adjacent_nodes(n);
            for (auto ait = adj.first; ait != adj.second; ++ait) {
                ExperienceGraphSearchNode& succ = search_nodes[*ait];
                if (succ.closed) {
                    continue;
                }
                int new_cost = min->g + 1;
                if (new_cost < succ.g) {
                    succ.g = new_cost;
                    succ.bp = min;
                    if (open.contains(&succ)) {
                        open.decrease(&succ);
                    } else {
                        open.push(&succ);
                    }
                }
            }
        }

        std::cout << "Expanded " << exp_count << " nodes looking for shortcut" << std::endl;
        return false;
    }

    /// @brief Parsing function of experience graph file
    /// @param filepath - path to experience graph file
    /// @param egraph_states - vector of states in the experience graph
    /// @return true if parsing was successful, false otherwise
    bool parseEGraphFile(const std::string& filepath,
                         PathType& egraph_states) {
        std::ifstream egraph_file(filepath);
        if (!egraph_file.is_open()) {
            std::cout << RED << "[ERROR]: Failed to open experience graph file: " << filepath << RESET << std::endl;
            return false;
        }

        /* parse CSV
            the format of the CSV file is as follows:
                1. header line: "Experience, N (number of states), dim (dimension of state)"
                2. state lines: "state_1, state_2, ..., state_n"
        */

        std::string line;
        std::vector<std::string> tokens;
        std::vector<std::string> state_tokens;
        StateType state_values;
        int num_states = 0;
        int dim = 0;
        while (std::getline(egraph_file, line)) {
            boost::split(tokens, line, boost::is_any_of(","));
            if (tokens[0] == "Experience") {
                num_states = std::stoi(tokens[1]);
                dim = std::stoi(tokens[2]);
                egraph_states.reserve(num_states);
            } else {
                state_tokens.clear();
                state_tokens = tokens;
                state_values.clear();
                state_values.reserve(dim);
                for (const auto& token : state_tokens) {
                    state_values.push_back(std::stod(token));
                }
                egraph_states.emplace_back(state_values);
            }
        }
        return true;
    }




};

#endif //SEARCH_SEARCH_DOMAINS_2D_ROBOT_NAV_ACTION_SPACE_EGRAPH_2D_ROB_HPP_
