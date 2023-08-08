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
 * \file   experience_heuristic.hpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   8/4/23
*/

#ifndef SEARCH_SEARCH_INCLUDE_SEARCH_HEURISTICS_EXPERIENCE_HEURISTICS_HPP_
#define SEARCH_SEARCH_INCLUDE_SEARCH_HEURISTICS_EXPERIENCE_HEURISTICS_HPP_

#include <iostream>
#include <utility>

#include "search/common/base_heuristic.hpp"
#include "search/common/types.hpp"
#include "search/common/scene_interface.hpp"
#include "search/common/action_space.hpp"
#include "search/common/intrusive_heap.h"

namespace ims {

class GenericEGraphHeuristic : public EGraphHeuristicBase {

public:

    /// @brief Constructor
    /// @param scene_interface pointer to the scene interface
    /// @param origin_heuristic pointer to the origin heuristic
    GenericEGraphHeuristic(const std::shared_ptr<BaseHeuristic>& origin_heuristic,
                           const std::shared_ptr<ExperienceGraphActionSpace>& experience_graph_as) {
//        scene_interface_ = scene_interface;
        origin_heuristic_ = origin_heuristic;
        eg_action_space_ = experience_graph_as;
    }

    /// @brief Destructor
    ~GenericEGraphHeuristic() override = default;

    /// @brief Set the weight (epsilon) for the experience graph
    /// @param eg_epsilon epsilon for the experience graph
    void setEGraphWeight(double eg_epsilon) {
        eg_epsilon_ = eg_epsilon;
        std::cout << BOLDYELLOW << "[INFO] EGraph weight set to: " << eg_epsilon_ << RESET << std::endl;
    }

    /// @brief Get the weight (epsilon) for the experience graph
    /// @return epsilon for the experience graph
    double getEGraphWeight() const {
        return eg_epsilon_;
    }

    /// @brief Get the experience graph action space
    /// @return pointer to the experience graph action space
    std::shared_ptr<ExperienceGraphActionSpace> getEGraphActionSpace() const {
        return eg_action_space_;
    }

    /// @{ overrides

    void getEquivalentStates(int s_id, std::vector<int>& state_ids) override{
        std::shared_ptr<smpl::ExperienceGraph> eg = eg_action_space_->getExperienceGraph();
        // get s_id node
        auto s_state = eg->state(s_id);
        auto nodes = eg->nodes();
        const double equiv_thresh = 1; // TODO: 1? why? should I leave it as is (cost of 1)?
        for (auto nit = nodes.first; nit != nodes.second; ++nit) {
            int egraph_state_id = eg_action_space_->getStateID(*nit);
            auto e_state = eg->state(*nit);
            double h;
            if (!origin_heuristic_->getHeuristic(s_state, e_state, h))
                h = INF_DOUBLE;
            if (h <= equiv_thresh) {
                state_ids.push_back(egraph_state_id);
            }
        }
    }

    void getShortcutSuccessors(int s_id, std::vector<int>& state_ids) override{
        std::vector<smpl::ExperienceGraph::node_id> egraph_nodes;
        eg_action_space_->getEGraphNodes(s_id, egraph_nodes);
        for (auto& n : egraph_nodes) {
            const int comp_id = component_ids_[n];
            for (smpl::ExperienceGraph::node_id sn : shortcut_nodes_[comp_id]) {
                int eg_state_id = eg_action_space_->getStateID(sn);
                if (eg_state_id != s_id)
                    state_ids.push_back(eg_state_id);
            }
        }
    }

    void setGoal(StateType& goal_state) override {
        origin_heuristic_->setGoal(goal_state);
        std::shared_ptr<ims::smpl::ExperienceGraph> eg = eg_action_space_->getExperienceGraph();
        if (!eg) {
            std::cout << BOLDRED << "[ERROR] Experience graph is not initialized!" << RESET << std::endl;
            return;
        }

        // compute connected components in the experience graph
        int comp_count {0};
        component_ids_.assign(eg->num_nodes(), -1);
        auto nodes = eg->nodes();
        for (auto nit = nodes.first; nit != nodes.second; ++nit) {
            if (component_ids_[*nit] == -1) {
                std::vector<smpl::ExperienceGraph::node_id> frontier;
                frontier.push_back(*nit);
                while (!frontier.empty()) {
                    smpl::ExperienceGraph::node_id n = frontier.back();
                    frontier.pop_back();
                    component_ids_[n] = comp_count;
                    auto adj = eg->adjacent_nodes(n);
                    for (auto ait = adj.first; ait != adj.second; ++ait) {
                        if (component_ids_[*ait] == -1) {
                            frontier.push_back(*ait);
                        }
                    }
                }
                comp_count++;
            }
        }
        std::cout << BOLDGREEN << "[INFO] Found " << comp_count << " connected components in the experience graph." << RESET << std::endl;

        /* compute shortcut nodes */

        shortcut_nodes_.assign(comp_count, std::vector<smpl::ExperienceGraph::node_id>());
        std::vector<double> shortcut_heuristics(comp_count);
        for (auto nit {nodes.first}; nit != nodes.second; ++nit){
            const smpl::ExperienceGraph::node_id n = *nit;
            const int comp_id = component_ids_[n];
//            const int state_id = eg_action_space_->getStateID(n);
            StateType state = eg->state(n);
            double h;
            if (!origin_heuristic_->getHeuristic(state, h))
                h = INF_DOUBLE;
            if (shortcut_nodes_[comp_id].empty()) {
                shortcut_nodes_[comp_id].push_back(n);
                shortcut_heuristics[comp_id] = h;
            } else {
                auto best_h = shortcut_heuristics[comp_id];
                if (h < best_h) {
                    shortcut_nodes_[comp_id].clear();
                    shortcut_nodes_[comp_id].push_back(n);
                    shortcut_heuristics[comp_id] = h;
                } else if (h == best_h) {
                    shortcut_nodes_[comp_id].push_back(n);
                }
            }
        }
//
        /* Compute Heuristic distance for Experience Graph nodes */

        h_nodes_.assign(eg->num_nodes(), INF_DOUBLE);
        open_.clear();
        h_nodes_[0].dist = 0;
        open_.push(&h_nodes_[0]);
        while (!open_.empty()) {
            HeuristicNode* s = open_.min();
            open_.pop();

            int nidx = std::distance(h_nodes_.data(), s);
            if (nidx == 0) {
                // neighbors: inflated edges to all experience graph states
                // unconditionally relaxed (goal node is the first node removed)
                for (auto nit = nodes.first; nit != nodes.second; ++nit) {
                    const smpl::ExperienceGraph::node_id nid = *nit;
                    HeuristicNode* n = &h_nodes_[nid + 1];
                    const int state_id = eg_action_space_->getStateID(nid);
                    StateType& state = eg->state(state_id);
                    double h;
                    if (!origin_heuristic_->getHeuristic(state, h))
                        h = INF_DOUBLE;
                    n->dist = eg_epsilon_ * h;
                    open_.push(n);
                }
            } else {
                // neighbors: inflated edges to all non-adjacent experience graph
                // states original cost edges to all adjacent experience graph
                // states
                const smpl::ExperienceGraph::node_id sid = nidx - 1;
                const int state_id = eg_action_space_->getStateID(sid);
                for (auto nit = nodes.first; nit != nodes.second; ++nit) {
                    const smpl::ExperienceGraph::node_id nid = *nit;
                    HeuristicNode* n = &h_nodes_[nid + 1];
                    if (eg->edge(sid, nid)) {
                        const double edge_cost = 10;
                        const double new_cost = s->dist + edge_cost;
                        if (new_cost < n->dist) {
                            n->dist = new_cost;
                            if (open_.contains(n)) {
                                open_.decrease(n);
                            } else {
                                open_.push(n);
                            }
                        }
                    } else {
                        const int nstate_id = eg_action_space_->getStateID(nid);
                        StateType& state = eg->state(nstate_id);
                        double h;
                        if (!origin_heuristic_->getHeuristic(state, h))
                            h = INF_DOUBLE;
                        const double new_cost = s->dist + eg_epsilon_ * h;
                        if (new_cost < n->dist) {
                            n->dist = new_cost;
                            if (open_.contains(n)) {
                                open_.decrease(n);
                            } else {
                                open_.push(n);
                            }
                        }
                    }
                }
            }
        }
        eg_ = eg;
    }

    bool getHeuristic(StateType& s1, StateType& s2, double& dist) override {
        dist = 0;
        return false;
    }

    bool getHeuristic(StateType& s, double& dist) override {
        if (eg_ == nullptr) {
            eg_ = eg_action_space_->getExperienceGraph();
        }
        if (!eg_) {
            dist = 0; // should I return 0?
            return false;
        }

        double best_h; origin_heuristic_->getHeuristic(s, best_h);
        best_h *= eg_epsilon_;
        auto nodes = eg_->nodes();
        for (auto nit = nodes.first; nit != nodes.second; ++nit) {
            const smpl::ExperienceGraph::node_id n = *nit;
            const int state_id = eg_action_space_->getStateID(n);
            StateType& state = eg_->state(state_id);
            double h;
            if (!origin_heuristic_->getHeuristic(s, state, h))
                h = INF_DOUBLE;
            const double distance = h_nodes_[n + 1].dist;
            const double new_h = distance + eg_epsilon_ * h;
            if (new_h < best_h) {
                best_h = new_h;
            }
        }
        dist = best_h;
        return true;
    }
    /// TODO: Do i need heuristic to start?

    /// @}


private:

//    std::shared_ptr<SceneInterface> scene_interface_ {nullptr};
    std::shared_ptr<BaseHeuristic> origin_heuristic_{nullptr};

    std::shared_ptr<ExperienceGraphActionSpace> eg_action_space_ {nullptr};

    std::shared_ptr<smpl::ExperienceGraph> eg_ {nullptr};

    double eg_epsilon_ {1.0};

    std::vector<int> component_ids_;
    std::vector<std::vector<smpl::ExperienceGraph::node_id>> shortcut_nodes_;

    struct HeuristicNode : public ::smpl::HeapElement
    {
        double dist{};

        HeuristicNode() = default;
        HeuristicNode(double d) : HeapElement(), dist(d) { }
    };

    struct NodeCompare
    {
        bool operator()(const HeuristicNode& a, const HeuristicNode& b) const
        {
            return a.dist < b.dist;
        }
    };

    std::vector<HeuristicNode> h_nodes_;
    ::smpl::IntrusiveHeap<HeuristicNode, NodeCompare> open_;

};

}


#endif //SEARCH_SEARCH_INCLUDE_SEARCH_HEURISTICS_EXPERIENCE_HEURISTICS_HPP_
