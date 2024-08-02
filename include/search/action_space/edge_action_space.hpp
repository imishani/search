/*
 * Copyright (C) 2024, Hanlan Yang
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
 * \file   edge_action_space.hpp
 * \author Hanlan Yang (yanghanlan666@gmail.com)
 * \date   7/20/24
 */

#pragma once

#include "action_space.hpp"
#include "mixin/action_space_edge_mixin.hpp"

namespace ims {

///@brief Action space for E-graph search
class EdgeActionSpace : public ActionSpace,
                        public ActionSpaceEdgeMixin {
public:
    /// @brief Constructor.
    explicit EdgeActionSpace() : ActionSpace(), ActionSpaceEdgeMixin() {
        // std::cout << "EdgeActionSpace: Constructor" << std::endl;
    }

    /// @brief Destructor.
    ~EdgeActionSpace() {
        resetPlanningData();
    };

    virtual bool createRobotEdgesFromState(int state_id, std::vector<int>& edges_ind) override {
        // ActionSequence vec
        std::vector<ActionSequence> action_seqs;
        getActions(state_id, action_seqs, false);
        if (action_seqs.empty()) {
            return false;
        }
        auto state_val = getRobotState(state_id)->state;

        // Create edges
        edges_ind.clear();
        for (auto act : action_seqs) {
            edges_ind.emplace_back(getOrCreateRobotEdge(std::make_pair(state_val, act)));
        }
        return true;
    }

    virtual int getOrCreateProxyEdge(StateType state_val) {
        // check if the proxy edge exists
        lock_e_.lock();
        auto curr_edge = std::make_shared<RobotEdge>();
        curr_edge->state = state_val;
        curr_edge->action = ActionSequence();
        auto it = edge_to_id_.find(curr_edge);
        if (it != edge_to_id_.end()) {
            lock_e_.unlock();
            return it->second;
        } else {
            edges_.push_back(curr_edge);
            int edge_id = (int)edges_.size() - 1;
            edge_to_id_[curr_edge] = edge_id;
            lock_e_.unlock();
            int state_id = getOrCreateRobotState(state_val);
            curr_edge->state_id = state_id;
            return edge_id;
        }
    }

    virtual int createProxyEdgeFromState(int state_id) {
        auto state_val = getRobotState(state_id)->state;
        return getOrCreateProxyEdge(state_val);
    }

    /// @brief Overrides reset planning data
    void resetPlanningData() override {
        ActionSpace::resetPlanningData();
        ActionSpaceEdgeMixin::resetPlanningData();
    }
};
}  // namespace ims
