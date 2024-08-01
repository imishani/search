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
 * \file   action_space.hpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   3/28/23
 */

#pragma once

// standard includes
#include <iostream>
#include <memory>
#include <numeric>

// project includes
#include "search/common/scene_interface.hpp"
#include "search/common/types.hpp"
#include "search/common/utils.hpp"

namespace ims {

///@brief RobotState base class
struct RobotState {
    StateType state_mapped;
    StateType state;

    enum VALID { valid,
                 invalid,
                 unknown };
    VALID validity = unknown;
};

/// @brief equality operator for RobotState
inline bool operator==(const RobotState& lhs, const RobotState& rhs) {
    return lhs.state == rhs.state;
}
}  // namespace ims

namespace std {
template <>
struct hash<ims::RobotState> {
    using argument_type = ims::RobotState;
    using result_type = std::size_t;

    result_type operator()(const argument_type& s) const {
        size_t seed = 0;
        boost::hash_combine(seed, boost::hash_range(s.state.begin(), s.state.end()));
        return seed;
    }
};
}  // namespace std

namespace ims {

/// @brief Action type abstract struct. This struct should be inherited by the action type
/// Make sure to implement the getPrimActions() function
struct ActionType {
    /// @brief Constructor
    ActionType() = default;

    /// @brief Destructor
    virtual ~ActionType() = default;

    /// @brief Get the possible actions
    /// @return The possible actions
    [[deprecated("Use the new getPrimActions (for sequences) instead.")]]
    virtual std::vector<Action> getPrimActions() = 0;
    virtual void getPrimActions(std::vector<ActionSequence>& action_seqs, std::vector<std::vector<double>>& action_transition_costs) = 0;

    /// @brief Get the resolution of the state space (for discretization)
    /// @param state_des The state discretization
    virtual void Discretization(StateType& state_des) = 0;

    // The discretization steps for each dimension in the state vector.
    StateType state_discretization_;
};

/// @class Action class for the search problem
class ActionSpace {
public:
    // Members
    using StateKey = RobotState;
    using StateHash = PointerValueHash<StateKey>;
    using StateEqual = PointerValueEqual<StateKey>;
    hash_map<StateKey*, int, StateHash, StateEqual> state_to_id_;
    /// @brief The states
    std::vector<RobotState*> states_;

    /// Methods.
    /// @brief Constructor
    explicit ActionSpace() = default;

    /// @brief Destructor
    ~ActionSpace() = default;

    /// @brief Get hash entry for a state. If doesnt exist return nullptr
    /// @param state_id The state id
    /// @return The state pointer
    virtual auto getRobotHashEntry(int state_id) -> RobotState* {
        lock_.lock();
        if (state_id < 0 || state_id >= states_.size()) {
            lock_.unlock();
            return nullptr;
        }
        lock_.unlock();
        return states_[state_id];
    }

    /// @brief Get a state by id
    /// @param id The id of the state
    /// @return The state
    /// @note The id is assumed to be valid - meaning that the state exists in states_
    virtual auto getRobotState(size_t id) -> RobotState* {
        lock_.lock();
        assert(id < states_.size() && id >= 0);
        lock_.unlock();
        return states_[id];
    }

    /// @brief Get a state by state value
    /// @param  state_val The state value
    /// @return The robot state
    /// @note The id is assumed to be valid - meaning that the state exists in states_
    virtual auto getRobotState(const StateType& state_val) -> RobotState* {
        lock_.lock();
        auto* curr_state = new RobotState;
        curr_state->state = state_val;
        auto it = state_to_id_.find(curr_state);
        if (it != state_to_id_.end()) {
            delete curr_state;
            lock_.unlock();
            return states_[it->second];
        } else {
            // Raise an error.
            std::cout << "getRobotState via state_val failed. State not found." << std::endl;
            throw std::runtime_error("getRobotState via state_val failed. State not found.");
            delete curr_state;
            lock_.unlock();
            return nullptr;
        }
    }

    /// @brief Get a state id by state value
    /// @param  state_val The state value
    /// @return The robot state id
    /// @note The id is assumed to be valid - meaning that the state exists in states_
    virtual int getRobotStateId(const StateType& state_val) {
        lock_.lock();
        auto* curr_state = new RobotState;
        curr_state->state = state_val;
        auto it = state_to_id_.find(curr_state);
        if (it != state_to_id_.end()) {
            delete curr_state;
            lock_.unlock();
            return it->second;
        } else {
            // Raise an error.
            std::cout << "getRobotStateId via state_val failed. State not found." << std::endl;
            throw std::runtime_error("getRobotStateId via state_val failed. State not found.");
            delete curr_state;
            lock_.unlock();
            return -1;
        }
    }

    /// @brief Get or create state by state value
    /// @param state_val The state value
    /// @return The state id
    virtual int getOrCreateRobotState(const StateType& state_val) {
        // check if the state exists
        lock_.lock();
        auto* curr_state = new RobotState;
        curr_state->state = state_val;
        auto it = state_to_id_.find(curr_state);
        if (it != state_to_id_.end()) {
            delete curr_state;
            lock_.unlock();
            return it->second;
        } else {
            states_.push_back(curr_state);
            int state_id = (int)states_.size() - 1;
            state_to_id_[curr_state] = state_id;
            lock_.unlock();
            return state_id;
        }
    }

    /// @brief Get actions
    /// @param state_id The state id
    /// @param actions The action sequences
    /// @param check_validity Check if the actions are valid
    virtual void getActions(int state_id,
                            std::vector<ActionSequence>& actions_seq,
                            bool check_validity) = 0;

    /// @brief Get Successor
    /// @param curr_state_ind The current state index
    /// @param seqs_state_ids All states between the current state and the successor, including them. For example, say we have a current state [1,1] and a successor [1,4]. Let's say the edge connecting them is [1,1], [1,2], [1,3], [1,4]. If their state ids are 101, 102, 103, 104, then seqs_state_ids should be [101, 102, 103. 104].
    /// @param seqs_transition_costs The cost of the transitions between all states on the edge. In the example above, the the seqs_transition_costs should be the cost of the transition from [1,1] to [1,2] and [1,2] to [1,3] and [1,3] to [1,4], for example: [1,1,1,0], with each element being the cost of the transition from i to i+1. The last entry therefore is zero since it is the cost of the transition from the last state to an unknown next state.
    /// @return Success bool
    /// @note Beware the you should make sure that the state is discretized! (see ActionType::Discretization)
    /// If you are using an implicit graph where the state space is not discrete then define the discretization
    /// based on the tolerance for comparison between states.
    /// @attention You should use getOrCreateRobotState() and getRobotState() when generating the successors!
    virtual bool getSuccessors(int curr_state_ind,
                               std::vector<std::vector<int>>& seqs_state_ids,
                               std::vector<std::vector<double>>& seqs_transition_costs) = 0;

    [[deprecated("Use the new getSuccessors (for sequences) instead.")]]
    bool getSuccessors(int curr_state_ind,
                       std::vector<int>& successors,
                       std::vector<double>& costs) {
        std::vector<std::vector<int>> seqs_state_ids;
        std::vector<std::vector<double>> seqs_transition_costs;
        bool success = getSuccessors(curr_state_ind, seqs_state_ids, seqs_transition_costs);
        if (success) {
            for (size_t i = 0; i < seqs_state_ids.size(); i++) {
                // Check that the edges have only two elements (the parent and child states). Otherwise, abort with a message.
                if (seqs_state_ids[i].size() != 2) {
                    std::cout << RED << "getSuccessors: The seqs_state_ids[i] should have only two elements (the parent and child states). Instead, it has " << seqs_state_ids[i].size() << " elements." << RESET << std::endl;
                    std::cout << RED << "Edge state ids: " << seqs_state_ids[i] << RESET << std::endl;
                    std::cout << RED << "GetSuccessors would have returned the edge state ids: [" << seqs_state_ids[i].front() << ", " << seqs_state_ids[i].back() << "], which would lose information." << RESET << std::endl;
                    throw std::runtime_error("getSuccessors: The seqs_state_ids should have only two elements (the parent and child states).");
                }

                successors.push_back(seqs_state_ids[i].back());
                costs.push_back(vectorSum(seqs_transition_costs[i]));
            }
        }
        return success;
    }

    /// @brief check if the state is valid
    /// @param state_val The values if the state
    /// @return Validity bool
    virtual bool isStateValid(const StateType& state_val) = 0;

    /// @brief check if the path is valid
    /// @param path The path
    /// @return Validity bool
    virtual bool isPathValid(const PathType& path) = 0;

    /// @brief Clear all the cached states data in the action space.
    virtual void resetPlanningData() {
        for (auto& state : states_) {
            delete state;
        }
        states_.clear();
        state_to_id_.clear();
    }

protected:
    /// @brief  Lock for the action space to make it multi-thread safe
    LockType lock_;
};
}  // namespace ims
