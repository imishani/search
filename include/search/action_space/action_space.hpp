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

#ifndef SEARCH_ACTIONSPACE_HPP
#define SEARCH_ACTIONSPACE_HPP

// standard includes
#include <iostream>
#include <memory>

// project includes
#include "search/common/scene_interface.hpp"
#include "search/common/types.hpp"

namespace ims {

    ///@brief RobotState base class
    struct RobotState {
        StateType state_mapped;
        StateType state;

        enum VALID {valid, invalid, unknown};
        VALID validity = unknown;
    };

    /// @brief equality operator for RobotState
    inline bool operator==(const RobotState &lhs, const RobotState &rhs) {
        return lhs.state == rhs.state;
    }
}

namespace std {
    template<>
    struct hash<ims::RobotState> {
        using argument_type = ims::RobotState;
        using result_type = std::size_t;

        result_type operator()(const argument_type& s) const {
            size_t seed = 0;
            boost::hash_combine(seed, boost::hash_range(s.state.begin(), s.state.end()));
            return seed;
        }
    };
}


namespace ims {

    /// @brief Action type abstract struct. This struct should be inherited by the action type
    /// Make sure to implement the getPrimActions() function
    struct ActionType{

        /// @brief Constructor
        ActionType() = default;

        /// @brief Destructor
        virtual ~ActionType() = default;

        /// @brief Get the possible actions
        /// @return The possible actions
        virtual std::vector<Action> getPrimActions() = 0;

        /// @brief Get the resolution of the state space (for discretization)
        /// @param state_des The state discretization
        virtual void Discretization(StateType& state_des) = 0;

        // The discretization steps for each dimension in the state vector.
        StateType state_discretization_;
    };

    /// @class Action class for the search problem
    class ActionSpace{

    public:

        // Members
        using StateKey = RobotState;
        using StateHash = PointerValueHash<StateKey>;
        using StateEqual = PointerValueEqual<StateKey>;
        hash_map<StateKey*, int, StateHash, StateEqual> state_to_id_;
        /// @brief The states
        std::vector<RobotState*> states_;

        /// Methods
        /// @brief Constructor
        explicit ActionSpace() = default;

        /// @brief Destructor
        ~ActionSpace() = default;

        /// @brief Get hash entry for a state. If doesnt exist return nullptr
        /// @param state_id The state id
        /// @return The state pointer
        virtual auto getRobotHashEntry(int state_id) -> RobotState*{
            if(state_id < 0 || state_id >= states_.size()){
                return nullptr;
            }
            return states_[state_id];
        }

        /// @brief Get a state by id
        /// @param id The id of the state
        /// @return The state
        /// @note The id is assumed to be valid - meaning that the state exists in states_
        virtual auto getRobotState(size_t id) -> RobotState*{
            assert(id < states_.size() && id >= 0);
            return states_[id];
        }

        /// @brief Get or create state by state value
        /// @param state_val The state value
        /// @return The state id
        virtual int getOrCreateRobotState(const StateType& state_val){
            // check if the state exists
            auto* curr_state = new RobotState;
            curr_state->state = state_val;
            auto it = state_to_id_.find(curr_state);
            if(it != state_to_id_.end()){
                delete curr_state;
                return it->second;
            } else {
                states_.push_back(curr_state);
                int state_id = (int)states_.size() - 1;
                state_to_id_[curr_state] = state_id;
                return state_id;
            }
        }

        /// @brief Get actions
        /// @param state_id The state id
        /// @param actions The action sequences
        /// @param check_validity Check if the actions are valid
        virtual void getActions(int state_id,
                                std::vector<ActionSequence> &actions_seq,
                                bool check_validity) = 0;

        /// @brief Get Successor
        /// @param curr_state_ind The current state index
        /// @param successors The successor state
        /// @return Success bool
        /// @note Beware the you should make sure that the state is discretized! (see ActionType::Discretization)
        /// If you are using an implicit graph where the state space is not discrete then define the discretization
        /// based on the tolerance for comparison between states.
        /// @attention You should use getOrCreateRobotState() and getRobotState() when generating the successors!
        virtual bool getSuccessors(int curr_state_ind,
                                   std::vector<int>& successors,
                                   std::vector<double>& costs) = 0;

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
    };
}
#endif //SEARCH_ACTIONSPACE_HPP
