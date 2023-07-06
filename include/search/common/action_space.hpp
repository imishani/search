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
#include <memory>

// project includes
#include <search/common/state.hpp>
#include <search/common/scene_interface.hpp>

//auto std::hash<ims::State>::operator()(
//        const argument_type& s) const -> result_type
//{
//    size_t seed = 0;
//    boost::hash_combine(seed, boost::hash_range(s.getState().begin(), s.getState().end()));
//    return seed;
//}

namespace ims{
    /// @brief Action type abstract struct. This struct should be inherited by the action type
    /// Make sure to implement the getActions() function
    struct ActionType{
        /// @brief Constructor
        ActionType() = default;

        /// @brief Destructor
        virtual ~ActionType() = default;

        /// @brief Get the possible actions
        /// @return The possible actions
        virtual std::vector<Action> getActions() = 0;

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
        /// @brief The states
        std::vector<State*> states_;
        using StateKey = State;
        using StateHash = PointerValueHash<StateKey>;
        using StateEqual = PointerValueEqual<StateKey>;
        hash_map<StateKey*, int, StateHash, StateEqual> state_to_id_;

        /// Methods
        /// @brief Constructor
        explicit ActionSpace() = default;

        /// @brief Destructor
        ~ActionSpace() = default;

        /// @brief Get a state by id
        /// @param id The id of the state
        /// @return The state
        /// @note The id is assumed to be valid - meaning that the state exists in states_
        virtual auto getState(size_t id) -> State*{
            assert(id < states_.size() && id >= 0);
            return states_[id];
        }

        /// @brief Get or create state by state value
        /// @param state_val The state value
        /// @return The state id
        virtual int getOrCreateState(const StateType& state_val){
            // check if the state exists
            auto* curr_state = new State(state_val);
            auto it = state_to_id_.find(curr_state);
            if(it != state_to_id_.end()){
                delete curr_state;
                return it->second;
            }
            curr_state->setStateId((int)states_.size());
            states_.push_back(curr_state);
            state_to_id_[curr_state] = curr_state->getStateId();
            return curr_state->getStateId();
        }


        /// @brief Get Successor
        /// @param curr_state_ind The current state index
        /// @param successors The successor state
        /// @return Success bool
        /// @note Beware the you should make sure that the state is discretized! (see ActionType::Discretization)
        /// If you are using an implicit graph where the state space is not discrete the define the discretization
        /// based on the tolerance for comparison between states.
        /// @attention You should use getOrCreateState() and getState() when generating the successors!
        virtual bool getSuccessors(int curr_state_ind,
                                   std::vector<State*>& successors,
                                   std::vector<double>& costs) = 0;

        /// @brief check if the state is valid
        /// @param state_val The values if the state
        /// @return Validity bool
        virtual bool isStateValid(const StateType& state_val) = 0;

        /// @brief check if the path is valid
        /// @param path The path
        /// @return Validity bool
        virtual bool isPathValid(const PathType& path) = 0;
    };
}
#endif //SEARCH_ACTIONSPACE_HPP
