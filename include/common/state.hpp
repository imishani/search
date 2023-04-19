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
 * \file   state.hpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   3/28/23
*/


#ifndef SEARCH_STATE_HPP
#define SEARCH_STATE_HPP

// standard includes
#include <utility>
#include <iostream>

// project includes
#include "common/instrusive_heap.h"
#include "common/types.hpp"

namespace ims{
    /// @class State class for the search problem
    /// @note This class can both be used and extended by inheriting from it
    class state : public smpl::heap_element{
    public:

        /// @brief Constructor
        /// @param state The state of the robot
        explicit state(stateType state=stateType()){
            flag = -1;
            m_state = std::move(state);
            parentInd = INVALID;
            g = INF;
            h = -1;
            f = INF;
            state_id = id_counter++;
        }

        /// @brief Destructor
        ~state() {
            id_counter--;
        };

        // Setters
        /// @brief Set the state
        /// @param state The state of the robot
        void setState(const stateType& state) { m_state = state; }

        void setUndiscretizedState(const stateType& state) { m_state_undiscretized = state; }

        void setMappedState(const stateType& state) { m_state_map = state; }

        /// @{ In general, it is better to directly set the g, h, and f values rather than
        /// calling these functions since it requires additional code to run,
        /// but they are provided for convenience.

        /// @brief Set the g value
        /// @param g_value The g value
        void setG(double g_value) { g = g_value; }

        /// @brief Set the h value
        /// @param h_value The h value
        void setH(double h_value){ h = h_value; }

        /// @brief Set the f value
        /// @param f_value The f value
        void setF(double f_value){ f = f_value; }

        /// @}

        /// @brief Set parent pointer
        /// @param parent The parent pointer
        void setParent(int parentInd_) {
            parentInd = parentInd_; }

        /// @brief Set as closed
        void setClosed() { flag = CLOSED; }

        /// @brief Set as open
        void setOpen() { flag = OPEN; }

        /// @brief Set as inconsistent
        void setIncons() { flag = INCONS; }

        // Getters
        /// @brief Get the state
        /// @return The state of the robot
        const stateType& getState() const { return m_state; }

        const stateType& getUndiscretizedState() const { return m_state_undiscretized; }

        const stateType& getMappedState() const { return m_state_map; }

        /// @brief Get the g value
        /// @return The g value
        double getG() const { return g; }

        /// @brief Get the h value
        /// @return The h value
        double getH() const { return h; }

        /// @brief Get the f value
        /// @return The f value
        double getF() const { return f; }

        /// @brief Get the state id
        /// @return The state id
        int getStateId() const { return state_id; }

        /// @brief Get the parent index
        /// @return The parent index
        int getParentInd() const { return parentInd; }

        /// @brief Check if the state is closed
        /// @return True if the state is closed, false otherwise
        bool isClosed() const { return flag == CLOSED; }

        /// @brief Check if the state is open
        /// @return True if the state is open, false otherwise
        bool isOpen() const { return flag == OPEN; }

        /// @brief Check if the state is inconsistent
        /// @return True if the state is inconsistent, false otherwise
        bool isIncons() const { return flag == INCONS; }

        /// Reset methods
        /// @brief Reset g value
        void resetG() { g = INF; }

        /// @brief Reset h value
        void resetH() { h = -1; }

        /// @brief Reset f value
        void resetF() { f = INF; }

        /// @brief Reset the flags
        void resetFlags() { state_id &= ~(CLOSED | OPEN | INCONS); } // TODO: Check this

        /// @brief Reset the id counter
        static void resetIdCounter() { id_counter = 0; }

        /// @brief Get the id counter
        static int getIdCounter() { return id_counter; }

        /// @brief Reset everything
        void reset() {
            resetG();
            resetH();
            resetF();
            resetFlags();
        }


        /// @brief Print the state
        void print() const {
            std::cout << "state_id: " << state_id << std::endl;
            std::cout << "State: " << std::endl;
            for (auto& i : m_state){
                std::cout << "  " << i << ", ";
            }
            std::cout << "g: " << g << std::endl;
            std::cout << "h: " << h << std::endl;
            std::cout << "f: " << f << std::endl;
            std::cout << "flags: " << "Closed? " << isClosed() << ", ";
            std::cout << "Open? " << isOpen() << ", ";
            std::cout << "Inconsistent? " << isIncons() << std::endl;

        }

        double g;
        double h;
        double f; // Is it necessary to store it here?

    protected:
        // id's
        int state_id;
        static int id_counter;
        // vars
        stateType m_state;
        stateType m_state_undiscretized;
        stateType m_state_map;
        int parentInd;
        int flag;
        // flag it as closed or not
        enum Flags {
            CLOSED = 1,
            OPEN = 2,
            INCONS = 3
        };


    };

    /// @struct State comparator
    struct stateCompare{
        /// @brief Compare two states
        /// @param s1 The first state
        /// @param s2 The second state
        /// @return True if s1 is less than s2, false otherwise
        bool operator() (const state& s1, const state& s2) {
            // tie breaking. TODO: do i need this?
            if (s1.f == s2.f){
                return s1.getStateId() < s2.getStateId();
            } else
            return s1.f < s2.f;
        };
    };

    inline
    bool operator==(const state& a, const state& b)
    {
        return a.getState() == b.getState();
    }

}


namespace std {
    template <>
    struct hash<ims::state>
    {
        typedef ims::state argument_type;
        typedef std::size_t result_type;
        result_type operator()(const argument_type& s) const;
    };
    inline
    auto hash<ims::state>::operator()(const argument_type& s) const -> result_type
    {
        size_t seed = 0;
        boost::hash_combine(seed, boost::hash_range(s.getState().begin(), s.getState().end()));
        return seed;
    }

} // namespace std


#endif //SEARCH_STATE_HPP
