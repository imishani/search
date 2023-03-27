//
// Created by itamar on 3/26/23.
//

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
            m_state = std::move(state);
            g = INF;
            h = -1;
            f = INF;
            state_id = id_counter++;
        }

        /// @brief Destructor
        ~state() = default;

        // Setters
        /// @brief Set the state
        /// @param state The state of the robot
        void setState(const stateType& state) { m_state = state; }

        /// @brief Set the g value
        /// @param g_value The g value
        void setG(double g_value) { g = g_value; }

        /// @brief Set the h value
        /// @param h_value The h value
        void setH(double h_value){ h = h_value; }

        /// @brief Set the f value
        /// @param f_value The f value
        void setF(double f_value){ f = f_value; }

        /// @brief Set parent pointer
        /// @param parent The parent pointer
        void setParent(state* parent) { parentPointer = parent; }

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

        /// @brief Check if the state is closed
        /// @return True if the state is closed, false otherwise
        bool isClosed() const { return flag & CLOSED; }

        /// @brief Check if the state is open
        /// @return True if the state is open, false otherwise
        bool isOpen() const { return flag & OPEN; }

        /// @brief Check if the state is inconsistent
        /// @return True if the state is inconsistent, false otherwise
        bool isIncons() const { return flag & INCONS; }

        /// Reset methods
        /// @brief Reset g value
        void resetG() { g = INF; }

        /// @brief Reset h value
        void resetH() { h = -1; }

        /// @brief Reset f value
        void resetF() { f = INF; }

        /// @brief Reset the flags
        void resetFlags() { state_id &= ~(CLOSED | OPEN | INCONS); }

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


    protected:
        // id's
        int state_id;
        static size_t id_counter;
        // vars
        stateType m_state;
        double g;
        double h;
        double f;
        state* parentPointer{};
        int flag{};
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
        bool operator()(const state* s1, const state* s2) const {
            return s1->getF() < s2->getF();
        }
    };

}


#endif //SEARCH_STATE_HPP
