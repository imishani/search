//
// Created by itamar on 4/13/23.
//

#ifndef SEARCH_BASEHEURISTIC_HPP
#define SEARCH_BASEHEURISTIC_HPP

#include <common/state.hpp>

namespace ims{
    /// @brief The base heuristic class
    class baseHeuristic {
    public:
        /// @brief Constructor
        baseHeuristic() = default;

        /// @brief Destructor
        virtual ~baseHeuristic(){
            delete mGoal;
        };


        /// @brief This function should be implemented by the user and is the function that calculates the heuristic
        /// @param s1 The first state
        /// @param s2 The second state
        /// @param dist The distance between the two states (output, pass by reference)
        /// @return The heuristic value
        virtual bool getHeuristic(state* s1, state* s2, double& dist) = 0;

        /// @brief An option to calculate the heuristic from a single state to the goal state
        /// @param s The state
        /// @param dist The distance between the two states (output, pass by reference)
        /// @return The heuristic value
        virtual bool getHeuristic(state* s, double& dist) {
            if (mGoal == nullptr) {
                throw std::runtime_error("Goal state is not set");
            }
            return getHeuristic(s, mGoal, dist);
        }

        /// @brief Set the goal state
        /// @param goal The goal state
        /// @note You have to set the goal state if you want to use the getHeuristic(const std::shared_ptr<state> s) function
        void setGoal(state* goal) {
            mGoal = goal;
        }

        state* mGoal = nullptr;

    };

}


#endif //SEARCH_BASEHEURISTIC_HPP
