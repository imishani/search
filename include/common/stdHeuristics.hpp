//
// Created by itamar on 4/1/23.
//

#ifndef SEARCH_STDHEURISTICS_HPP
#define SEARCH_STDHEURISTICS_HPP


#include <common/types.hpp>
#include <common/state.hpp>

namespace ims {
    /// @brief The standard heuristic functions

    /// @brief The Euclidean distance heuristic
    /// @param state1 The first state (state values) type: stateType
    /// @param state2 The second state (state values) type: stateType
    /// @param dist The distance between the two states (output, pass by reference)
    /// @return bool success
    bool euclideanHeuristic(const stateType& state1, const stateType& state2, double& dist){
        // check id the states are the same size
        if (state1.size() != state2.size()){
            std::cout << "Error: The states are not the same size!" << std::endl;
            return false;
        }
        else{
            for (int i {0} ; i < state1.size() ; i++){
                dist += pow(state1[i] - state2[i], 2);
            }
            dist = sqrt(dist);
            return true;
        }
    }

    /// @brief The Manhattan distance heuristic
    /// @param state1 The first state (state values) type: stateType
    /// @param state2 The second state (state values) type: stateType
    /// @param dist The distance between the two states (output, pass by reference)
    /// @return bool success
    bool manhattanHeuristic(const stateType& state1, const stateType& state2, double& dist){
        // check id the states are the same size
        if (state1.size() != state2.size()){
            std::cout << "Error: The states are not the same size!" << std::endl;
            return false;
        }
        else{
            for (int i {0} ; i < state1.size() ; i++){
                dist += std::abs(state1[i] - state2[i]);
            }
            return true;
        }
    }

    /// @brief The Chebyshev distance heuristic
    /// @param state1 The first state (state values) type: stateType
    /// @param state2 The second state (state values) type: stateType
    /// @param dist The distance between the two states (output, pass by reference)
    /// @return bool success
    bool chebyshevHeuristic(const stateType& state1, const stateType& state2, double& dist){
        // check id the states are the same size
        if (state1.size() != state2.size()){
            std::cout << "Error: The states are not the same size!" << std::endl;
            return false;
        }
        else{
            for (int i {0} ; i < state1.size() ; i++){
                dist = std::max(dist, std::abs(state1[i] - state2[i]));
            }
            return true;
        }
    }

    /// @brief The Octile distance heuristic
    /// @param state1 The first state (state values) type: stateType
    /// @param state2 The second state (state values) type: stateType
    /// @param dist The distance between the two states (output, pass by reference)
    /// @return bool success
    bool octileHeuristic(const stateType& state1, const stateType& state2, double& dist){
        // check id the states are the same size
        if (state1.size() != state2.size()){
            std::cout << "Error: The states are not the same size!" << std::endl;
            return false;
        }
        else{
            for (int i {0} ; i < state1.size() ; i++){
                dist += std::abs(state1[i] - state2[i]);
            }
            dist = sqrt(2) * dist;
            return true;
        }
    }

    /// @brief The Zero distance heuristic
    /// @param state1 The first state (state values) type: stateType
    /// @param state2 The second state (state values) type: stateType
    /// @param dist The distance between the two states (output, pass by reference)
    /// @return bool success
    bool zeroHeuristic(const stateType& state1, const stateType& state2, double& dist){
        // check id the states are the same size
        if (state1.size() != state2.size()){
            std::cout << "Error: The states are not the same size!" << std::endl;
            return false;
        }
        else{
            dist = 0;
            return true;
        }
    }
}




#endif //SEARCH_STDHEURISTICS_HPP
