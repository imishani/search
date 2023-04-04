//
// Created by itamar on 4/1/23.
//

#ifndef SEARCH_STDHEURISTICS_HPP
#define SEARCH_STDHEURISTICS_HPP


#include <eigen3/Eigen/Dense>

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

    /// @brief Robot joint angles distance heuristic
    /// @param state1 The first state (state values) type: stateType
    /// @param state2 The second state (state values) type: stateType
    /// @param dist The distance between the two states (output, pass by reference)
    /// @return bool success
    bool jointAnglesHeuristic(const stateType& state1, const stateType& state2, double& dist){
        // make sure to deal with discontinuity in joint angles
        // check id the states are the same size
        if (state1.size() != state2.size()){
            std::cout << "Error: The states are not the same size!" << std::endl;
            return false;
        }
        else{
            dist = 0;
            for (int i {0} ; i < state1.size() ; i++){
                dist += std::min(std::abs(state1[i] - state2[i]), 2*M_PI - std::abs(state1[i] - state2[i]));
            }
            return true;
        }
    }

    /// @brief SE(3) distance heuristic
    /// @param state1 The first state (state values) type: stateType
    /// @param state2 The second state (state values) type: stateType
    /// @param dist The distance between the two states (output, pass by reference)
    /// @return bool success
    bool SE3Heuristic(const stateType& state1, const stateType& state2, double& dist){
        // check id the states are the same size
        if (state1.size() != state2.size()){
            std::cout << "Error: The states are not the same size!" << std::endl;
            return false;
        }
        else{
            // get the position of the states
            Eigen::Vector3d pos1 {state1[0], state1[1], state1[2]};
            Eigen::Vector3d pos2 {state2[0], state2[1], state2[2]};
            // get the orientation of the states
            // Transform from RPY to quaternion
            Eigen::Quaterniond quat1;
            quat1 = Eigen::AngleAxisd(state1[5], Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(state1[4], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(state1[3], Eigen::Vector3d::UnitX());
            Eigen::Quaterniond quat2;
            quat2 = Eigen::AngleAxisd(state2[5], Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(state2[4], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(state2[3], Eigen::Vector3d::UnitX());
            // get the distance between the positions
            dist = (pos1 - pos2).norm();
            // get the distance between the orientations
            dist += 1 - quat1.dot(quat2);
            return true;
        }
    }

}




#endif //SEARCH_STDHEURISTICS_HPP
