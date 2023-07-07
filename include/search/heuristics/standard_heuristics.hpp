//
// Created by itamar on 4/13/23.
//

#ifndef SEARCH_STANDARDHEU_HPP
#define SEARCH_STANDARDHEU_HPP


#include <search/common/base_heuristic.hpp>
#include <search/common/types.hpp>

#include <eigen3/Eigen/Dense>

/// @brief The standard heuristic functions
namespace ims {

    /// @brief The Euclidean distance heuristic
    struct EuclideanHeuristic : public BaseHeuristic {

        bool getHeuristic(StateType& s1, StateType& s2,
                          double& dist) override {
            // check id the states are the same size
            if (s1.size() != s2.size()) {
                std::cout << "Error: The states are not the same size!" << std::endl;
                return false;
            } else {
                for (int i{0}; i < s1.size(); i++) {
                    dist += pow(s1[i] - s2[i], 2);
                }
                dist = sqrt(dist);
                return true;
            }
        }
    };

    /// @brief The Manhattan distance heuristic
    struct ManhattanHeuristic : public BaseHeuristic {

        bool getHeuristic(StateType& s1, StateType& s2,
                          double& dist) override {
            // check id the states are the same size
            if (s1.size() != s2.size()) {
                std::cout << "Error: The states are not the same size!" << std::endl;
                return false;
            } else {
                for (int i{0}; i < s1.size(); i++) {
                    dist += std::abs(s1[i] - s2[i]);
                }
                return true;
            }
        }
    };

    /// @brief The Chebyshev distance heuristic
    struct ChebyshevHeuristic : public BaseHeuristic {

        bool getHeuristic(StateType& s1, StateType& s2,
                          double& dist) override {
            // check id the states are the same size
            if (s1.size() != s2.size()) {
                std::cout << "Error: The states are not the same size!" << std::endl;
                return false;
            } else {
                dist = 0;
                for (int i{0}; i < s1.size(); i++) {
                    dist = std::max(dist, std::abs(s1[i] - s2[i]));
                }
                return true;
            }
        }
    };

    /// @brief The Minkowski distance heuristic
    struct MinkowskiHeuristic : public BaseHeuristic {

        bool getHeuristic(StateType& s1, StateType& s2,
                          double& dist) override {
            // check id the states are the same size
            if (s1.size() != s2.size()) {
                std::cout << "Error: The states are not the same size!" << std::endl;
                return false;
            } else {
                for (int i{0}; i < s1.size(); i++) {
                    dist += pow(std::abs(s1[i] - s2[i]), 3);
                }
                dist = pow(dist, 1.0/3.0);
                return true;
            }
        }
    };

    /// @brief The Octile distance heuristic
    struct OctileHeuristic : public BaseHeuristic {

        bool getHeuristic(StateType& s1, StateType& s2,
                          double& dist) override {
            // check id the states are the same size
            if (s1.size() != s2.size()) {
                std::cout << "Error: The states are not the same size!" << std::endl;
                return false;
            } else {
                for (int i{0}; i < s1.size(); i++) {
                    dist += std::abs(s1[i] - s2[i]);
                }
                dist = sqrt(2) * dist;
                return true;
            }
        }
    };

    /// @brief The Zero distance heuristic
    struct ZeroHeuristic : public BaseHeuristic {

        bool getHeuristic(StateType& s1, StateType& s2,
                          double& dist) override {
            // check id the states are the same size
            if (s1.size() != s2.size()) {
                std::cout << "Error: The states are not the same size!" << std::endl;
                return false;
            } else {
                dist = 0;
                return true;
            }
        }

        bool getHeuristic(StateType&, double& dist) override{
            // check id the states are the same size
            dist = 0;
            return true;
        }
    };

    /// @brief Robot joint angles distance heuristic
    struct JointAnglesHeuristic : public BaseHeuristic {

        bool getHeuristic(StateType& s1, StateType& s2,
                          double& dist) override {
            // check id the states are the same size
            if (s1.size() != s2.size()) {
                std::cout << "Error: The states are not the same size!" << std::endl;
                return false;
            } else {
                dist = 0;
                for (int i{0}; i < s1.size(); i++) {
//                    dist += std::min(std::abs(s1[i] - s2[i]),
//                                     2*M_PI - std::abs(s1[i] - s2[i]));
                    double dj = (s1[i] - s2[i]);
                    dist += dj*dj;
                }
                dist = sqrt(dist);
                return true;
            }
        }
    };

    /// @brief SE(3) distance heuristic
    struct SE3HeuristicRPY : public BaseHeuristic {
    
        bool getHeuristic(StateType& s1, StateType& s2,
                          double& dist) override {
            // check id the states are the same size
            if (s1.size() != s2.size()) {
                std::cout << "Error: The states are not the same size!" << std::endl;
                return false;
            } else {
                // get the position of the states
                Eigen::Vector3d pos1 {s1[0], s1[1], s1[2]};
                Eigen::Vector3d pos2 {s2[0], s2[1], s2[2]};
                // get the orientation of the states
                // Transform from RPY to quaternion
                Eigen::Quaterniond quat1 = Eigen::AngleAxisd(s1[5], Eigen::Vector3d::UnitZ())
                                           * Eigen::AngleAxisd(s1[4], Eigen::Vector3d::UnitY())
                                           * Eigen::AngleAxisd(s1[3], Eigen::Vector3d::UnitX());
                Eigen::Quaterniond quat2 = Eigen::AngleAxisd(s2[5], Eigen::Vector3d::UnitZ())
                                             * Eigen::AngleAxisd(s2[4], Eigen::Vector3d::UnitY())
                                             * Eigen::AngleAxisd(s2[3], Eigen::Vector3d::UnitX());
                // get the distance between the positions
                dist = (pos1 - pos2).norm();
                // get the distance between the orientations
                dist += 2 * std::acos(std::min(1.0, std::abs(quat1.dot(quat2))));
                return true;
            }
        }
    };

    /// @brief SE(3) distance heuristic
    struct SE3HeuristicQuat : public BaseHeuristic {
        bool getHeuristic(StateType& s1, StateType& s2,
                          double& dist) override {
            // check id the states are the same size
            if (s1.size() != s2.size()) {
                std::cout << "Error: The states are not the same size!" << std::endl;
                return false;
            } else {
                // get the position of the states
                Eigen::Vector3d pos1 {s1[0], s1[1], s1[2]};
                Eigen::Vector3d pos2 {s2[0], s2[1], s2[2]};
                // get the orientation of the states
                // Transform from RPY to quaternion
                Eigen::Quaterniond quat1 {s1[6], s1[3], s1[4], s1[5]};
                Eigen::Quaterniond quat2 {s2[6], s2[3], s2[4], s2[5]};
                // get the distance between the positions
                dist = (pos1 - pos2).norm();
                // get the distance between the orientations
                dist += 2 * std::acos(std::min(1.0, std::abs(quat1.dot(quat2))));
                return true;
            }
        }
    };

}


#endif //SEARCH_STANDARDHEU_HPP
