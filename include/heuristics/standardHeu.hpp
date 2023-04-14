//
// Created by itamar on 4/13/23.
//

#ifndef SEARCH_STANDARDHEU_HPP
#define SEARCH_STANDARDHEU_HPP


#include <common/baseHeuristic.hpp>
#include <common/types.hpp>

#include <eigen3/Eigen/Dense>

/// @brief The standard heuristic functions
namespace ims {

    /// @brief The Euclidean distance heuristic
    class euclideanHeuristic : public baseHeuristic {

        bool getHeuristic(state* s1, state* s2,
                          double& dist) override {
            // check id the states are the same size
            if (s1->getState().size() != s2->getState().size()) {
                std::cout << "Error: The states are not the same size!" << std::endl;
                return false;
            } else {
                for (int i{0}; i < s1->getState().size(); i++) {
                    dist += pow(s1->getState()[i] - s2->getState()[i], 2);
                }
                dist = sqrt(dist);
                return true;
            }
        }
    };

    /// @brief The Manhattan distance heuristic
    class manhattanHeuristic : public baseHeuristic {

        bool getHeuristic(state* s1, state* s2,
                          double& dist) override {
            // check id the states are the same size
            if (s1->getState().size() != s2->getState().size()) {
                std::cout << "Error: The states are not the same size!" << std::endl;
                return false;
            } else {
                for (int i{0}; i < s1->getState().size(); i++) {
                    dist += std::abs(s1->getState()[i] - s2->getState()[i]);
                }
                return true;
            }
        }
    };

    /// @brief The Chebyshev distance heuristic
    class chebyshevHeuristic : public baseHeuristic {

        bool getHeuristic(state* s1, state* s2,
                          double& dist) override {
            // check id the states are the same size
            if (s1->getState().size() != s2->getState().size()) {
                std::cout << "Error: The states are not the same size!" << std::endl;
                return false;
            } else {
                for (int i{0}; i < s1->getState().size(); i++) {
                    dist = std::max(dist, std::abs(s1->getState()[i] - s2->getState()[i]));
                }
                return true;
            }
        }
    };

    /// @brief The Minkowski distance heuristic
    class minkowskiHeuristic : public baseHeuristic {

        bool getHeuristic(state* s1, state* s2,
                          double& dist) override {
            // check id the states are the same size
            if (s1->getState().size() != s2->getState().size()) {
                std::cout << "Error: The states are not the same size!" << std::endl;
                return false;
            } else {
                for (int i{0}; i < s1->getState().size(); i++) {
                    dist += pow(std::abs(s1->getState()[i] - s2->getState()[i]), 3);
                }
                dist = pow(dist, 1.0/3.0);
                return true;
            }
        }
    };

    /// @brief The Octile distance heuristic
    class octileHeuristic : public baseHeuristic {

        bool getHeuristic(state* s1, state* s2,
                          double& dist) override {
            // check id the states are the same size
            if (s1->getState().size() != s2->getState().size()) {
                std::cout << "Error: The states are not the same size!" << std::endl;
                return false;
            } else {
                for (int i{0}; i < s1->getState().size(); i++) {
                    dist += std::abs(s1->getState()[i] - s2->getState()[i]);
                }
                dist = sqrt(2) * dist;
                return true;
            }
        }
    };

    /// @brief The Zero distance heuristic
    class zeroHeuristic : public baseHeuristic {

        bool getHeuristic(state* s1, state* s2,
                          double& dist) override {
            // check id the states are the same size
            if (s1->getState().size() != s2->getState().size()) {
                std::cout << "Error: The states are not the same size!" << std::endl;
                return false;
            } else {
                dist = 0;
                return true;
            }
        }

        bool getHeuristic(state* s, double& dist) override{
            // check id the states are the same size
            dist = 0;
            return true;
        }
    };

    /// @brief Robot joint angles distance heuristic
    class jointAnglesHeuristic : public baseHeuristic {

        bool getHeuristic(state* s1, state* s2,
                          double& dist) override {
            // check id the states are the same size
            if (s1->getState().size() != s2->getState().size()) {
                std::cout << "Error: The states are not the same size!" << std::endl;
                return false;
            } else {
                dist = 0;
                for (int i{0}; i < s1->getState().size(); i++) {
                    dist += std::min(std::abs(s1->getState()[i] - s2->getState()[i]),
                                     2*M_PI - std::abs(s1->getState()[i] - s2->getState()[i]));
                }
                return true;
            }
        }
    };

    /// @brief SE(3) distance heuristic
    class SE3Heuristic : public baseHeuristic {

        bool getHeuristic(state* s1, state* s2,
                          double& dist) override {
            // check id the states are the same size
            if (s1->getState().size() != s2->getState().size()) {
                std::cout << "Error: The states are not the same size!" << std::endl;
                return false;
            } else {
                // get the position of the states
                Eigen::Vector3d pos1 {s1->getState()[0], s1->getState()[1], s1->getState()[2]};
                Eigen::Vector3d pos2 {s2->getState()[0], s2->getState()[1], s2->getState()[2]};
                // get the orientation of the states
                // Transform from RPY to quaternion
                Eigen::Quaterniond quat1 = Eigen::AngleAxisd(s1->getState()[5], Eigen::Vector3d::UnitZ())
                                           * Eigen::AngleAxisd(s1->getState()[4], Eigen::Vector3d::UnitY())
                                           * Eigen::AngleAxisd(s1->getState()[3], Eigen::Vector3d::UnitX());
                Eigen::Quaterniond quat2 = Eigen::AngleAxisd(s2->getState()[5], Eigen::Vector3d::UnitZ())
                                             * Eigen::AngleAxisd(s2->getState()[4], Eigen::Vector3d::UnitY())
                                             * Eigen::AngleAxisd(s2->getState()[3], Eigen::Vector3d::UnitX());
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
