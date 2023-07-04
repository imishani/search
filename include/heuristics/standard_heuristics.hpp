//
// Created by itamar on 4/13/23.
//

#ifndef SEARCH_STANDARDHEU_HPP
#define SEARCH_STANDARDHEU_HPP


#include <common/base_heuristic.hpp>
#include <common/types.hpp>

#include <eigen3/Eigen/Dense>

/// @brief The standard heuristic functions
namespace ims {

    /// @brief The Euclidean distance heuristic
    struct EuclideanHeuristic : public BaseHeuristic {

        bool getHeuristic(State* s1, State* s2,
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
    struct ManhattanHeuristic : public BaseHeuristic {

        bool getHeuristic(State* s1, State* s2,
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
    struct ChebyshevHeuristic : public BaseHeuristic {

        bool getHeuristic(State* s1, State* s2,
                          double& dist) override {
            // check id the states are the same size
            if (s1->getState().size() != s2->getState().size()) {
                std::cout << "Error: The states are not the same size!" << std::endl;
                return false;
            } else {
                dist = 0;
                for (int i{0}; i < s1->getState().size(); i++) {
                    dist = std::max(dist, std::abs(s1->getState()[i] - s2->getState()[i]));
                }
                return true;
            }
        }
    };

    /// @brief The Minkowski distance heuristic
    struct MinkowskiHeuristic : public BaseHeuristic {

        bool getHeuristic(State* s1, State* s2,
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
    struct OctileHeuristic : public BaseHeuristic {

        bool getHeuristic(State* s1, State* s2,
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
    struct ZeroHeuristic : public BaseHeuristic {

        bool getHeuristic(State* s1, State* s2,
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

        bool getHeuristic(State* s, double& dist) override{
            // check id the states are the same size
            dist = 0;
            return true;
        }
    };

    /// @brief Robot joint angles distance heuristic
    struct JointAnglesHeuristic : public BaseHeuristic {

        bool getHeuristic(State* s1, State* s2,
                          double& dist) override {
            // check id the states are the same size
            if (s1->getState().size() != s2->getState().size()) {
                std::cout << "Error: The states are not the same size!" << std::endl;
                return false;
            } else {
                dist = 0;
                for (int i{0}; i < s1->getState().size(); i++) {
//                    dist += std::min(std::abs(s1->getState()[i] - s2->getState()[i]),
//                                     2*M_PI - std::abs(s1->getState()[i] - s2->getState()[i]));
                    double dj = (s1->getState()[i] - s2->getState()[i]);
                    dist += dj*dj;
                }
                dist = sqrt(dist);
                return true;
            }
        }
    };

    /// @brief SE(3) distance heuristic
    struct SE3HeuristicRPY : public BaseHeuristic {
    
        bool getHeuristic(State* s1, State* s2,
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

    /// @brief SE(3) distance heuristic
    struct SE3HeuristicQuat : public BaseHeuristic {
        bool getHeuristic(State* s1, State* s2,
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
                Eigen::Quaterniond quat1 {s1->getState()[6], s1->getState()[3], s1->getState()[4], s1->getState()[5]};
                Eigen::Quaterniond quat2 {s2->getState()[6], s2->getState()[3], s2->getState()[4], s2->getState()[5]};
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
