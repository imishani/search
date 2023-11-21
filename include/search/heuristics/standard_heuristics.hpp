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
 * \file   astar.hpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   4/13/23
 */

#ifndef SEARCH_STANDARDHEU_HPP
#define SEARCH_STANDARDHEU_HPP

#include <eigen3/Eigen/Dense>
#include "base_heuristic.hpp"
#include <search/common/types.hpp>
#include <search/planners/wastar.hpp>
#include <utility>

/// @brief The standard heuristic functions
namespace ims {

/// @brief The Euclidean distance heuristic
struct EuclideanHeuristic : public BaseHeuristic {
    bool getHeuristic(const StateType& s1, const StateType& s2,
                      double& dist) override {
        // check id the states are the same size
        if (s1.size() != s2.size()) {
            std::cout << "Error: The states are not the same size!" << std::endl;
            return false;
        }
        else {
            for (int i{0}; i < s1.size(); i++) {
                dist += pow(s1[i] - s2[i], 2);
            }
            dist = sqrt(dist);
            return true;
        }
    }
};

/// @brief The Euclidean distance heuristic with time removed from state
struct EuclideanRemoveTimeHeuristic : public EuclideanHeuristic {
    bool getHeuristic(const StateType& s1, const StateType& s2,
                      double& dist) override {
        StateType s1_no_time = s1;
        StateType s2_no_time = s2;
        s1_no_time.pop_back();
        s2_no_time.pop_back();
          
        return EuclideanHeuristic::getHeuristic(s1_no_time, s2_no_time, dist);
    }
};

/// @brief The Euclidean distance heuristic with theta removed from state
struct EuclideanRemoveThetaHeuristic : public EuclideanHeuristic {
    bool getHeuristic(const StateType& s1, const StateType& s2,
                      double& dist) override {
        StateType s1_no_theta = s1;
        StateType s2_no_theta = s2;
        s1_no_theta.pop_back();
        s2_no_theta.pop_back();

        return EuclideanHeuristic::getHeuristic(s1_no_theta, s2_no_theta, dist);
    }
};

/// @brief The Manhattan distance heuristic
struct ManhattanHeuristic : public BaseHeuristic {
    bool getHeuristic(const StateType& s1, const StateType& s2,
                      double& dist) override {
        // check id the states are the same size
        if (s1.size() != s2.size()) {
            std::cout << "Error: The states are not the same size!" << std::endl;
            return false;
        }
        else {
            for (int i{0}; i < s1.size(); i++) {
                dist += std::abs(s1[i] - s2[i]);
            }
            return true;
        }
    }
};

/// @brief The Chebyshev distance heuristic
struct ChebyshevHeuristic : public BaseHeuristic {
    bool getHeuristic(const StateType& s1, const StateType& s2,
                      double& dist) override {
        // check id the states are the same size
        if (s1.size() != s2.size()) {
            std::cout << "Error: The states are not the same size!" << std::endl;
            return false;
        }
        else {
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
    bool getHeuristic(const StateType& s1, const StateType& s2,
                      double& dist) override {
        // check id the states are the same size
        if (s1.size() != s2.size()) {
            std::cout << "Error: The states are not the same size!" << std::endl;
            return false;
        }
        else {
            for (int i{0}; i < s1.size(); i++) {
                dist += pow(std::abs(s1[i] - s2[i]), 3);
            }
            dist = pow(dist, 1.0 / 3.0);
            return true;
        }
    }
};

/// @brief The Octile distance heuristic
struct OctileHeuristic : public BaseHeuristic {
    bool getHeuristic(const StateType& s1, const StateType& s2,
                      double& dist) override {
        // check id the states are the same size
        if (s1.size() != s2.size()) {
            std::cout << "Error: The states are not the same size!" << std::endl;
            return false;
        }
        else {
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
    bool getHeuristic(const StateType& s1, const StateType& s2,
                      double& dist) override {
        // check id the states are the same size
        if (s1.size() != s2.size()) {
            std::cout << "Error: The states are not the same size!" << std::endl;
            return false;
        }
        else {
            dist = 0;
            return true;
        }
    }

    bool getHeuristic(const StateType&, double& dist) override {
        // check id the states are the same size
        dist = 0;
        return true;
    }
};

/// @brief Robot joint angles distance heuristic
struct JointAnglesHeuristic : public BaseHeuristic {
    explicit JointAnglesHeuristic(const std::vector<bool>& valid_mask = std::vector<bool>()): valid_mask_(valid_mask) {}

    bool getHeuristic(const StateType& s1, const StateType& s2,
                      double& dist) override {
        // check id the states are the same size
        if (s1.size() != s2.size()) {
            std::cout << "Error: The states are not the same size!" << std::endl;
            return false;
        }
        else {
            dist = 0;
            for (int i{0}; i < s1.size(); i++) {
                //                    dist += std::min(std::abs(s1[i] - s2[i]),
                //                                     2*M_PI - std::abs(s1[i] - s2[i]));
                if (!valid_mask_.empty()) {
                    if (!valid_mask_[i]){
                        continue;
                    }
                }
                
                double dj = (s1[i] - s2[i]);
                dist += dj * dj;
            }
            dist = sqrt(dist);
            return true;
        }
    }

private:
    /// @brief The mask of valid joints, any indices marked false will not be used in the computation.
    std::vector<bool> valid_mask_;
};

/// @brief SE(3) distance heuristic
struct SE3HeuristicRPY : public BaseHeuristic {
    bool getHeuristic(const StateType& s1, const StateType& s2,
                      double& dist) override {
        // check id the states are the same size
        if (s1.size() != s2.size()) {
            std::cout << "Error: The states are not the same size!" << std::endl;
            return false;
        }
        else {
            // get the position of the states
            Eigen::Vector3d pos1{s1[0], s1[1], s1[2]};
            Eigen::Vector3d pos2{s2[0], s2[1], s2[2]};
            // get the orientation of the states
            // Transform from RPY to quaternion
            Eigen::Quaterniond quat1 = Eigen::AngleAxisd(s1[5], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(s1[4], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(s1[3], Eigen::Vector3d::UnitX());
            Eigen::Quaterniond quat2 = Eigen::AngleAxisd(s2[5], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(s2[4], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(s2[3], Eigen::Vector3d::UnitX());
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
    bool getHeuristic(const StateType& s1, const StateType& s2,
                      double& dist) override {
        // check id the states are the same size
        if (s1.size() != s2.size()) {
            std::cout << "Error: The states are not the same size!" << std::endl;
            return false;
        }
        else {
            // get the position of the states
            Eigen::Vector3d pos1{s1[0], s1[1], s1[2]};
            Eigen::Vector3d pos2{s2[0], s2[1], s2[2]};
            // get the orientation of the states
            // Transform from RPY to quaternion
            Eigen::Quaterniond quat1{s1[6], s1[3], s1[4], s1[5]};
            Eigen::Quaterniond quat2{s2[6], s2[3], s2[4], s2[5]};
            // get the distance between the positions
            dist = (pos1 - pos2).norm();
            // get the distance between the orientations
            dist += 2 * std::acos(std::min(1.0, std::abs(quat1.dot(quat2))));
            return true;
        }
    }
};

/// @brief Local Heuristic using Dijkstra's Algorithm
struct LocalHeuristic : public BaseHeuristic {

    // Constructor
    LocalHeuristic(std::shared_ptr<ActionSpace> action_space_ptr,
                   BaseHeuristic* heuristic, double distance) : action_space_ptr_(std::move(action_space_ptr)),
                                                                base_heuristic_(heuristic),
                                                                distance_(distance) {}

    //custom dijkstra class to handle escaping a local region
    struct MyAStar : public wAStar {

        double goal_distance;
        StateType goal_center;

        MyAStar(StateType goal,
                const wAStarParams& params,
                double distance) : goal_distance(distance),
                                   goal_center(std::move(goal)),
                                   wAStar(params) {}

        bool isGoalState(int state_id) override {
            StateType cur_state = action_space_ptr_->getRobotState(state_id)->state;
            for (int i = 0; i < cur_state.size(); i++)
            {
                if (fabs(cur_state[i]-goal_center[i])>goal_distance)
                {
                    return true;
                }
            }
            return false;
        }
    };

    //base_heuristic_ that uses above dijkstra class
    bool getHeuristic(const StateType& s1, const StateType& s2,
                      double& dist) override {
        // check if the states are the same size
        if (s1.size() != s2.size()) {
            std::cout << "Error: The states are not the same size!" << std::endl;
            return false;
        } else {
            wAStarParams params(base_heuristic_, 1.0);
            // construct planner
            MyAStar planner(s1, params, distance_);

            planner.initializePlanner(action_space_ptr_, s1, s2);
            for (int i = 0; i < s1.size(); i++)
            {
                if (fabs(s1[i]-s2[i])<planner.goal_distance)
                {
                    base_heuristic_->getHeuristic(s1, s2, dist);
                    return true;
                }
            }
            std::vector<StateType> path;
            bool success = planner.plan(path);
            if (success)
            {
                dist = planner.reportStats().cost;
                double euclidDist = 0;
                StateType temp_state = path[path.size()-1];
                base_heuristic_->getHeuristic(temp_state, s2, euclidDist);
                dist += euclidDist;
                return true;
            }
            else
            {
                std::cout << "Error: Dijkstra's algorithm failed!" << std::endl;
                return false;
            }
        }
    }
    std::shared_ptr<ActionSpace> action_space_ptr_;
    BaseHeuristic* base_heuristic_;
    double distance_;
};

}  // namespace ims

#endif  // SEARCH_STANDARDHEU_HPP
