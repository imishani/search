//  * Copyright (c) 2024, Ramkumar Natarajan
//  * All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without
//  * modification, are permitted provided that the following conditions are met:
//  *
//  *     * Redistributions of source code must retain the above copyright
//  *       notice, this list of conditions and the following disclaimer.
//  *     * Redistributions in binary form must reproduce the above copyright
//  *       notice, this list of conditions and the following disclaimer in the
//  *       documentation and/or other materials provided with the distribution.
//  *     * Neither the name of the Carnegie Mellon University nor the names of its
//  *       contributors may be used to endorse or promote products derived from
//  *       this software without specific prior written permission.
//  *
//  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
//  * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  * POSSIBILITY OF SUCH DAMAGE.
//

/*!
 * \file heuristics.hpp 
 * \author Ram Natarajan (rnataraj@cs.cmu.edu)
 * \date 2/11/24
*/

#ifndef SEARCH_HEURISTICS_HPP
#define SEARCH_HEURISTICS_HPP

#include "search/heuristics/base_heuristic.hpp"

/// @brief The Manhattan distance heuristic for the sliding puzzle
struct ManhattanDistanceHeuristic : public ims::BaseHeuristic {
  bool getHeuristic(const StateType& s1, const StateType& s2,
                    double& dist) override {
    dist = 0;
    int puzzle_size = static_cast<int>(std::sqrt(s1.size()));
    MatDf s1_mat = Eigen::Map<const MatDf>(s1.data(), puzzle_size, puzzle_size);
    MatDf s2_mat = Eigen::Map<const MatDf>(s2.data(), puzzle_size, puzzle_size);

    for (int i = 0; i < puzzle_size; ++i) {
      for (int j = 0; j < puzzle_size; ++j) {
        int value = s1_mat(i, j);
        if (value != 0) { // Skip the empty tile (represented by 0)
          // Calculate the goal position (row, col) for the current tile value
          Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> mask = (s2_mat.array() == value);
          Eigen::Index row = -1, col = -1;
          mask.maxCoeff(&row, &col);

          // Calculate the Manhattan distance and add it to the heuristic
          dist += std::abs(i - row) + std::abs(j - col);
        }
      }
    }
    return true;
  }
};


/// @brief The Linear Conflict heuristic for the sliding puzzle
struct LinearConflictHeuristic : public ims::BaseHeuristic {
  bool getHeuristic(const StateType& s1, const StateType& s2,
                    double& dist) override {
    dist = 0;
    int puzzle_size = static_cast<int>(std::sqrt(s1.size()));
    MatDf s1_mat = Eigen::Map<const MatDf>(s1.data(), puzzle_size, puzzle_size);
    MatDf s2_mat = Eigen::Map<const MatDf>(s2.data(), puzzle_size, puzzle_size);

    Eigen::Index rowa = -1, cola = -1;
    Eigen::Index rowb = -1, colb = -1;
    for (int i = 0; i < puzzle_size; ++i) {
      for (int j = 0; j < puzzle_size; ++j) {
        int value = s1_mat(i, j);
        if (value != 0) { // Skip the empty tile (represented by 0)
          // Calculate the goal position (row, col) for the current tile value
          Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> mask = (s2_mat.array() == value);
          mask.maxCoeff(&rowa, &cola);

          if (i == rowa && j == cola) {
            // The tile is in the correct position
            continue;
          }
        }

        for (int k=i+1; k<puzzle_size; ++k) {
          value = s1_mat(k, j);
          Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> mask = (s2_mat.array() == value);
          mask.maxCoeff(&rowb, &colb);

          if (j == cola && j == colb) {
            if (rowa > rowb) {
              dist += 1;
            }
          }
        }

        for (int k=j+1; k<puzzle_size; ++k) {
          value = s1_mat(i, k);
          Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> mask = (s2_mat.array() == value);
          mask.maxCoeff(&rowb, &colb);

          if (i == rowa && i == rowb) {
            if (cola > colb) {
              dist += 1;
            }
          }
        }
      }
    }
    return true;
  }
};

/// @brief The Manhattan distance plus Linear Conflict heuristic (admissible) for the sliding puzzle
struct ManhattanPlusLinearConflictHeuristic : public ims::BaseHeuristic {
  bool getHeuristic(const StateType& s1, const StateType& s2,
                    double& dist) override {
    double manhattan_dist = 0;
    double linear_conflict_dist = 0;
    if (manhattan_heuristic.getHeuristic(s1, s2, manhattan_dist) &&
        linear_conflict_heuristic.getHeuristic(s1, s2, linear_conflict_dist)) {
      dist = manhattan_dist + linear_conflict_dist;
    }
    else {
      return false;
    }
    return true;
  }
  ManhattanDistanceHeuristic manhattan_heuristic;
  LinearConflictHeuristic linear_conflict_heuristic;
};


#endif //SEARCH_HEURISTICS_HPP
