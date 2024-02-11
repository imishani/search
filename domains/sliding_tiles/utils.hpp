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
 * \file utils.hpp 
 * \author Ram Natarajan (rnataraj@cs.cmu.edu)
 * \date 2/10/24
*/

#ifndef SEARCH_UTILS_HPP
#define SEARCH_UTILS_HPP

// standard includes
#include <fstream>
// library includes
#include "search/common/types.hpp"

bool isActionValid(const StateType& curr_state, const Action& action,
                         int puzzle_size)
{
  MatDf curr_state_mat = Eigen::Map<const MatDf>(curr_state.data(), puzzle_size, puzzle_size);
  Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> mask = (curr_state_mat.array() == 0);
  Eigen::Index row = -1, col = -1;
  mask.maxCoeff(&row, &col);
  switch(static_cast<int>(action[0])) {
    case 0:
      if (row == 0) return false;
      break;
    case 1:
      if (col == puzzle_size-1) return false;
      break;
    case 2:
      if (col == 0) return false;
      break;
    case 3:
      if (row == puzzle_size-1) return false;
      break;
  }
  return true;
}


StateType applyActionToState(const StateType& curr_state, const Action& action,
                             int puzzle_size)
{
  MatDf curr_state_mat = Eigen::Map<const MatDf>(curr_state.data(), puzzle_size, puzzle_size);
  Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> mask = (curr_state_mat.array() == 0);
  Eigen::Index row = -1, col = -1;
  mask.maxCoeff(&row, &col);
  int temp = static_cast<int>(curr_state_mat(row, col));
  switch(static_cast<int>(action[0])) {
    case 0:
      curr_state_mat(row, col) = curr_state_mat(row-1, col);
      curr_state_mat(row-1, col) = temp;
      break;
    case 1:
      curr_state_mat(row, col) = curr_state_mat(row, col+1);
      curr_state_mat(row, col+1) = temp;
      break;
    case 2:
      curr_state_mat(row, col) = curr_state_mat(row, col-1);
      curr_state_mat(row, col-1) = temp;
      break;
    case 3:
      curr_state_mat(row, col) = curr_state_mat(row+1, col);
      curr_state_mat(row+1, col) = temp;
      break;
  }

  StateType next_state_val(curr_state_mat.data(), curr_state_mat.data() + curr_state_mat.size());
  return next_state_val;
}

StateType generateSolvableStartState(StateType& goal, int num_jumble, int puzzle_size)
{
  StateType solvable_state = goal;
  for (int i=0; i < num_jumble;)
  {
    int rand_ind = rand() % static_cast<int>(4);
    Action action = {static_cast<double>(rand_ind)};
    if (isActionValid(solvable_state, action, puzzle_size))
    {
      solvable_state = applyActionToState(solvable_state, action, puzzle_size);
      ++i;
    }
  }

  MatDf solvable_state_mat = Eigen::Map<const MatDf>(solvable_state.data(), puzzle_size, puzzle_size);

  return solvable_state;
}

#endif //SEARCH_UTILS_HPP
