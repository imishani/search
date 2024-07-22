/*
 * Copyright (C) 2023, Yorai Shaoul
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
 * \file   utils.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   2024-01-18
*/

#pragma once

// Standard includes.
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <numeric>

/// @brief Pretty print a vector of doubles into a stream.
/// @param stream The stream to print to.
/// @param vec The vector to print.
/// @return The stream.
/// @note example usage:
/// @code
/// std::vector<double> vec = {1, 2, 3};
/// std::cout << vec << std::endl;
/// @endcode
template <typename T>
inline std::ostream& operator<<(std::ostream& stream, const std::vector<T>& vec) {
    if (vec.empty()) {
        stream << "[]";
        return stream;
    }
    stream << "[";
    for (size_t i = 0; i < vec.size() - 1; ++i) {
        stream << vec[i];
        if (i < vec.size() - 1) {
            stream << ", ";
        }
    }
    stream << vec.back() << "]";
    return stream;
}

/// @brief Sum of elements in a vector.
inline double vectorSum(const std::vector<double>& vec) {
    return std::accumulate(vec.begin(), vec.end(), 0.0);
}
inline int vectorSum(const std::vector<int>& vec) {
    return std::accumulate(vec.begin(), vec.end(), 0);
}
inline int vectorSum(const std::vector<bool>& vec) {
    return std::accumulate(vec.begin(), vec.end(), 0);
}

/// @brief Given a path made up of sequences, each connecting a parent to a child with some intermediate states, convert it to a single path.
/// \param seq_states_to_child The path dividied to sequences. The structure of each sequence in the path is [state, intermediate states, child]. Note that the child sequence of this one will begin with 'child.'
/// \param seq_transition_costs_to_child The transition costs within each sequence. For example, [1.0, 1.0, 0.0]
/// means that the transition from the state to the first intermediate state costs 1.0, the transition from the state
/// to the first intermediate state costs 1.0, the transition from the last intermediate state to the child state costs
/// 1.0, and the cost to leave the child is unknown and is therefore zero.
/// \param path Concatenation of all the states without trivial duplicates.
/// \param transition_costs Concatenation of all the corresponding transition costs.
inline void flattenSequencePathToPathType(const std::vector<PathType>& seq_states_to_child,
                                          const std::vector<std::vector<double>>& seq_transition_costs_to_child,
                                          PathType& path,
                                          std::vector<double>& transition_costs) {
    path.clear();
    transition_costs.clear();
    for (size_t i {0}; i < seq_states_to_child.size(); ++i){
        path.insert(path.end(),
                    seq_states_to_child[i].begin(),
                    seq_states_to_child[i].end() - 1);
        transition_costs.insert(transition_costs.end(),
                                seq_transition_costs_to_child[i].begin(),
                                seq_transition_costs_to_child[i].end() - 1);
    }
    // Add the last element of the last sequence.
    path.push_back(seq_states_to_child.back().back());
    transition_costs.push_back(seq_transition_costs_to_child.back().back());
    assert(path.size() == transition_costs.size());
}
inline void flattenSeqPathToPathType(const std::vector<PathType>& seq_states_to_child,
                                     PathType& path) {
    path.clear();
    for (size_t i {0}; i < seq_states_to_child.size(); ++i){
        path.insert(path.end(),
                    seq_states_to_child[i].begin(),
                    seq_states_to_child[i].end() - 1);
    }
    // Add the last element of the last sequence.
    path.push_back(seq_states_to_child.back().back());
}
inline void flattenMultiAgentSeqPathsToMultiAgentPaths(const MultiAgentSeqPaths & seq_states_to_child,
                                          MultiAgentPaths& paths) {
    for (const std::pair<int, std::vector<PathType>>& agent_seq : seq_states_to_child){
        flattenSeqPathToPathType(agent_seq.second, paths[agent_seq.first]);
    }
}
inline void flattenSequencePathsToMultiAgentPaths(const std::unordered_map<int, std::vector<PathType>>& seq_states_to_child,
                                          const std::unordered_map<int, std::vector<std::vector<double>>>& seq_transition_costs_to_child,
                                          MultiAgentPaths& paths,
                                          std::unordered_map<int, std::vector<double>> & paths_transition_costs) {
    for (const std::pair<int, std::vector<PathType>>& agent_seq : seq_states_to_child){
        flattenSequencePathToPathType(agent_seq.second,
                                      seq_transition_costs_to_child.at(agent_seq.first),
                                      paths[agent_seq.first],
                                      paths_transition_costs[agent_seq.first]);
    }
}

/// @brief Given a path made up of sequences, each connecting a parent to a child with some intermediate states (all with the last element being their time),
/// assign the time component of the last element in the last sequence to be the cost of the entire path.
/// \param seq_path
/// \param seq_path_transition_costs
inline void setLastStateTimeInTimedSequencePath(SeqPathType & seq_path, const SeqPathTransitionCostsType & seq_path_transition_costs){
    double total_cost = 0;
    for (size_t i {0}; i < seq_path_transition_costs.size(); ++i){
        total_cost += vectorSum(seq_path_transition_costs[i]);
    }
    seq_path.back().back().back() = total_cost;
}

inline double computeTotalCostFromSeqPathsTransitionCosts(const SeqPathTransitionCostsType & seq_path_transition_costs){
    double total_costs = 0;
    for (const std::vector<double>& transition_costs : seq_path_transition_costs){
        total_costs += vectorSum(transition_costs);
    }
    return total_costs;
}

inline double computeTotalCostFromMultiAgentSeqPathsTransitionCosts(const MultiAgentSeqPathsTransitionCosts & seq_path_transition_costs){
    double total_costs = 0;
    for (const std::pair<int, SeqPathTransitionCostsType>& agent_seq_path_transition_costs : seq_path_transition_costs){
        total_costs += computeTotalCostFromSeqPathsTransitionCosts(agent_seq_path_transition_costs.second);
    }
    return total_costs;
}

inline bool isPathStatesAllEqual(const PathType & path){
    for (size_t i {1}; i < path.size(); ++i){
        if (path[i] != path[i-1]){
            return false;
        }
    }
    return true;
}

inline void removeTimeFromPath(PathType & path){
    for (StateType& state : path){
        state.pop_back();
    }
}

template <typename K, typename V>
inline std::vector<K> getMapKeys(const std::unordered_map<K, V>& map){
    std::vector<K> keys;
    keys.reserve(map.size());
    for (std::pair<const K, V> pair : map){
        keys.push_back(pair.first);
    }
    return keys;
}