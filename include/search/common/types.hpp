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
 * \file   types.hpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   3/28/23
*/
#pragma once

#include <mutex>
#include <vector>
#include <limits>
#include <unordered_map>
#include <boost/functional/hash.hpp>

// Constants
const double INF_DOUBLE = std::numeric_limits<double>::infinity();
const int INF_INT = std::numeric_limits<int16_t>::max();
const double PI = 3.14159265358979323846;
// Colors for printing
#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */

// Typedefs
using StateType = std::vector<double>;
using MultiAgentStateType = std::unordered_map<int, StateType>;
using PathType = std::vector<StateType>;
using SeqPathType = std::vector<std::vector<StateType>>;
using SeqPathTransitionCostsType = std::vector<std::vector<double>>;
using MultiAgentSeqPaths = std::map<int, SeqPathType>;
using MultiAgentSeqPathsTransitionCosts = std::map<int, SeqPathTransitionCostsType>;
/// @brief An object for mapping [agent_ids][timestamp] to a state.
using MultiAgentPaths = std::unordered_map<int, std::vector<StateType>>;
using Action = std::vector<double>;
using ActionSequence = std::vector<Action>;
using ParamsType = std::unordered_map<std::string, double>;
using TimeType = int;
const TimeType INF_TIME_TYPE = std::numeric_limits<TimeType>::max() / 2;
using SafeIntervalType = std::pair<TimeType, TimeType>;
using LockType = std::mutex;

enum PARENT_TYPE {
    START = -1,
    GOAL = -2,
    UNSET = -3};

// Structs
struct PlannerStats{
    double time {0};
    double cost {INF_DOUBLE};
    int path_length {INF_INT};
    std::vector<double> transition_costs; // Each step in the path has a cost to move to the next step. The last one is zero.

    int num_expanded {0};
    int num_generated {0};
    int num_reopened {0};

    double suboptimality {1};  // AKA: epsilon

    // Add a catch-all field for any other stats that might be useful.
    std::unordered_map<std::string, double> bonus_stats;
};

template<class Key,
        class T,
        class Hash = std::hash<Key>,
        class KeyEqual = std::equal_to<Key>,
        class Allocator = std::allocator<std::pair<const Key, T>>>
using hash_map = std::unordered_map<Key, T, Hash, KeyEqual, Allocator>;

/// helper struct to compute a hash value for a pointer using the hash value of
/// the object it points to
template <typename T>
struct PointerValueHash
{
    typedef T* argument_type;
    typedef std::size_t result_type;
    result_type operator()(argument_type s) const { return std::hash<T>()(*s); }
};

/// Helper struct to test for equality between two pointers by testing for
/// equality between the objects they point to.
template <typename T>
struct PointerValueEqual
{
    typedef T* argument_type;
    bool operator()(argument_type a, argument_type b) const { return *a == *b; }
};

template <class T, class Allocator = std::allocator<T>>
struct VectorHash
{
    using argument_type = std::vector<T, Allocator>;
    using result_type = std::size_t;
    auto operator()(const argument_type& s) const -> result_type
    {
        auto seed = result_type(0);
        boost::hash_combine(seed, boost::hash_range(begin(s), end(s)));
        return seed;
    }
};

struct StateTypeHash {
    typedef StateType argument_type;
    typedef std::size_t result_type;

    result_type operator()(const argument_type& s) const {
        auto seed = result_type(0);
        boost::hash_combine(seed, boost::hash_range(begin(s), end(s)));
        return seed;
    }
};

enum class ExperienceReuseType {
    NONE,
    PREVIOUS_SOLUTION,
    CT_BRANCH,
    CT_GLOBAL
};

