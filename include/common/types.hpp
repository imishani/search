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


#ifndef SEARCH_TYPES_HPP
#define SEARCH_TYPES_HPP

#include <vector>
#include <limits>
#include <unordered_map>
#include <boost/functional/hash.hpp>

// Constants
const double INF = std::numeric_limits<double>::infinity();
const int Infinity = std::numeric_limits<int16_t>::max();
const double PI = 3.14159265358979323846;


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

/// helper struct to test for equality between two pointers by testing for
/// equality between the objects they point to
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

// Typedefs
using stateType = std::vector<double>;
using pathType = std::vector<stateType>;
using action = std::vector<double>;
using paramsType = std::unordered_map<std::string, double>;
//using Heuristic = std::function<bool(const stateType&, const stateType&, double&)>;

enum PARENT_TYPE {
    START = -1,
    GOAL = -2,
    INVALID = -3};

// Structs
struct plannerStats{
    double time {0};
    double cost {0};
    int pathLength {0};

    int numExpanded {0};
    int numGenerated {0};
    int numReopened {0};

    int subOptimality {0};  // AKA: epsilon

};


#endif //SEARCH_TYPES_HPP
