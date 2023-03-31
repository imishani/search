//
// Created by itamar on 3/26/23.
//

#ifndef SEARCH_TYPES_HPP
#define SEARCH_TYPES_HPP

#include <vector>
#include <limits>
#include <unordered_map>
#include <boost/functional/hash.hpp>

// Constants
const double INF = std::numeric_limits<double>::infinity();
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
using Heuristic = std::function<double(const stateType&, const stateType&)>;

// Structs
struct plannerStats{
    double time {0};
    double cost {0};
    double pathLength {0};

    int numExpanded {0};
    int numGenerated {0};
    int numReopened {0};

    int subOptimality {0};  // AKA: epsilon

};


#endif //SEARCH_TYPES_HPP
