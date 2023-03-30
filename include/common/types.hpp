//
// Created by itamar on 3/26/23.
//

#ifndef SEARCH_TYPES_HPP
#define SEARCH_TYPES_HPP

#include <vector>
#include <limits>
#include <unordered_map>

// Constants
const double INF = std::numeric_limits<double>::infinity();
const double PI = 3.14159265358979323846;

// Typedefs
using stateType = std::vector<double>;
using actionType = std::vector<double>;
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
