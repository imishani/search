//
// Created by itamar on 3/30/23.
//

#ifndef SEARCH_DIJKSTRA_HPP
#define SEARCH_DIJKSTRA_HPP

#include <planners/AStar.hpp>

namespace ims{

    /// @brief The heuristic function
    /// @param s1 The state values
    /// @param s2 The state values
    double heuristicFunction(const stateType &s1, const stateType &s2){
        return 0;
    }
    Heuristic heuristic = heuristicFunction;

    struct dijkstraParams : public AStarParams{

        /// @brief Constructor
        /// @param heuristic The heuristic function. Passing the default heuristic function will result in a uniform cost search
        dijkstraParams() : AStarParams(heuristic) {}

        /// @brief Destructor
        ~dijkstraParams() override = default;

    };


    class dijkstra : public AStar{
    public:
        /// @brief Constructor
        /// @param params The parameters
        explicit dijkstra(const dijkstraParams &params);

        /// @brief Destructor
        ~dijkstra() override = default;

    };

}


#endif //SEARCH_DIJKSTRA_HPP
