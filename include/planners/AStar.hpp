//
// Created by itamar on 3/30/23.
//

#ifndef SEARCH_ASTAR_HPP
#define SEARCH_ASTAR_HPP

// standard includes
#include <functional>
// Standard includes
#include <utility>
#include <algorithm>

// project includes
#include <planners/BestFirstSearch.hpp>

namespace ims{

    /// @class AStarParams class.
    /// @brief The parameters for the AStar algorithm
    struct AStarParams : public BestFirstSearchParams{

        /// @brief Constructor
        /// @param heuristic The heuristic function. Passing the default heuristic function will result in a uniform cost search
        explicit AStarParams(Heuristic &heuristic) : BestFirstSearchParams(heuristic) {}

        /// @brief Destructor
        ~AStarParams() override = default;

    };


    /// @class BestFirstSearch class.
    /// @brief A general search algorithm that uses heuristics and g values to find the optimal path
    class AStar : public BestFirstSearch{
    public:
        /// @brief Constructor
        /// @param params The parameters
        explicit AStar(const AStarParams &params);

        /// @brief Destructor
        ~AStar() override = default;

        /// @brief Initialize the planner
        /// @param actionSpacePtr The action space
        /// @param start The start state
        /// @param goal The goal state
        void initializePlanner(std::shared_ptr<actionSpace>& actionSpacePtr, stateType start, stateType goal) override;


    protected:

        void setStateVals(state* state_, state* parent, double cost) override;

        void expand(state* state_) override;

        Heuristic m_heuristicFunction;

    };

}


#endif //SEARCH_ASTAR_HPP
