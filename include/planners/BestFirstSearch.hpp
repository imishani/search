//
// Created by itamar on 3/29/23.
//

#ifndef SEARCH_BESTFIRSTSEARCH_HPP
#define SEARCH_BESTFIRSTSEARCH_HPP

// standard includes
#include <functional>

// project includes
#include <planners/planner.hpp>

namespace ims{

    /// @class BestFirstSearch Parameters
    /// @note Before initializing the planner, the heuristic function must be set
    /// So you define a heuristic function and then pass it to the constructor of the BestFirstSearchParams
    struct BestFirstSearchParams : public PlannerParams{
        /// @brief Constructor
        BestFirstSearchParams(Heuristic &heuristic) : PlannerParams(), m_heuristicFunction(heuristic) {}

        /// @brief Destructor
        ~BestFirstSearchParams() override = default;

        Heuristic m_heuristicFunction;
    };

    /// @class BestFirstSearch class.
    /// @brief A general search algorithm that uses heuristics and g values to find the optimal path
    class BestFirstSearch : public Planner{
    public:
        /// @brief Constructor
        /// @param params The parameters
        explicit BestFirstSearch(const BestFirstSearchParams &params);

        /// @brief Destructor
        ~BestFirstSearch() override = default;

        /// @brief Initialize the planner
        /// @param start The start state
        /// @param goal The goal state
        void initializePlanner(stateType start, stateType goal) override;

        /// @brief Compute the heuristic value of from state s to the goal state
        /// @param s The state
        double computeHeuristic(state& s);

        /// @brief Compute the heuristic value from state s1 to state s2
        /// @param s1 The state
        /// @param s2 The state
        double computeHeuristic(state& s1, state& s2);

        /// @brief plan
        /// @return if the plan was successful or not
        bool plan() override;

    protected:

        void expand() override;

        void reconstructPath() override;

        Heuristic m_heuristicFunction;

    };

}




#endif //SEARCH_BESTFIRSTSEARCH_HPP
