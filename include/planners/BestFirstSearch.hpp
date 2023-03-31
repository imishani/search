//
// Created by itamar on 3/29/23.
//

#ifndef SEARCH_BESTFIRSTSEARCH_HPP
#define SEARCH_BESTFIRSTSEARCH_HPP

// standard includes
#include <functional>
// Standard includes
#include <utility>
#include <algorithm>

// project includes
#include <planners/planner.hpp>

namespace ims{

    /// @class BestFirstSearch Parameters
    /// @note Before initializing the planner, the heuristic function must be set
    /// So you define a heuristic function and then pass it to the constructor of the BestFirstSearchParams
    /// @note Since this is general BestFS, the heuristic function returns an f value!
    struct BestFirstSearchParams : public PlannerParams{
        /// @brief Constructor
        explicit BestFirstSearchParams(Heuristic &heuristic) : PlannerParams(), m_heuristicFunction(heuristic) {}

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
        /// @param actionSpacePtr The action space
        /// @param start The start state
        /// @param goal The goal state
        void initializePlanner(std::shared_ptr<actionSpace>& actionSpacePtr, stateType start, stateType goal) override;

        /// TODO: Do I need this function?
        /// @brief Get the state by id
        /// @param state_id The id of the state
        /// @return The state
        state* getState(size_t state_id);

        /// @brief Compute the heuristic value of from state s to the goal state
        /// @param s The state
        double computeHeuristic(state& s);

        /// @brief Compute the heuristic value from state s1 to state s2
        /// @param s1 The state
        /// @param s2 The state
        double computeHeuristic(state& s1, state& s2);

        /// @brief plan
        /// @param path The path
        /// @return if the plan was successful or not
        bool plan(std::vector<state*>& path) override;

    protected:

        virtual void setStateVals(state* state_, state* parent, double cost);

        void expand(state* state_) override;

        void reconstructPath(std::vector<state*>& path) override;

        bool isGoalState(const state& s) override;

        Heuristic m_heuristicFunction;

    };

}




#endif //SEARCH_BESTFIRSTSEARCH_HPP
