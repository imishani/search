#ifndef SEARCH_PLANNER_H
#define SEARCH_PLANNER_H

// standard includes

// project includes
#include "common/state.hpp"
#include "common/action.hpp"

// Always initialize the static member variable
size_t ims::state::id_counter = 0;

namespace ims{

    /// @class Planner Parameters abstract class
    struct PlannerParams{
        /// @brief Constructor
        PlannerParams() = default;

        /// @brief Destructor
        virtual ~PlannerParams() = default;
    };

    ///@brief Pure virtual base class planner interface
    /// It should include methods for initializing the problem, evaluating the cost of a state,
    /// generating successors, and checking if a state is a goal state.
    /// This interface should be implemented by all search problem instances.
    class Planner {
    public:

        ///@brief Constructor
        ///@param params The planner parameters based on PlannerParams struct
        explicit Planner(const PlannerParams& params);

        ///@brief Destructor
        virtual ~Planner() = default;

        /// @brief Initialize the planner
        /// @param start The start state
        /// @param goal The goal state
        virtual void initializePlanner(state start, state goal) = 0;


        /// Setters
        /// @brief Set the start state
        /// @param start The start state
        void setStartState(state start) { m_start = &start; }

        /// @brief Set the goal state
        /// @param goal The goal state
        void setGoalState(state goal) { m_goal = &goal; }


        using openList =  smpl::intrusive_heap<state, stateCompare>;
        openList m_open;


    protected:
        /// Evaluate the cost of a state
        virtual double evaluateCost() = 0;

        /// @brief Expand the current state
        virtual void expand() = 0;

        /// @brief Reconstruct the path
        virtual void reconstructPath() = 0;

        state* m_start;
        state* m_goal;
        PlannerParams m_params;


    };



}


#endif //SEARCH_PLANNER_H
