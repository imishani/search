#ifndef SEARCH_PLANNER_H
#define SEARCH_PLANNER_H


namespace ims{

    /// Pure virtual base class planner interface
    /// It should include methods for initializing the problem, evaluating the cost of a state,
    /// generating successors, and checking if a state is a goal state.
    /// This interface should be implemented by all search problem instances.
    class Planner {
    public:

        ///@brief Constructor
        Planner();

        ///@brief Destructor
        virtual ~Planner();

        /// Initialize the problem
        virtual void init() = 0;

        /// Check if a state is a goal state
        virtual bool isGoal() = 0;

    protected:
        /// Evaluate the cost of a state
        virtual double evaluateCost() = 0;

        /// Generate successors of a state
        virtual void generateSuccessors() = 0;




    };

}


#endif //SEARCH_PLANNER_H
