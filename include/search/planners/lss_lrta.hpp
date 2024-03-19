// standard includes
#include <functional>
// Standard includes
#include <utility>
#include <map>
#include <algorithm>
#include <stack>
#include <set>
#include <queue>
// project includes
#include "search/action_space/action_space.hpp"
#include <search/planners/planner.hpp>
#include <search/common/intrusive_heap.h>
#include <search/common/intrusive_heap_wrapper.h>
#include <search/common/queue_general.h>
#include <search/planners/realTimePlanner.hpp>
#include <search/common/types.hpp>
#include "search/heuristics/base_heuristic.hpp"

namespace ims{

    struct lssLRTAPlannerParams: public realTimePlannerParams{
        explicit lssLRTAPlannerParams(BaseHeuristic* heuristic, int N) : realTimePlannerParams(heuristic, N, false) {
            N_ = N;
            heuristic_ = heuristic;
        }


        /// @brief Destructor
        ~lssLRTAPlannerParams() override = default;

        int N_;
        /// @brief the base heuristic of the map
        BaseHeuristic* heuristic_ = nullptr;
    };

    class lssLRTA : public realTimePlanner{
    
    private:

        class heuristicCompare {
        public:
            bool operator()(const SearchState& a, const SearchState &b) const
            {
                return a.h <= b.h;
            }
        };

        // using OpenList = SimpleQueue<SearchState, SearchStateCompare>;
        // OpenList open_;
        ::smpl::IntrusiveHeapWrapper<SearchState, SearchStateCompare> open_;

        // using OpenList_h = SimpleQueue<SearchState, heuristicCompare>;
        // OpenList_h open_H;
        ::smpl::IntrusiveHeapWrapper<SearchState, heuristicCompare> open_H;

        std::vector<SearchState*> states_;

        std::map<SearchState*, double> heuristic_dict_;

        std::map<SearchState*, std::vector<std::tuple<SearchState*, double>> > parent_;

        std::set<SearchState*> close_;

        auto getSearchState(int state_id) -> SearchState*;
    
    public:

        explicit lssLRTA(const lssLRTAPlannerParams &params);

        ~lssLRTA() override;


    /// @brief Initialize the planner
        /// @param action_space_ptr The action space
        /// @param starts Vector of start states
        /// @param goals Vector of goal states
        void initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
                               const std::vector<StateType>& starts,
                               const std::vector<StateType>& goals) override;
            
        /// @brief Initialize the planner
        /// @param action_space_ptr The action space
        /// @param start The start state
        /// @param goal The goal state
        void initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
                               const StateType& start, const StateType& goal) override;

        
        /// @brief construct the path from current iteration of budgeted planning
        /// @param state_id the starting position of the current plan
        /// @param target_id the end position of the current plan
        /// @param path the path that the robot took updated cumulatively
        void constructPartialPath(int state_id, int target_id, std::vector<StateType>& path);
        
        /// @brief plan for a path
        /// @param path the solution of the path
        /// @return if a solution was found
        bool plan(std::vector<StateType> &path) override;

    
        /// @brief update the heuristic of visited states in the dictionary using the LRTA* algorithm
        void updateHeuristicLRTA();

        void updateHeuristicLRTA_K();

        /// @brief moves onto the most promising adjacent state and construct the current partial path
        /// @param start the starting state of the current iteration of budgeted plan
        /// @param path the path that the robot took updated cumulatively
        /// @return the current state of the agent after execution
        auto executePartialPlan(SearchState* start, std::vector<StateType>& path);


        /// @brief given a budget of N, expand N states from start
        /// @param start start state of the agent
        /// @param path the path that the robot took updated cumulatively
        /// @return "found" if a solution is found, "failed" if there is no solution, "continue" if the current plan is not finished
        std::string budgetedPopulateOpen(SearchState* start, std::vector<StateType>& path);

        /// @brief Reset the planner
        void resetPlanningData() override;

        double computeHeuristic(int state_id);

    protected:

        void setSearchStateVals(int state_id, int parent_id, double cost) ;

        auto getOrCreateSearchState(int state_id) -> SearchState*;

        /// @brief expand the neighbors of the current state by updating the close and open list
        /// @param state_id current state to be expanded
        void expand(int state_id);

        void reconstructPath(std::vector<StateType>& path) override;

        bool isGoalState(int state_id) override;

        realTimePlannerParams params_;

        BaseHeuristic* heuristic_;
    };
}