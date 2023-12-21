// standard includes
#include <functional>
// Standard includes
#include <utility>
#include <map>
#include <algorithm>
#include <stack>
// project includes
#include "search/action_space/action_space.hpp"
#include <search/planners/planner.hpp>
#include <search/common/intrusive_heap.h>
#include <search/common/types.hpp>
#include "search/heuristics/base_heuristic.hpp"



namespace ims{

    /// @class realTimePlannerParams class.
    /// @brief The parameters for the real-time planning algorithm
    struct realTimePlannerParams : public PlannerParams{

        /// @brief constructor
        /// @param heuristic base heuristic
        /// @param N number of expanded states in each iteration
        /// @param rtaa whether to use the rtaa* or lrta* algorithm
        explicit realTimePlannerParams(BaseHeuristic* heuristic, int N, bool rtaa): PlannerParams(){
            heuristic_ = heuristic;
            time_limit_ = 1000;
            N_ = N;     
            rtaa_ = rtaa;
        }

        /// @brief destructor
        ~realTimePlannerParams() override = default;

        int N_;
        bool rtaa_;
        BaseHeuristic* heuristic_ = nullptr;
    };

    class realTimePlanner : public Planner{
        
    protected:
        struct SearchState: public ims::SearchState {

            /// @brief The parent state
            int parent_id = UNSET;
            /// @brief The cost to come
            double g = INF_DOUBLE;
            /// @brief The f value
            double f = INF_DOUBLE;
            double h = INF_DOUBLE;
            /// @brief open list boolean
            bool in_open = false;
            /// @brief closed list boolean
            bool in_closed = false;

            /// @brief set the state to open list (make sure it is not in closed list and if it is, update it)
            void setOpen(){
                in_open = true;
                in_closed = false;
            }

            /// @brief set the state to closed list (make sure it is not in open list and if it is, update it)
            void setClosed(){
                in_closed = true;
                in_open = false;
            }

            void print() override{
                std::cout << "State: " << state_id << " Parent: " << parent_id << " g: " << g << " f: " << f << std::endl;
            }
        };

        /// @brief The search state compare struct.
        struct SearchStateCompare{
            bool operator()(const SearchState& s1, const SearchState& s2) const{
                if ((s1.f == s2.f) && (s1.g == s2.g))
                    return (s1.state_id < s2.state_id);
                else if (s1.f == s2.f)
                    return s1.g < s2.g;
                else
                    return s1.f < s2.f;
            }
        };

        using OpenList = ::smpl::IntrusiveHeap<SearchState, SearchStateCompare>;
        OpenList open_;

        std::vector<SearchState*> states_;

        std::vector<SearchState*> close_;

        std::map<SearchState*, double> heuristicDict;

        auto getSearchState(int state_id) -> SearchState*;

        auto getOrCreateSearchState(int state_id) -> SearchState*;

    public:

        explicit realTimePlanner(const realTimePlannerParams &params);

        ~realTimePlanner() override;


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

        /// @brief update the heuristic of visited states in the dictionary using the RTAA* algorithm
        void updateHeuristicRTAA();

        /// @brief update the heuristic of visited states in the dictionary using the LRTA* algorithm
        void updateHeuristicLRTA();


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

        /// @brief expand the neighbors of the current state by updating the close and open list
        /// @param state_id current state to be expanded
        void expand(int state_id);

        void reconstructPath(std::vector<StateType>& path) override;

        bool isGoalState(int state_id) override;

        realTimePlannerParams params_;

        BaseHeuristic* heuristic_;
    };
}