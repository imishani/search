// standard includes
#include <functional>
// Standard includes
#include <utility>
#include <map>
#include <algorithm>
// project includes
#include "search/action_space/action_space.hpp"
#include <search/planners/planner.hpp>
#include <search/common/intrusive_heap.h>
#include <search/common/types.hpp>
#include "search/heuristics/base_heuristic.hpp"



namespace ims{
    struct rtaaStarParams : public PlannerParams{
        explicit rtaaStarParams(BaseHeuristic* heuristic, int N): PlannerParams(){
            heuristic_ = heuristic;
            N_ = N;
        }

        ~rtaaStarParams() override = default;

        int N_;
        BaseHeuristic* heuristic_ = nullptr;
    };

    class rtaaStar : public Planner{
        
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

        explicit rtaaStar(const rtaaStarParams &params);

        ~rtaaStar() override;

        void initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
                               const std::vector<StateType>& starts,
                               const std::vector<StateType>& goals) override;
            
        void initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
                               const StateType& start, const StateType& goal) override;

        void constructPartialPath(int state_id, int target_id, std::vector<StateType>& path);
        
        bool plan(std::vector<StateType> &path) override;

        void updateHeuristicRTAA();

        void updateHeuristicLRTA();

        auto executePartialPlan(SearchState* start, std::vector<StateType>& path);

        std::string budgetedPlan(SearchState* start, std::vector<StateType>& path);

        /// @brief Reset the planner
        void resetPlanningData() override;

        double computeHeuristic(int state_id);

    protected:

        void setStateVals(int state_id, int parent_id, double cost);

        void expand(int state_id);

        void reconstructPath(std::vector<StateType>& path) override;

        bool isGoalState(int state_id) override;

        rtaaStarParams params_;

        BaseHeuristic* heuristic_;
    };
}