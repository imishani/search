// standard includes
#include <functional>
// Standard includes
#include <utility>
#include <algorithm>
// project includes
#include <search/planners/real_time_search_planner.hpp>



namespace ims{
    struct rtaaStarParams : public realTimePlannerParams{
        explicit rtaaStarParams(BaseHeurisitc* heuristic, int N) : BestFirstSearchParams(heuristic){
            this->N = N;
        }

        ~rtaaStarParams() override = default;

        int N;
    };
    class rtaaStar : public realTimePlanner{
        
    private:
        struct SearchState: public ims::realTimePlanner::SearchState{};
        using OpenList = ::smpl::IntrusiveHeap<SearchState, SearchStateCompare>;
        OpenList open_;

        std::vector<SearchState*> states_;

        std::vector<SearchState*> close_;

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

        bool plan(std::vector<StateType> &path) override;

        int updateH();

        /// @brief Reset the planner
        void resetPlanningData();

    protected:

        void setStateVals(int state_id, int parent_id, double cost) override;

        void expand(int state_id) override;

        void reconstructPath(std::vector<StateTyoe>& path) override;

        bool isGoalState(int state_id) override;

        rtaaStarParams params_;
    };
}