// standard includes
#include <memory>
#include <chrono>
// project includes
#include "search/action_space/action_space.hpp"
#include <search/common/intrusive_heap.h>
#include <search/common/types.hpp>


namespace ims{

    struct realTimePlannerParams : public PlannerParams{
        realTimePlannerParams(BaseHeuristic* heuristic, int N) : 
            heuristic_(heuristic), N_(N){}

        virtual ~realTimePlannerParams() = default;

        int N;
        BaseHeuristic* heuristic_ = nullptr;
    };

    class realTimePlanner : public Planner{

    private:

        struct SearchState: public ims::Planner::SearchState {

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
            
    public:
        explicit realTimePlanner(const realTimePlannerParams& params): params_(params){

        };

        virtual ~realTimePlanner() = default;

        void initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
                               const std::vector<StateType>& starts,
                               const std::vector<StateType>& goals) override;

        void initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
                               const StateType& start, const StateType& goal) override;

        bool plan(std::vector<StateType>& path) = 0;

        void resetPlanningData() = 0;

    protected:

        void setStateVals(int state_id, int parent_id, double cost);

        double computeHeuristic(int state_id);

        void expand(int state_id);

        void reconstructPath(std::vector<StateType>& path) override;

        bool isGoalState(int state_id) override;

    }





}