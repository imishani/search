//
// Created by spencer on 9/26/24.
//

#ifndef SEARCH_MTRRT_HPP
#define SEARCH_MTRRT_HPP
// standard includes
#include <functional>
#include <utility>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <vector>

// project includes
#include <search/heuristics/standard_heuristics.hpp>
#include <search/planners/sampling_based/sampling_based_planner.hpp>
#include <search/action_space/mtrrt_action_space.hpp>
#include <search/common/distance.hpp>

namespace ims {

/// @class MTRRTParams class.
/// @brief The parameters for the MT-RRT algorithm
struct MTRRTParams : public SamplingBasedPlannerParams {
    explicit MTRRTParams(BaseHeuristic* heuristic, double step_size, int max_itr) : SamplingBasedPlannerParams(), step_size_(step_size), max_itr_(max_itr) {};

    ~MTRRTParams() override = default;

    BaseHeuristic* heuristic_ = nullptr;

    double step_size_ = 10.0;

    int max_itr_ = 1000;
};


class MTRRT : public SamplingBasedPlanner {

public:
    explicit MTRRT(const MTRRTParams& params);

    ~MTRRT() override = default;

    void initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
                           const StateType& start,
                           const StateType& goal) override;

    bool plan(std::vector<StateType> &path) override;

    void resetPlanningData() override;

protected:
    void reconstructPath(std::vector<StateType>& path) override;

    void reconstructPath(std::vector<StateType>& path, std::vector<double>& costs) override;

    void sampleRoboState(StateType& sampled_robo_state);

    std::shared_ptr<MTRRTActionSpace> mtrrt_action_space_ptr_ = nullptr;

private:
    struct SearchState : public ims::SearchState {
        int parent_id = UNSET;
        double g = INF_DOUBLE;
        SearchState(): parent_id(UNSET){}

        void print() override {
            std::cout << "State: " << state_id << " Parent: " << parent_id << "g: " << g << std::endl;
        }
    };

    MTRRTParams params_;
    std::vector<SearchState*> states_;

    void getNearestSearchState(const StateType& sample_state, SearchState*& nearest_state, double& nearest_state_distance);

    /// @brief Distance function from two robot state values
    double distanceRoboStates(const StateType& state_1, const StateType& state_2);
    /// @brief Distance function from two search state id
    double distanceSearchState(SearchState*& state_1, SearchState*& state_2);

    bool extend(SearchState*& state_to_extend, const StateType& sampled_robo_state, SearchState*& new_search_state);

    auto getSearchState(int state_id) -> SearchState *;

    auto getOrCreateSearchState(int state_id) -> SearchState *;

    bool canReachGoal(SearchState*& cur_search_state);

    bool isGoalState(int s_id) override {
        return false;
    }

    bool getNewSample(int& sample_id) override {
        return false;
    }


};  // namespace ims



}

#endif //SEARCH_MTRRT_HPP
