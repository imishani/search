//
// Created by spencer on 9/26/24.
//

#include "search/planners/sampling_based/mtrrt.hpp"

ims::MTRRT::MTRRT(const ims::MTRRTParams& params) : params_(params), SamplingBasedPlanner(params){}

void ims::MTRRT::initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
                                   const StateType& start,
                                   const StateType& goal) {
    mtrrt_action_space_ptr_ = std::dynamic_pointer_cast<MTRRTActionSpace>(action_space_ptr);
    action_space_ptr_ = mtrrt_action_space_ptr_;
    // Clear both.
    mtrrt_action_space_ptr_->resetPlanningData();
    resetPlanningData();

    // check if start is valid
    if (!mtrrt_action_space_ptr_->isStateValid(start)){
        throw std::runtime_error("Start state is not valid");
    }
    // check if goal is valid
    if (!mtrrt_action_space_ptr_->isStateValid(goal)){
        throw std::runtime_error("Goal state is not valid");
    }
    int start_ind_ = mtrrt_action_space_ptr_->getOrCreateRobotState(start);
    auto search_start_ = getOrCreateSearchState(start_ind_);
    search_start_->g = 0.0;

    int goal_ind_ = mtrrt_action_space_ptr_->getOrCreateRobotState(goal);
    auto search_goal_ = getOrCreateSearchState(goal_ind_);
    goal_ = search_goal_->state_id;
    goals_.push_back(goal_ind_);

    search_start_->parent_id = PARENT_TYPE(START);
    search_goal_->parent_id = PARENT_TYPE(GOAL);

}

auto ims::MTRRT::getSearchState(int state_id) -> ims::MTRRT::SearchState * {
    assert(state_id < states_.size() && state_id >= 0);
    return states_[state_id];
}

auto ims::MTRRT::getOrCreateSearchState(int state_id) -> ims::MTRRT::SearchState * {
    if (state_id >= states_.size()){
        states_.resize(state_id + 1, nullptr);
    }
    if (states_[state_id] == nullptr){
        assert(state_id < states_.size() && state_id >= 0);
        states_[state_id] = new SearchState;
        states_[state_id]->state_id = state_id;
    }
    return states_[state_id];
}


void ims::MTRRT::sampleRoboState(StateType& sampled_robo_state) {
    StateType sampled_state;
    mtrrt_action_space_ptr_->sampleState(sampled_state);
    sampled_robo_state = sampled_state;
}

// Must make sure we are not extending from the goal state here.
void ims::MTRRT::getNearestSearchState(const StateType& sample_state, SearchState*& nearest_state, double& nearest_state_distance) {
    nearest_state = nullptr;
    nearest_state_distance = std::numeric_limits<double>::max();

    for (auto& state : states_) {
        // find the distance between the sample state to a state from the RRT
        StateType state_from_tree = mtrrt_action_space_ptr_->getRobotState(state->state_id)->state;
        double distance = distanceRoboStates(sample_state, state_from_tree);
        if (distance < nearest_state_distance) {
            // Check if the nearest state candidate is the goal state, skip it if true
            if (state->parent_id == PARENT_TYPE(GOAL)) {
                continue;
            }
            nearest_state = state;
            nearest_state_distance = distance;
        }
    }
}

double ims::MTRRT::distanceRoboStates(const StateType& state_1, const StateType& state_2) {
    return ims::euclideanDistance(state_1, state_2);
}

double ims::MTRRT::distanceSearchState(SearchState*& state_1, SearchState*& state_2) {
    StateType rob_state_1 = mtrrt_action_space_ptr_->getRobotState(state_1->state_id)->state;
    StateType rob_state_2 = mtrrt_action_space_ptr_->getRobotState(state_2->state_id)->state;
    return distanceRoboStates(rob_state_1, rob_state_2);
}

void ims::MTRRT::resetPlanningData(){
    for (auto state_ : states_){
        delete state_;
    }
    states_.clear();
    goals_.clear();
    goal_ = -1;
    stats_ = PlannerStats();
}

void ims::MTRRT::reconstructPath(std::vector<StateType>& path) {
    SearchState* state_ = getSearchState(goal_);
    int counter = 0;
    while (state_->parent_id != -1){
        path.push_back(action_space_ptr_->getRobotState(state_->state_id)->state);
        state_ = getSearchState(state_->parent_id);
        std::cout << counter << ": " << action_space_ptr_->getRobotState(state_->state_id)->state[0] << " "
                                     << action_space_ptr_->getRobotState(state_->state_id)->state[1] << std::endl;
        counter ++;
    }
    path.push_back(action_space_ptr_->getRobotState(state_->state_id)->state);
    std::reverse(path.begin(), path.end());
}

void ims::MTRRT::reconstructPath(std::vector<StateType>& path, std::vector<double>& costs) {
    path.clear();
    costs.clear();

    costs.push_back(0); // The goal state gets a transition cost of 0.
    SearchState* state_ = getSearchState(goal_);
    while (state_->parent_id != -1){
        path.push_back(action_space_ptr_->getRobotState(state_->state_id)->state);
        double transition_cost = state_->g - getSearchState(state_->parent_id)->g;
        costs.push_back(transition_cost);

        state_ = getSearchState(state_->parent_id);
    }
    path.push_back(action_space_ptr_->getRobotState(state_->state_id)->state);

    std::reverse(path.begin(), path.end());
    std::reverse(costs.begin(), costs.end());
}

bool ims::MTRRT::plan(std::vector<StateType> &path){
    startTimer();
    for (int i=0; i < params_.max_itr_; i++){
        StateType sampled_robo_state;
        sampleRoboState(sampled_robo_state);

        SearchState* state_to_extend;
        double distance_from_sample;
        getNearestSearchState(sampled_robo_state, state_to_extend, distance_from_sample);

        SearchState* new_search_state = nullptr;
        // extend also takes care of add_vertex
        bool reachedOrAdvanced = extend(state_to_extend, sampled_robo_state, new_search_state);
        if (!reachedOrAdvanced) {
            i--;
            continue;
        }
        // add edge manually
        new_search_state->parent_id = state_to_extend->state_id;
        new_search_state->g = state_to_extend->g + distanceSearchState(state_to_extend,
                                                                       new_search_state);

        if(canReachGoal(new_search_state)) {
            std::cout << GREEN << "We are reaching the Goal!" << RESET << std::endl;
            SearchState* goal_search_state = getSearchState(goal_);
            goal_search_state->parent_id = new_search_state->state_id;
            goal_search_state->g = new_search_state->g + distanceSearchState(new_search_state,
                                                                            goal_search_state);
            reconstructPath(path, stats_.transition_costs);

            getTimeFromStart(stats_.time);
            stats_.cost = goal_search_state->g;
            stats_.path_length = (int) path.size();
            stats_.num_generated = (int)mtrrt_action_space_ptr_->states_.size();
            return true;
        }
    }
    return false;
}

bool ims::MTRRT::extend(SearchState*& state_to_extend, const StateType& sampled_robo_state, SearchState*& new_search_state) {
    int cur_state_id = state_to_extend->state_id;
    StateType succ_state = StateType(sampled_robo_state.size());
    mtrrt_action_space_ptr_->getSuccessorInDirection(cur_state_id, sampled_robo_state, succ_state, params_.step_size_);
    // If trapped (no movement at all), return false
    if (succ_state.empty()) {
        return false;
    }
    if (succ_state == mtrrt_action_space_ptr_->getRobotState(cur_state_id)->state) {
        return false;
    }
    // If reach
    if (succ_state == sampled_robo_state) {
        int new_state_id = mtrrt_action_space_ptr_->getOrCreateRobotState(succ_state);
        new_search_state = getOrCreateSearchState(new_state_id);
        stats_.num_expanded++;
        return true;
    } else {
        // Advanced but not reach
        int new_state_id = mtrrt_action_space_ptr_->getOrCreateRobotState(succ_state);
        new_search_state = getOrCreateSearchState(new_state_id);
        stats_.num_expanded++;
        return true;
    }
}

bool ims::MTRRT::canReachGoal(SearchState*& cur_search_state) {
    // success
    int cur_state_id = cur_search_state->state_id;
    StateType goal_state = mtrrt_action_space_ptr_->getRobotState(goal_)->state;
    StateType succ_state = StateType(goal_state.size());
    mtrrt_action_space_ptr_->getSuccessorInDirection(cur_state_id, goal_state, succ_state, params_.step_size_);

    if (mtrrt_action_space_ptr_->isSameState(goal_state, succ_state)) {
        return true;
    }
    return false;
}


