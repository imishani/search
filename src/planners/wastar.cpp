//
// Created by itamar on 4/4/23.
//

#include <search/planners/wastar.hpp>

ims::wAStar::wAStar(const ims::wAStarParams &params) : params_(params), BestFirstSearch(params) {}

void ims::wAStar::initializePlanner(const std::shared_ptr<ActionSpace>& actionSpacePtr,
                                   const StateType& start, const StateType& goal) {
    // space pointer
    action_space_ptr_ = actionSpacePtr;
    // check if start is valid
    if (!action_space_ptr_->isStateValid(start)){
        throw std::runtime_error("Start state is not valid");
    }
    // check if goal is valid
    if (!action_space_ptr_->isStateValid(goal)){
        throw std::runtime_error("Goal state is not valid");
    }
    int start_ind_ = action_space_ptr_->getOrCreateRobotState(start);
    printf("start ind: %d \n", start_ind_);
    auto start_ = getOrCreateSearchState(start_ind_);

    int goal_ind_ = action_space_ptr_->getOrCreateRobotState(goal);
    auto goal_ = getOrCreateSearchState(goal_ind_);
    goals_.push_back(goal_ind_);

    start_->parent_id = PARENT_TYPE(START);
    // Evaluate the goal state
    goal_->parent_id = PARENT_TYPE(GOAL);
    heuristic_->setGoal(const_cast<StateType &>(goal));
    goal_->h = 0;
    // Evaluate the start state
    start_->g = 0;
    start_->h = computeHeuristic(start_ind_);
    start_->f = start_->g + params_.epsilon*start_->h;
    start_->setOpen();

    open_.push(start_);
    // update stats suboptimality
    stats_.suboptimality = params_.epsilon;

}


auto ims::wAStar::getSearchState(int state_id) -> ims::wAStar::SearchState * {
    assert(state_id < states_.size() && state_id >= 0);
    return states_[state_id];
}

auto ims::wAStar::getOrCreateSearchState(int state_id) -> ims::wAStar::SearchState * {
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


bool ims::wAStar::plan(std::vector<StateType>& path) {
    startTimer();
    int iter {0};
    while (!open_.empty() && !isTimeOut()){
        // report progress every 1000 iterations
        if (iter % 100000 == 0){
            std::cout << "open size: " << open_.size() << std::endl;
        }
        auto state  = open_.min();
        open_.pop();
        state->setClosed();
        if (isGoalState(state->state_id)){
            goal_ = state->state_id;
            getTimeFromStart(stats_.time);
            reconstructPath(path);
            stats_.cost = state->g;
            stats_.path_length = (int)path.size();
            stats_.num_generated = (int)action_space_ptr_->states_.size();
            return true;
        }
        expand(state->state_id);
        ++iter;
    }
    getTimeFromStart(stats_.time);
    return false;
}

void ims::wAStar::expand(int state_id){
    auto state_ = getSearchState(state_id);
    std::vector<int> successors;
    std::vector<double> costs;
    action_space_ptr_->getSuccessors(state_->state_id, successors, costs);
    for (size_t i {0} ; i < successors.size() ; ++i){
        int successor_id = successors[i];
        double cost = costs[i];
        auto successor = getOrCreateSearchState(successor_id);
        if (successor->in_closed){
            continue;
        }
        if (successor->in_open){
            if (successor->g > state_->g + cost){
                successor->parent_id = state_->state_id;
                successor->g = state_->g + cost;
                successor->f = successor->g + params_.epsilon*successor->h;
                open_.update(successor);
            }
        } else {
            setStateVals(successor->state_id, state_->state_id, cost);
            open_.push(successor);
            successor->setOpen();
        }
    }
    stats_.num_expanded++;
}

void ims::wAStar::setStateVals(int state_id, int parent_id, double cost)
{
    auto state_ = getSearchState(state_id);
    auto parent = getSearchState(parent_id);
    state_->parent_id = parent_id;
    state_->g = parent->g + cost;
    state_->h = computeHeuristic(state_id);
    state_->f = state_->g + params_.epsilon*state_->h;
}

void ims::wAStar::reconstructPath(std::vector<StateType>& path) {
    SearchState* state_ = getSearchState(goal_);
    while (state_->parent_id != -1){
        path.push_back(action_space_ptr_->getRobotState(state_->state_id)->state);
        state_ = getSearchState(state_->parent_id);
    }
    path.push_back(action_space_ptr_->getRobotState(state_->state_id)->state);
    std::reverse(path.begin(), path.end());
}