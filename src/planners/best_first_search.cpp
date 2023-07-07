//
// Created by itamar on 3/29/23.
//

#include <search/planners/best_first_search.hpp>


ims::BestFirstSearch::BestFirstSearch(const BestFirstSearchParams &params) : Planner(params) {
    heuristic_ = params.heuristic_;
}

void ims::BestFirstSearch::initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
                                             const StateType& start, const StateType& goal) {
    // space pointer
    action_space_ptr_ = action_space_ptr;
    int start_ind_ = action_space_ptr_->getOrCreateRobotState(start);
    auto start_ = getOrCreateSearchState(start_ind_);

    int goal_ind_ = action_space_ptr_->getOrCreateRobotState(goal);
    auto goal_ = getOrCreateSearchState(goal_ind_);
    goals_.push_back(goal_ind_);

    // Evaluate the start state
    start_->parent_id = PARENT_TYPE(START);
    // Evaluate the goal state
    goal_->parent_id = PARENT_TYPE(GOAL);
    heuristic_->setGoal(const_cast<StateType &>(goal));
    // Evaluate the start state
    start_->g = 0;
    start_->f = computeHeuristic(start_ind_);
    open_.push(start_);
    start_->setOpen();
}

auto ims::BestFirstSearch::getSearchState(int state_id) -> ims::BestFirstSearch::SearchState*{
    assert(state_id < states_.size() && state_id >= 0);
    return states_[state_id];
}

auto ims::BestFirstSearch::getOrCreateSearchState(int state_id) -> ims::BestFirstSearch::SearchState * {
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


double ims::BestFirstSearch::computeHeuristic(int state_id) {
    double dist;
    auto s = action_space_ptr_->getRobotState(state_id);
    if (!heuristic_->getHeuristic(s->state, dist))
        throw std::runtime_error("Heuristic function failed");
    else
        return dist;
}

double ims::BestFirstSearch::computeHeuristic(int s1_id, int s2_id) {
    double dist;
    auto s1 = action_space_ptr_->getRobotState(s1_id);
    auto s2 = action_space_ptr_->getRobotState(s2_id);
    if (!heuristic_->getHeuristic(s1->state, s2->state, dist))
        throw std::runtime_error("Heuristic function failed");
    else
        return dist;
}


bool ims::BestFirstSearch::plan(std::vector<StateType>& path) {
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

void ims::BestFirstSearch::expand(int state_id){
    auto state = getSearchState(state_id);
    std::vector<int> successors;
    std::vector<double> costs; // In this case we use the "cost" as the new f value
    action_space_ptr_->getSuccessors(state->state_id, successors, costs);
    for (size_t i {0} ; i < successors.size() ; ++i){
        int successor_id = successors[i];
        double cost = costs[i];
        auto successor = getOrCreateSearchState(successor_id);
        if (successor->in_closed){
            continue;
        }
        if (successor->in_open){
            if (successor->f > cost){
                successor->parent_id = state->state_id;
                successor->f = cost;
                open_.update(successor);
            }
        } else {
            setStateVals(successor->state_id, state->state_id, cost);
            open_.push(successor);
            successor->setOpen();
        }
    }
    stats_.num_expanded++;
}


void ims::BestFirstSearch::setStateVals(int state_id, int parent_id, double cost) {
    auto state_ = getSearchState(state_id);
    state_->parent_id = parent_id;
    state_->f = cost;
}


void ims::BestFirstSearch::reconstructPath(std::vector<StateType>& path) {
    SearchState* state = getSearchState(goal_);
    while (state->parent_id != -1){
        path.push_back(action_space_ptr_->getRobotState(state->state_id)->state);
        state = getSearchState(state->parent_id);
    }
    path.push_back(action_space_ptr_->getRobotState(state->state_id)->state);
    std::reverse(path.begin(), path.end());
}

bool ims::BestFirstSearch::isGoalState(int s_id) {
    return std::any_of(goals_.begin(), goals_.end(), [&s_id](int goal_ind) {return s_id == goal_ind;});
}


