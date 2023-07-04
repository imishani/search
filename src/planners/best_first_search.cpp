//
// Created by itamar on 3/29/23.
//

#include <planners/best_first_search.hpp>


ims::BestFirstSearch::BestFirstSearch(const BestFirstSearchParams &params) : Planner(params) {
    heuristic_ = params.heuristic_;
}

void ims::BestFirstSearch::initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
                                             const StateType& start, const StateType& goal) {
    // space pointer
    action_space_ptr_ = action_space_ptr;
    int start_ind_ = action_space_ptr_->getOrCreateState(start);
    start_ = action_space_ptr_->getState(start_ind_);

    int goal_ind_ = action_space_ptr_->getOrCreateState(start);
    goal_ = action_space_ptr_->getState(goal_ind_);
    // Evaluate the start state
    start_->setParent(PARENT_TYPE(START));
    start_->setState(start);
    start_->g = 0;
    start_->f = computeHeuristic(start_);
    open_.push(start_);
    start_->setOpen();
    // Evaluate the goal state
    goal_->setParent(PARENT_TYPE(GOAL));
    goal_->setState(goal);
    heuristic_->setGoal(goal_);
}

ims::State *ims::BestFirstSearch::getState(size_t state_id) {
    return action_space_ptr_->getState(state_id);
}

double ims::BestFirstSearch::computeHeuristic(ims::State* s) {
    return computeHeuristic(s, goal_);
}

double ims::BestFirstSearch::computeHeuristic(ims::State* s1, ims::State* s2) {
    double dist;
    if (!heuristic_->getHeuristic(s1, s2, dist))
        throw std::runtime_error("Heuristic function failed");
    else
        return dist;
}


bool ims::BestFirstSearch::plan(std::vector<State*>& path) {
    startTimer();
    int iter {0};
    while (!open_.empty() && !isTimeOut()){
        // report progress every 1000 iterations
        if (iter % 100000 == 0){
            std::cout << "open size: " << open_.size() << std::endl;
        }
        State* state  = open_.min();
        open_.pop();
        state->setClosed();
        if (isGoalState(*state)){
            goal_ = state;
            getTimeFromStart(stats_.time);
            reconstructPath(path);
            stats_.cost = goal_->g;
            stats_.path_length = (int)path.size();
            stats_.num_generated = (int)action_space_ptr_->states_.size();
            return true;
        }
        expand(state);
        ++iter;
    }
    getTimeFromStart(stats_.time);
    return false;
}

void ims::BestFirstSearch::expand(ims::State* state_){
    std::vector<State*> successors;
    std::vector<double> costs; // In this case we use the "cost" as the new f value
    action_space_ptr_->getSuccessors(state_->getStateId(), successors, costs);
    for (size_t i {0} ; i < successors.size() ; ++i){
        State* successor = successors[i];
        double cost = costs[i];
        if (successor->isClosed()){
            continue;
        }
        if (successor->isOpen()){
            if (successor->f > cost){
                successor->setParent(state_->getStateId());
                successor->f = cost;
                open_.update(successor);
            }
        } else {
            setStateVals(successor, state_, cost);
            open_.push(successor);
            successor->setOpen();
        }
    }
    stats_.num_expanded++;
}

void ims::BestFirstSearch::reconstructPath(std::vector<State*>& path) {
    State* state_ = goal_;
    while (state_->getParentInd() != -1){
        path.push_back(state_);
        state_ = getState(state_->getParentInd());
    }
    path.push_back(state_);
    std::reverse(path.begin(), path.end());
}

bool ims::BestFirstSearch::isGoalState(const ims::State &s) {
    if (s.getStateId() == goal_->getStateId())
        return true;
    else
        return false;
}

void ims::BestFirstSearch::setStateVals(State* state_, State* parent, double cost)
{
    state_->setParent(parent->getStateId());
    state_->f = cost;
}


