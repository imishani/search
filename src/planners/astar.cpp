//
// Created by itamar on 3/30/23.
//

#include <search/planners/astar.hpp>


ims::AStar::AStar(const ims::AStarParams &params) : BestFirstSearch(params) {}

void ims::AStar::initializePlanner(const std::shared_ptr<ActionSpace>& actionSpacePtr,
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
    int start_ind_ = action_space_ptr_->getOrCreateState(start);
    printf("start ind: %d \n", start_ind_);
    start_ = action_space_ptr_->getState(start_ind_);
    start_->setParent(START);
    int goal_ind_ = action_space_ptr_->getOrCreateState(goal);
    goal_ = action_space_ptr_->getState(goal_ind_);
    goal_->setParent(GOAL);
    heuristic_->setGoal(goal_);
    // Evaluate the start state
    start_->g = 0;
    start_->h = computeHeuristic(start_);
    start_->f = start_->g + start_->h;
    open_.push(start_);
    start_->setOpen();
    // Evaluate the goal state
    goal_->h = 0;

}


void ims::AStar::expand(ims::State* state_){
    std::vector<State*> successors;
    std::vector<double> costs;
    action_space_ptr_->getSuccessors(state_->getStateId(), successors, costs);
    for (size_t i {0} ; i < successors.size() ; ++i){
       State* successor = successors[i];
        double cost = costs[i];
        if (successor->isClosed()){
            continue;
        }
        if (successor->isOpen()){
            if (successor->g > state_->g + cost){
                successor->setParent(state_->getStateId());
                successor->g = state_->g + cost;
                successor->f = successor->g + successor->h;
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


void ims::AStar::setStateVals(State* state_, State* parent, double cost)
{
    state_->setParent(parent->getStateId());
    state_->g = parent->g + cost;
    state_->h = computeHeuristic(state_);
    state_->f = state_->g + state_->h;
}


