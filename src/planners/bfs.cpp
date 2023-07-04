//
// Created by itamar on 4/13/23.
//

#include "planners/bfs.hpp"

//ims::BFS::BFS(const ims::BFSParams &params) : Planner(params) {
//    exhaustive = params.mExhaustive;
//}
//
//
//void ims::BFS::initializePlanner(const std::shared_ptr<ActionSpace>& actionSpacePtr,
//                                             const StateType& start, const StateType& goal) {
//    // space pointer
//    action_space_ptr_ = actionSpacePtr;
//
//    int start_ind_ = action_space_ptr_->getOrCreateState(start);
//    start_ = action_space_ptr_->getState(start_ind_);
//
//    int goal_ind_ = action_space_ptr_->getOrCreateState(start);
//    goal_ = action_space_ptr_->getState(goal_ind_);
//    // Evaluate the start state
//    start_->setParent(PARENT_TYPE(START));
//    start_->setState(start);
//    start_->g = 0;
//    start_->f = start_->g;
//    open_.push(start_);
//    start_->setOpen();
//    // Evaluate the goal state
//    goal_->setParent(PARENT_TYPE(GOAL));
//    goal_->setState(goal);
//}
//
//ims::State *ims::BFS::getState(size_t state_id) {
//    return action_space_ptr_->getState(state_id);
//}
//
//bool ims::BFS::plan(std::vector<state*>& path) {
//    startTimer();
//    int iter {0};
//    while (!open_.empty() && !isTimeOut()){
//        // report progress every 1000 iterations
//        if (iter % 1000 == 0){
//            std::cout << "open size: " << open_.size() << std::endl;
//        }
//       State* state  = open_.min();
//        open_.pop();
//        state->setClosed();
//        if (isGoalState(*state)){
//            goal_ = state;
//            getTimeFromStart(stats_.time);
//            reconstructPath(path);
//            stats_.cost = goal_->g;
//            stats_.path_length = (int)path.size();
//            ims::State::resetIdCounter();
//            return true;
//        }
//        expand(state);
//        ++iter;
//    }
//    getTimeFromStart(stats_.time);
//    return false;
//}
//
//void ims::BFS::expand(ims::State* state_){
//    std::vector<State*> successors;
//    std::vector<double> costs; // In this case we use the "cost" as the new f value
//    action_space_ptr_->getSuccessors(state_->getStateId(), successors, costs);
//    for (size_t i {0} ; i < successors.size() ; ++i){
//       State* successor = successors[i];
//        double cost = costs[i];
//        if (successor->isClosed()){
//            continue;
//        }
//        if (successor->isOpen()){
//            if (successor->f > cost){
//                successor->setParent(state_->getStateId());
//                successor->f = cost;
//                open_.update(successor);
//            }
//        } else {
//            setStateVals(successor, state_, cost);
//            open_.push(successor);
//            successor->setOpen();
//        }
//    }
//}
//
//void ims::BFS::reconstructPath(std::vector<State*>& path) {
//   State* state_ = goal_;
//    while (state_->getParentInd() != -1){
//        path.push_back(state_);
//        state_ = getState(state_->getParentInd());
//    }
//    path.push_back(state_);
//    std::reverse(path.begin(), path.end());
//}
//
//bool ims::BFS::isGoalState(const ims::State &s) {
//    if (s.getStateId() == goal_->getStateId())
//        return true;
//    else
//        return false;
//}
//
//void ims::BFS::setStateVals(State* state_(State* parent, double cost)
//{
//    state_->setParent(parent->getStateId());
//    state_->f = cost;
//}
