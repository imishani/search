//
// Created by itamar on 4/13/23.
//

#include "planners/BFS.hpp"

//ims::BFS::BFS(const ims::BFSParams &params) : Planner(params) {
//    exhaustive = params.mExhaustive;
//}
//
//
//void ims::BFS::initializePlanner(const std::shared_ptr<actionSpace>& actionSpacePtr,
//                                             const stateType& start, const stateType& goal) {
//    // space pointer
//    m_actionSpacePtr = actionSpacePtr;
//
//    int m_start_ind = m_actionSpacePtr->getOrCreateState(start);
//    m_start = m_actionSpacePtr->getState(m_start_ind);
//
//    int m_goal_ind = m_actionSpacePtr->getOrCreateState(start);
//    m_goal = m_actionSpacePtr->getState(m_goal_ind);
//    // Evaluate the start state
//    m_start->setParent(PARENT_TYPE(START));
//    m_start->setState(start);
//    m_start->g = 0;
//    m_start->f = m_start->g;
//    m_open.push(m_start);
//    m_start->setOpen();
//    // Evaluate the goal state
//    m_goal->setParent(PARENT_TYPE(GOAL));
//    m_goal->setState(goal);
//}
//
//ims::state *ims::BFS::getState(size_t state_id) {
//    return m_actionSpacePtr->getState(state_id);
//}
//
//bool ims::BFS::plan(std::vector<state*>& path) {
//    startTimer();
//    int iter {0};
//    while (!m_open.empty() && !isTimeOut()){
//        // report progress every 1000 iterations
//        if (iter % 1000 == 0){
//            std::cout << "open size: " << m_open.size() << std::endl;
//        }
//        state* state  = m_open.min();
//        m_open.pop();
//        state->setClosed();
//        if (isGoalState(*state)){
//            m_goal = state;
//            getTimeFromStart(m_stats.time);
//            reconstructPath(path);
//            m_stats.cost = m_goal->g;
//            m_stats.pathLength = (int)path.size();
//            ims::state::resetIdCounter();
//            return true;
//        }
//        expand(state);
//        ++iter;
//    }
//    getTimeFromStart(m_stats.time);
//    return false;
//}
//
//void ims::BFS::expand(ims::state* state_){
//    std::vector<state*> successors;
//    std::vector<double> costs; // In this case we use the "cost" as the new f value
//    m_actionSpacePtr->getSuccessors(state_->getStateId(), successors, costs);
//    for (size_t i {0} ; i < successors.size() ; ++i){
//        state* successor = successors[i];
//        double cost = costs[i];
//        if (successor->isClosed()){
//            continue;
//        }
//        if (successor->isOpen()){
//            if (successor->f > cost){
//                successor->setParent(state_->getStateId());
//                successor->f = cost;
//                m_open.update(successor);
//            }
//        } else {
//            setStateVals(successor, state_, cost);
//            m_open.push(successor);
//            successor->setOpen();
//        }
//    }
//}
//
//void ims::BFS::reconstructPath(std::vector<state*>& path) {
//    state* state_ = m_goal;
//    while (state_->getParentInd() != -1){
//        path.push_back(state_);
//        state_ = getState(state_->getParentInd());
//    }
//    path.push_back(state_);
//    std::reverse(path.begin(), path.end());
//}
//
//bool ims::BFS::isGoalState(const ims::state &s) {
//    if (s.getStateId() == m_goal->getStateId())
//        return true;
//    else
//        return false;
//}
//
//void ims::BFS::setStateVals(state* state_, state* parent, double cost)
//{
//    state_->setParent(parent->getStateId());
//    state_->f = cost;
//}
