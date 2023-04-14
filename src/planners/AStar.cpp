//
// Created by itamar on 3/30/23.
//

#include "planners/AStar.hpp"


ims::AStar::AStar(const ims::AStarParams &params) : BestFirstSearch(params) {}

void ims::AStar::initializePlanner(const std::shared_ptr<actionSpace>& actionSpacePtr,
                                   const stateType& start, const stateType& goal) {
    // space pointer
    m_actionSpacePtr = actionSpacePtr;
    // check if start is valid
    if (!m_actionSpacePtr->isStateValid(start)){
        throw std::runtime_error("Start state is not valid");
    }
    // check if goal is valid
    if (!m_actionSpacePtr->isStateValid(goal)){
        throw std::runtime_error("Goal state is not valid");
    }
    int m_start_ind = m_actionSpacePtr->getOrCreateState(start);
    printf("start ind: %d \n", m_start_ind);
    m_start = m_actionSpacePtr->getState(m_start_ind);
    m_start->setParent(START);
    int m_goal_ind = m_actionSpacePtr->getOrCreateState(goal);
    m_goal = m_actionSpacePtr->getState(m_goal_ind);
    m_goal->setParent(GOAL);

    // Evaluate the start state
    m_start->g = 0;
    m_start->h = computeHeuristic(m_start);
    m_start->f = m_start->g + m_start->h;
    m_open.push(m_start);
    m_start->setOpen();
    // Evaluate the goal state
    m_goal->h = 0;
    m_heuristic->setGoal(m_goal);
}


void ims::AStar::expand(ims::state* state_){
    std::vector<state*> successors;
    std::vector<double> costs;
    m_actionSpacePtr->getSuccessors(state_->getStateId(), successors, costs);
    for (size_t i {0} ; i < successors.size() ; ++i){
        state* successor = successors[i];
        double cost = costs[i];
        if (successor->isClosed()){
            continue;
        }
        if (successor->isOpen()){
            if (successor->g > state_->g + cost){
                successor->setParent(state_->getStateId());
                successor->g = state_->g + cost;
                successor->f = successor->g + successor->h;
                m_open.update(successor);
            }
        } else {
            setStateVals(successor, state_, cost);
            m_open.push(successor);
            successor->setOpen();
        }
    }
}


void ims::AStar::setStateVals(state* state_, state* parent, double cost)
{
    state_->setParent(parent->getStateId());
    state_->g = parent->g + cost;
    state_->h = computeHeuristic(state_);
    state_->f = state_->g + state_->h;
}


