//
// Created by itamar on 3/30/23.
//

#include "planners/AStar.hpp"


ims::AStar::AStar(const ims::AStarParams &params) : BestFirstSearch(params) {}

void ims::AStar::initializePlanner(std::shared_ptr<actionSpace>& actionSpacePtr,
                                             stateType start, stateType goal) {
    // space pointer
    m_actionSpacePtr = actionSpacePtr;
    // Evaluate the start state
    m_start->setParent(START);
    m_start->setState(start);
    m_start->g = 0;
    m_start->h = computeHeuristic(*m_start);
    m_start->f = m_start->g + m_start->h;
    m_open.push(m_start);
    m_start->setOpen();
    // Evaluate the goal state
    m_goal->setParent(GOAL);
    m_goal->setState(goal);
    m_goal->h = 0;
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
    state_->h = computeHeuristic(*state_);
    state_->f = state_->g + state_->h;
}


