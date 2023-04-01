//
// Created by itamar on 3/29/23.
//

#include <planners/BestFirstSearch.hpp>


ims::BestFirstSearch::BestFirstSearch(const BestFirstSearchParams &params) : Planner(params) {
    m_heuristicFunction = params.m_heuristicFunction;
}

void ims::BestFirstSearch::initializePlanner(std::shared_ptr<actionSpace>& actionSpacePtr,
                                             stateType start, stateType goal) {
    // space pointer
    m_actionSpacePtr = actionSpacePtr;
    // Evaluate the start state
    m_start->setParent(START);
    m_start->setState(start);
    m_start->g = 0;
    m_start->f = computeHeuristic(*m_start);
    m_open.push(m_start);
    m_start->setOpen();
    // Evaluate the goal state
    m_goal->setParent(GOAL);
    m_goal->setState(goal);
}

ims::state *ims::BestFirstSearch::getState(size_t state_id) {
    return m_actionSpacePtr->getState(state_id);
}

double ims::BestFirstSearch::computeHeuristic(ims::state &s) {
    return computeHeuristic(s, *m_goal);
}

double ims::BestFirstSearch::computeHeuristic(ims::state &s1, ims::state &s2) {
    return m_heuristicFunction(s1.getState(), s2.getState());
}


bool ims::BestFirstSearch::plan(std::vector<state*>& path) {
    startTimer();
    while (!m_open.empty() || !isTimeOut()){
        state* state  = m_open.min();
        m_open.pop();
        state->setClosed();
        if (isGoalState(*state)){
            m_goal = state;
            getTimeFromStart(m_stats.time);
            reconstructPath(path);
            return true;
        }
        expand(state);
    }
    getTimeFromStart(m_stats.time);
    return false;
}

void ims::BestFirstSearch::expand(ims::state* state_){
    std::vector<state*> successors;
    std::vector<double> costs; // In this case we use the "cost" as the new f value
    m_actionSpacePtr->getSuccessors(state_->getStateId(), successors, costs);
    for (size_t i {0} ; i < successors.size() ; ++i){
        state* successor = successors[i];
        double cost = costs[i];
        if (successor->isClosed()){
            continue;
        }
        if (successor->isOpen()){
            if (successor->f > cost){
                successor->setParent(state_->getStateId());
                successor->f = cost;
                m_open.update(successor);
            }
        } else {
            setStateVals(successor, state_, cost);
            m_open.push(successor);
            successor->setOpen();
        }
    }
}

void ims::BestFirstSearch::reconstructPath(std::vector<state*>& path) {
    state* state_ = m_goal;
    while (state_->getParentInd() != -1){
        path.push_back(state_);
        state_ = getState(state_->getParentInd());
    }
    path.push_back(state_);
    std::reverse(path.begin(), path.end());
}

bool ims::BestFirstSearch::isGoalState(const ims::state &s) {
    if (s.getStateId() == m_goal->getStateId())
        return true;
    else
        return false;
}

void ims::BestFirstSearch::setStateVals(state* state_, state* parent, double cost)
{
    state_->setParent(parent->getStateId());
    state_->f = cost;
}


