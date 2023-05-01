//
// Created by itamar on 3/29/23.
//

#include <planners/BestFirstSearch.hpp>


ims::BestFirstSearch::BestFirstSearch(const BestFirstSearchParams &params) : Planner(params) {
    m_heuristic = params.m_heuristic;
}

void ims::BestFirstSearch::initializePlanner(const std::shared_ptr<actionSpace>& actionSpacePtr,
                                             const stateType& start, const stateType& goal) {
    // space pointer
    m_actionSpacePtr = actionSpacePtr;
    int m_start_ind = m_actionSpacePtr->getOrCreateState(start);
    m_start = m_actionSpacePtr->getState(m_start_ind);

    int m_goal_ind = m_actionSpacePtr->getOrCreateState(start);
    m_goal = m_actionSpacePtr->getState(m_goal_ind);
    // Evaluate the start state
    m_start->setParent(PARENT_TYPE(START));
    m_start->setState(start);
    m_start->g = 0;
    m_start->f = computeHeuristic(m_start);
    m_open.push(m_start);
    m_start->setOpen();
    // Evaluate the goal state
    m_goal->setParent(PARENT_TYPE(GOAL));
    m_goal->setState(goal);
    m_heuristic->setGoal(m_goal);
}

ims::state *ims::BestFirstSearch::getState(size_t state_id) {
    return m_actionSpacePtr->getState(state_id);
}

double ims::BestFirstSearch::computeHeuristic(ims::state* s) {
    return computeHeuristic(s, m_goal);
}

double ims::BestFirstSearch::computeHeuristic(ims::state* s1, ims::state* s2) {
    double dist;
    if (!m_heuristic->getHeuristic(s1, s2, dist))
        throw std::runtime_error("Heuristic function failed");
    else
        return dist;
}


bool ims::BestFirstSearch::plan(std::vector<state*>& path) {
    startTimer();
    int iter {0};
    while (!m_open.empty() && !isTimeOut()){
        // report progress every 1000 iterations
        if (iter % 100000 == 0){
            std::cout << "open size: " << m_open.size() << std::endl;
        }
        state* state  = m_open.min();
        m_open.pop();
        state->setClosed();
        if (isGoalState(*state)){
            m_goal = state;
            getTimeFromStart(m_stats.time);
            reconstructPath(path);
            m_stats.cost = m_goal->g;
            m_stats.pathLength = (int)path.size();
            m_stats.numGenerated = (int)m_actionSpacePtr->m_states.size();
            return true;
        }
        expand(state);
        ++iter;
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
    m_stats.numExpanded++;
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


