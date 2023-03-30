//
// Created by itamar on 3/29/23.
//

#include <planners/BestFirstSearch.hpp>


ims::BestFirstSearch::BestFirstSearch(const BestFirstSearchParams &params) : Planner(params) {
    m_heuristicFunction = params.m_heuristicFunction;
}

void ims::BestFirstSearch::initializePlanner(stateType start, stateType goal) {
    // Evaluate the start state
    m_start->setState(start);
    m_start->g = 0;
    m_start->h = computeHeuristic(*m_start);
    m_start->f = m_start->g + m_start->h;
}

double ims::BestFirstSearch::computeHeuristic(ims::state &s) {
    return computeHeuristic(s, *m_goal);
}

double ims::BestFirstSearch::computeHeuristic(ims::state &s1, ims::state &s2) {
    return m_heuristicFunction(s1.getState(), s2.getState());
}


bool ims::BestFirstSearch::plan() {
    m_open.clear();
    m_open.push(m_start);


}