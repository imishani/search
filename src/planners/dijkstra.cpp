//
// Created by itamar on 3/30/23.
//

#include "planners/dijkstra.hpp"


ims::dijkstra::dijkstra(const dijkstraParams &params) : AStar(params) {}

bool ims::dijkstra::exhaustPlan() {
    startTimer();
    int iter {0};
    while (!m_open.empty() && !isTimeOut()){
        // report progress every 1000 iterations
//        if (iter % 50000 == 0){
//            std::cout << "open size: " << m_open.size() << std::endl;
//        }
        state* state  = m_open.min();
        m_open.pop();
        expand(state);
        iter++;
    }
    if (isTimeOut()){
        std::cout << "Time out!" << std::endl;
        return false;
    }
    else{
        std::cout << "Open got empty!" << std::endl;
        // report stats
        getTimeFromStart(m_stats.time);
        std::cout << "Time: " << m_stats.time << std::endl;
        return true;
    }

};
