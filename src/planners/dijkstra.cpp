//
// Created by itamar on 3/30/23.
//

#include "planners/dijkstra.hpp"


ims::dijkstra::dijkstra(const dijkstraParams &params) : AStar(params) {}

bool ims::dijkstra::exhaustPlan() {
    startTimer();
    int iter {0};
    while (!open_.empty() && !isTimeOut()){
        // report progress every 1000 iterations
//        if (iter % 50000 == 0){
//            std::cout << "open size: " << open_.size() << std::endl;
//        }
       State* state  = open_.min();
        open_.pop();
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
        getTimeFromStart(stats_.time);
        std::cout << "Time: " << stats_.time << std::endl;
        return true;
    }

};
