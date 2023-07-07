//
// Created by itamar on 3/30/23.
//

#include <search/planners/dijkstra.hpp>


ims::dijkstra::dijkstra(const dijkstraParams &params) : AStar(params) {}

auto ims::dijkstra::getSearchState(size_t state_id) -> ims::dijkstra::SearchState * {
    assert(state_id < states_.size() && state_id >= 0);
    return states_[state_id];
}

bool ims::dijkstra::exhaustPlan() {
    startTimer();
    int iter {0};
    while (!open_.empty() && !isTimeOut()){
        // report progress every 1000 iterations
//        if (iter % 50000 == 0){
//            std::cout << "open size: " << open_.size() << std::endl;
//        }
        auto state  = open_.min();
        open_.pop();
        expand(state->state_id);
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

}
