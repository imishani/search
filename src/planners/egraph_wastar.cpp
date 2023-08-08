//
// Created by itamar on 8/7/23.
//

#include "search/planners/egraph_wastar.hpp"

ims::ExperienceWAstar::ExperienceWAstar(const ims::ExperienceWAStarParams &params) : wAStar(params), params_(params) {
}

void ims::ExperienceWAstar::initializePlanner(const std::shared_ptr<ActionSpace> &action_space_ptr,
                                              const std::vector<StateType> &starts,
                                              const std::vector<StateType> &goals) {
    egraph_action_space_ptr_ = std::dynamic_pointer_cast<ExperienceGraphActionSpace>(action_space_ptr);
    wAStar::initializePlanner(action_space_ptr, starts, goals);
}

void ims::ExperienceWAstar::initializePlanner(const std::shared_ptr<ActionSpace> &action_space_ptr,
                                              const StateType &start,
                                              const StateType &goal) {
    egraph_action_space_ptr_ = std::dynamic_pointer_cast<ExperienceGraphActionSpace>(action_space_ptr);
    wAStar::initializePlanner(action_space_ptr, start, goal);
}


bool ims::ExperienceWAstar::plan(std::vector<StateType> &path) {
    startTimer();
    int iter {0};
    while (!open_.empty() && !isTimeOut()){
        // report progress every 1000 iterations
        if (iter % 100000 == 0){
            std::cout << "Open size: " << open_.size() << std::endl;
        }
        auto state  = open_.min();
        open_.pop();
        state->setClosed();
        if (isGoalState(state->state_id)){
            goal_ = state->state_id;
            getTimeFromStart(stats_.time);
            reconstructPath(path);
            stats_.cost = state->g;
            stats_.path_length = (int)path.size();
            stats_.num_generated = (int)action_space_ptr_->states_.size();
            return true;
        }
        expand(state->state_id);
        ++iter;
    }
    getTimeFromStart(stats_.time);
    return false;
}

void ims::ExperienceWAstar::expand(int state_id) {
    auto state_ = getSearchState(state_id);
    std::vector<int> successors;
    std::vector<double> costs;
    action_space_ptr_->getSuccessors(state_->state_id, successors, costs);
    for (size_t i {0} ; i < successors.size() ; ++i){
        int successor_id = successors[i];
        double cost = costs[i];
        auto successor = getOrCreateSearchState(successor_id);
        if (successor->in_closed){
            continue;
        }
        if (isGoalState(successor_id)){
            std::cout << "Added Goal to open list" << std::endl;
        }
        if (successor->in_open){
            if (successor->g > state_->g + cost){
                successor->parent_id = state_->state_id;
                successor->g = state_->g + cost;
                successor->f = successor->g + params_.epsilon*successor->h;
                open_.update(successor);
            }
        } else {
            setStateVals(successor->state_id, state_->state_id, cost);
            open_.push(successor);
            successor->setOpen();
        }
    }
    // TODO: Add condition to check if to use experience
    std::vector<int> snap_successors;
    params_.egraph_heuristic->getEquivalentStates(state_id, snap_successors);
    for (size_t sidx {0}; sidx < snap_successors.size(); ++sidx){
        int snap_successor_id = snap_successors[sidx];
        int cost;
        if (!egraph_action_space_ptr_->snap(state_id, snap_successor_id, cost)){
            continue;
        }
        auto snap_successor = getOrCreateSearchState(snap_successor_id);
        if (snap_successor->in_closed){
            continue;
        }

        if (snap_successor->in_open){
            if (snap_successor->g > state_->g + cost){
                snap_successor->parent_id = state_->state_id;
                snap_successor->g = state_->g + cost;
                snap_successor->f = snap_successor->g + params_.epsilon*snap_successor->h;
                open_.update(snap_successor);
            }
        } else {
            setStateVals(snap_successor->state_id, state_->state_id, cost);
            open_.push(snap_successor);
            snap_successor->setOpen();
        }
    }

    // shortcut succ
    std::vector<int> shortcut_successors;
    params_.egraph_heuristic->getShortcutSuccessors(state_id, shortcut_successors);

    for (size_t sidx {0}; sidx < shortcut_successors.size(); ++sidx){
        int shortcut_successor_id = shortcut_successors[sidx];
        int cost;
        if (!egraph_action_space_ptr_->shortcut(state_id, shortcut_successor_id, cost)){
            continue;
        }
        auto shortcut_successor = getOrCreateSearchState(shortcut_successor_id);
        if (shortcut_successor->in_closed){
            continue;
        }

        if (shortcut_successor->in_open){
            if (shortcut_successor->g > state_->g + cost){
                shortcut_successor->parent_id = state_->state_id;
                shortcut_successor->g = state_->g + cost;
                shortcut_successor->f = shortcut_successor->g + params_.epsilon*shortcut_successor->h;
                open_.update(shortcut_successor);
            }
        } else {
            setStateVals(shortcut_successor->state_id, state_->state_id, cost);
            open_.push(shortcut_successor);
            shortcut_successor->setOpen();
        }
    }
    stats_.num_expanded++;
}

void ims::ExperienceWAstar::reconstructPath(std::vector<StateType> &path) {
    std::vector<int> search_path;
    SearchState* state_ = getSearchState(goal_);
    while (state_->parent_id != -1){
        search_path.push_back(state_->state_id);
        state_ = getSearchState(state_->parent_id);
    }
    search_path.push_back(state_->state_id);
    std::reverse(path.begin(), path.end());
    extractPath(search_path, path);
}

void ims::ExperienceWAstar::extractPath(const std::vector<int> &search_path, PathType &path) {
    // check for edge cases
    if (search_path.empty()){
        return;
    }
    if (search_path.size() == 1){
        path.push_back(action_space_ptr_->getRobotHashEntry(search_path[0])->state);
        std::cout << YELLOW << "[WARN]: Path has only one state" << RESET << std::endl;
        return;
    }
    if (search_path[0] == goal_){
        std::cout << RED << "[ERROR]: Goal is the first state in the path!" << RESET << std::endl;
        return;
    }

    auto prev_s = action_space_ptr_->getRobotHashEntry(search_path[0]);
    if (!prev_s){
        std::cout << RED << "[ERROR]: State " << search_path[0] << " is not in the action space!" << RESET << std::endl;
        return;
    }
    path.push_back(prev_s->state);
    int prev_s_id = search_path[0];
    // loop through the path
    for (size_t i {1}; i < search_path.size(); ++i){
        int curr_id = search_path[i];

        auto curr_state = action_space_ptr_->getRobotHashEntry(curr_id);
        if (!curr_state){
            std::cout << RED << "[ERROR]: State " << curr_id << " is not in the action space!" << RESET << std::endl;
            return;
        }
        std::vector<int> succ_ids; std::vector<double> costs;

        action_space_ptr_->getSuccessors(prev_s_id, succ_ids, costs);
        /*
         * TODO: currently we are only considering successors but we should consider
         * all actions where an action can also be a trajectory, controller or whatever.
        */
        for (size_t j {0}; j < succ_ids.size(); ++j){
            if (succ_ids[j] == curr_id){
                path.push_back(curr_state->state);
                break;
            }
        }


    }
}
