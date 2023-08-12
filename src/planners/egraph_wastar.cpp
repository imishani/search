//
// Created by itamar on 8/7/23.
//

#include "search/planners/egraph_wastar.hpp"

ims::ExperienceWAstar::ExperienceWAstar(const ims::ExperienceWAStarParams &params) : wAStar(params), params_(params) {
}

void ims::ExperienceWAstar::initializePlanner(const std::shared_ptr<ActionSpace> &action_space_ptr,
                                              const std::vector<StateType> &starts,
                                              const std::vector<StateType> &goals) {
    if (goals.empty() || starts.empty()) {
        throw std::runtime_error("Starts or goals are empty");
    }

    if (goals.size() > 1) {
        throw std::runtime_error("Currently, only one goal is supported");
    }

    std::cout << RED << "[ERROR]: Experience planner with multiple starts is not implemented yet!" << std::endl;
    std::cout << "      Assuming goal and start to be the first element is the vector " << RESET << std::endl;
    initializePlanner(action_space_ptr, starts[0], goals[0]);
}

void ims::ExperienceWAstar::initializePlanner(const std::shared_ptr<ActionSpace> &action_space_ptr,
                                              const StateType &start,
                                              const StateType &goal) {
    // TODO: Check if its ok:
    egraph_action_space_ptr_ = std::dynamic_pointer_cast<EGraphActionSpace>(action_space_ptr);
    action_space_ptr_ = egraph_action_space_ptr_;
    // Clear both.
    egraph_action_space_ptr_->resetPlanningData();
    resetPlanningData();

    // check if start is valid
    if (!egraph_action_space_ptr_->isStateValid(start)){
        throw std::runtime_error("Start state is not valid");
    }
    // check if goal is valid
    if (!egraph_action_space_ptr_->isStateValid(goal)){
        throw std::runtime_error("Goal state is not valid");
    }
    int start_ind_ = egraph_action_space_ptr_->getOrCreateRobotState(start);
    auto start_ = getOrCreateSearchState(start_ind_);

    int goal_ind_ = egraph_action_space_ptr_->getOrCreateRobotState(goal);
    auto goal_ = getOrCreateSearchState(goal_ind_);
    goals_.push_back(goal_ind_);

    // load the experience graph
    egraph_action_space_ptr_->loadEGraph(params_.experiences_dir);

    start_->parent_id = PARENT_TYPE(START);
    goal_->parent_id = PARENT_TYPE(GOAL);

    syncStatesCreated();

    heuristic_->setStart(const_cast<StateType &>(start));
    // Evaluate the goal state
    heuristic_->setGoal(const_cast<StateType &>(goal));
    goal_->h = 0;
    // Evaluate the start state
    start_->g = 0;
    start_->h = computeHeuristic(start_ind_);
    start_->f = start_->g + params_.epsilon*start_->h;
    start_->setOpen();

    open_.push(start_);
    // update stats suboptimality
    stats_.suboptimality = params_.epsilon;
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
            if (path.empty()){
                std::cout << RED << "[ERROR]: Path size is 0. Something is wrong" << RESET << std::endl;
                return false;
            }
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
    std::reverse(search_path.begin(), search_path.end());
    extractPath(search_path, path);
}

void ims::ExperienceWAstar::extractPath(const std::vector<int> &search_path, PathType &path) {
    PathType temp_path;
    // check for edge cases
    if (search_path.empty()){
        return;
    }
    if (search_path.size() == 1){
        temp_path.push_back(action_space_ptr_->getRobotHashEntry(search_path[0])->state);
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
    temp_path.push_back(prev_s->state);
    int prev_s_id = search_path[0];
    // loop through the path
    for (size_t i {1}; i < search_path.size(); ++i){
        int curr_id = search_path[i];

        auto curr_state = action_space_ptr_->getRobotHashEntry(curr_id);
        if (!curr_state){
            std::cout << RED << "[ERROR]: State " << curr_id << " is not in the action space!" << RESET << std::endl;
            return;
        }
        std::vector<ActionSequence> action_sequences;
        action_space_ptr_->getActions(prev_s_id, action_sequences, true);
        for (auto& action_sequence : action_sequences){
            if (curr_id == goal_){
                // check if the last point is the goal
                if (action_sequence.back() == curr_state->state){
                    temp_path.push_back(curr_state->state);
                    path = temp_path;
                    return;
                }
            } else {
                // check if the last point is the goal
                if (action_sequence.back() == curr_state->state){
                    temp_path.push_back(curr_state->state);
                    prev_s_id = curr_id;
                    break;
                }
            }
        }
        if (prev_s_id == curr_id){
            continue;
        }

        PathType shortcut_path;
        bool found_shortcut = egraph_action_space_ptr_->checkShortcutTransition(prev_s_id, curr_id, shortcut_path);
        if (found_shortcut){
            for (auto& state : shortcut_path){
                temp_path.push_back(state);
            }
            prev_s_id = curr_id;
            continue;
        }

        PathType snap_path;
        bool found_snap = egraph_action_space_ptr_->checkSnapTransition(prev_s_id, curr_id, snap_path);
        if (found_snap){
            for (auto& state : snap_path){
                temp_path.push_back(state);
            }
            prev_s_id = curr_id;
            continue;
        }
        std::cout << RED << "[ERROR]: Could not find transition from " << prev_s_id << " to " << curr_id << std::endl;
        for (size_t ind {0}; ind < prev_s->state.size() ; ++ind){
            std::cout << prev_s->state[ind] << "    " << curr_state->state[ind] << std::endl;
        }
        std::cout << RESET << std::endl;
        return;
    }

    // Awesome! We have a path!
    path = temp_path;
}

void ims::ExperienceWAstar::syncStatesCreated() {
    // Loop through all the states in the action space and create a search state for each one
    for (int ind {0}; ind < action_space_ptr_->states_.size(); ++ind){
        getOrCreateSearchState(ind);
    }
}
