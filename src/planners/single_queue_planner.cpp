#include <search/planners/single_queue_planner.hpp>

namespace ims {

SingleQueuePlanner::~SingleQueuePlanner() {
    resetPlanningData();
}

void SingleQueuePlanner::initializePlanner(const std::shared_ptr<ActionSpace> &action_space_ptr,
                                    const std::vector<StateType> &starts,
                                    const std::vector<StateType> &goals) {
    // space pointer
    action_space_ptr_ = action_space_ptr;
    // Clear both.
    action_space_ptr_->resetPlanningData();
    resetPlanningData();
    main_queue_->clear();

    if (goals.empty() || starts.empty()) {
        throw std::runtime_error("Starts or goals are empty");
    }
    if (goals.size() > 1) {
        throw std::runtime_error("Currently, only one goal is supported");
    }
    // check if goal is valid
    if (!action_space_ptr_->isStateValid(goals[0])){
        throw std::runtime_error("Goal state is not valid");
    }
    int goal_robot_state_id = action_space_ptr_->getOrCreateRobotState(goals[0]);
    goals_.push_back(goal_robot_state_id);
    // GenericSearchState* goal_ = settings_->createNewSearchState(goal_robot_state_id, nullptr, 0);
    // addNewSearchState(goal_);
    // goals_.push_back(goal_->robot_state_id);
    // goal_->parent_id = PARENT_TYPE(GOAL);


    for (const StateType &start : starts) {
        // check if start is valid
        if (!action_space_ptr_->isStateValid(start)){
            throw std::runtime_error("Start state is not valid");
        }
        // Evaluate the start state
        int start_robot_state_ind_ = action_space_ptr_->getOrCreateRobotState(start);
        GenericSearchState* start_ss = settings_->createNewSearchState(start_robot_state_ind_, nullptr, 0);
        addNewSearchState(start_ss);
        start_ss->parent_id = PARENT_TYPE(START);
        start_ss->g = 0;
        main_queue_->push(start_ss);
    }
}

bool SingleQueuePlanner::plan(std::vector<StateType>& path) {
    startTimer();
    int iter {0};
    settings_->updateQueue();
    while (!main_queue_->empty() && !isTimeOut()){
        // report progress every 1000 iterations
        if (iter % 100 == 0){
            std::cout << "open size: " << main_queue_->size() << std::endl;
        }
        GenericSearchState* state = main_queue_->min();
        main_queue_->pop();
        // Skip if we have expanded better version of this search state
        if (settings_->skipAsAlreadyExpanded(state)) {
            continue;
        } else {
            settings_->addToExpanded(state);
        }

        if (isGoalState(state)){
            // getTimeFromStart(stats_.time);
            reconstructPath(state->search_id, path);
            // stats_.cost = state->g;
            // stats_.path_length = (int)path.size();
            // stats_.num_generated = (int)action_space_ptr_->states_.size();
            return true;
        }
        expand(state->search_id);
        settings_->updateQueue();
        ++iter;
    }
    if (main_queue_->empty()){
        std::cout << "Queue is empty" << std::endl;
    } else {
        std::cout << "Time out" << std::endl;
    }
    // getTimeFromStart(stats_.time);
    return false;
}


//////////////////////////////////////////////////////////////////////////
///////////// Not to be overridden by derived classes ////////////////////

void SingleQueuePlanner::expand(int search_id){
    auto state = getSearchState(search_id);
    std::vector<int> successors;
    std::vector<double> costs; 
    action_space_ptr_->getSuccessors(state->robot_state_id, successors, costs);
    for (size_t i {0} ; i < successors.size() ; ++i){
        int robot_state_id = successors[i];
        double cost = costs[i];
        GenericSearchState* successor = settings_->createNewSearchState(robot_state_id, state, cost);
        
        // Skip if we have expanded better version of this search state
        if (settings_->skipAsAlreadyExpanded(successor)) {
            delete successor; // Remove as don't need, benefit of separate addSearchState()
            continue;
        }
        else {
            addNewSearchState(successor);
            main_queue_->push(successor);
        }
    }
    // stats_.num_expanded++;
}


void SingleQueuePlanner::reconstructPath(int goal_id, vector<StateType>& path) {
    GenericSearchState* state_ = getSearchState(goal_id);
    while (state_->parent_id != -1){
        path.push_back(action_space_ptr_->getRobotState(state_->robot_state_id)->state);
        state_ = getSearchState(state_->parent_id);
    }
    path.push_back(action_space_ptr_->getRobotState(state_->robot_state_id)->state);
    std::reverse(path.begin(), path.end());
}


void SingleQueuePlanner::resetPlanningData(){
    for (auto state_ : states_){
        delete state_;
    }
    states_.clear();
    if (main_queue_ != nullptr) {
        main_queue_->clear();
    }
    goals_.clear();
    // stats_ = PlannerStats();
}

SingleQueuePlanner::GenericSearchState* SingleQueuePlanner::getSearchState(int state_id) {
    assert(state_id < states_.size() && state_id >= 0);
    return states_[state_id];
}

void SingleQueuePlanner::addNewSearchState(GenericSearchState* search_state) {
    search_state->setSearchId(states_.size());
    states_.push_back(search_state);
}

// SingleQueuePlanner::GenericSearchState* SingleQueuePlanner::createNewSearchState(int robot_state_id,
//                                         GenericSearchState* parent, double cost) {
//     GenericSearchState* succ;
//     if (parent == nullptr) {
//         succ = new GenericSearchState(/*robot_state_id*/ robot_state_id, 
//                                 /*search_id=*/ states_.size(), /*parent_id=*/ PARENT_TYPE(UNSET), 
//                                 /*g=*/-1);
//     } else {
//         succ = new GenericSearchState(/*robot_state_id*/ robot_state_id, 
//                                 /*search_id=*/ states_.size(), /*parent_id=*/ parent->search_id, 
//                                 /*g=*/parent->g + cost);
//     }
//     // Populate here, specific per application. Requires casting here.
//     states_.push_back(succ);
//     return succ;
// }


} // namespace ims