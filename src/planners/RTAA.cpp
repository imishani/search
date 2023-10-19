#include <search/planners/RTAA.hpp>

using namespace ims;

ims::rtaaStar::rtaaStar(const realTimePlannerParams &params) : Planner(params){
    heuristic_ = params.heuristic_;
    N = params.N_;
}

ims::rtaaStar::~rtaaStar() {
    for (auto &state : states_){
        delete state;
    }
}

void ims::rtaaStar::initializePlanner(const std::shared_ptr<ActionSpace> &action_space_ptr,
                                             const std::vector<StateType> &starts,
                                             const std::vector<StateType> &goals) {
    action_space_ptr = action_space_ptr;

    action_space_ptr_->resetPlanningData();
    resetPlanningData();
    if (goals.empty() || starts.empty()) {
        throw std::runtime_error("Starts or goals are empty");
    }

    if (goals.size() > 1) {
        throw std::runtime_error("Currently, only one goal is supported");
    }

    int goal_ind = actioin_space_ptr_->getOrCreateRobotState(goals[0]);
    auto goal_ getOrCreateSearchState(goal_ind_);
    goals_.push_back(goal_ind);

    goal_->parent_id = PARENT_TYPE(GOAL);
    heuristic_->setGoal(const_cast<StateType &>(goals[0]));
    goal_->h = 0;

    for (auto &start : starts) {
        // Evaluate the start state
        int start_ind_ = action_space_ptr_->getOrCreateRobotState(start);
        auto start_ = getOrCreateSearchState(start_ind_);
        start_->parent_id = PARENT_TYPE(START);
        heuristic_->setStart(const_cast<StateType &>(start));
        start_->g = 0;
        start_->f = computeHeuristic(start_ind_);
        open_.push(start_);
        start_->setOpen();
    }

}


void ims::rtaaStar::initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
                                             const StateType& start, const StateType& goal) {
                                             
    action_space_ptr_ = action_space_ptr;

    action_space_ptr_->resetPlanningData();
    resetPlanningData();  

    if (!action_space_ptr_->isStateValid(start)){
        throw std::runtime_error("Start state is not valid");
    }
    // check if goal is valid
    if (!action_space_ptr_->isStateValid(goal)){
        throw std::runtime_error("Goal state is not valid");
    }                                 

    int start_ind_ = action_space_ptr_->getOrCreateRobotState(start);
    auto start_ = getOrCreateSearchState(start_ind_);

    int goal_ind_ = action_space_ptr_->getOrCreateRobotState(goal);
    auto goal_ = getOrCreateSearchState(goal_ind_);
    goals_.push_back(goal_ind_);

    // Evaluate the start state
    start_->parent_id = PARENT_TYPE(START);
    heuristic_->setStart(const_cast<StateType &>(start));
    // Evaluate the goal state
    goal_->parent_id = PARENT_TYPE(GOAL);
    heuristic_->setGoal(const_cast<StateType &>(goal));
    // Evaluate the start state
    start_->g = 0;
    start_->h = computeHeuristic(start_ind_);
    start_->f = computeHeuristic(start_ind_);
    open_.push(start_);
    start_->setOpen();
                                             

}

auto ims::rtaaStar::getSearchState(int state_id) -> ims::realTimePlanner::SearchState * {
    return states_[state_id];
}

auto ims::rtaaStar::getOrCreateSearchState(int state_id) -> ims::realTimePlanner *{
    if (state_id >= states_.size()){
        states_.resize(state_id + 1, nullptr);
    }
    if (states_[state_id] == nullptr){
        assert(state_id < states_.size() && state_id >= 0);
        states_[state_id] = new SearchState;
        states_[state_id]->state_id = state_id;
    }
    return states_[state_id];
}

bool ims::rtaaStar::plan(std::vector<StateType>& path){
    int count = 0;
    int iter {0};
    startTimer();
    auto start = start_;
    while (!open_.empty() && !isTimeOut()){
        auto state;
        if (count == params_.N){
            count = 0;
            int sID = updateH();
            constructCurrentPath(sID,start->state_id,path);
            close_.clear();
            state = getSearchState(sID);
            start = state;
        }else{
            state = open_.min();
            open_.pop();
        }
        state->setClosed();
        if (isGoalState(state->state_id)){
            goal_ = state->state_id;
            getTimeFromStart(stats_.time);
            constructCurrentPath(goal_,start->state_id,path);
            path.push_back(action_space_ptr_->getRobotState(goal_->state_id)->state)
            stats_.cost = state->g;
            stats_.path_length = (int)path.size();
            stats_.num_generated = (int)action_space_ptr_->states_.size();
            return true;            
        }
        expand(state->state_id);
        ++iter;
        count++;
    }
    getTimeFromStart(stats_.time);
    return false;
}

int ims::rtaaStar::updateH(){
    OpenList temp;
    auto minState = open_.min();
    double minCost = minState->f;
    // while (!open_.empty()){
    //     auto state = open_.min();
    //     open_.pop();
    //     auto parent = getSearchState(state->parent_id);
    //     double cost = parent->g + 1 + state->h;
    //     if (cost < minCost){
    //         minCost = cost;
    //         minState = state;
    //     }
    //     temp.push(state);
    // }
    // while (!temp.empty()){
    //     open_.push(temp.min());
    //     temp.pop();
    // }
    for (size_t i = 0; i < close_.size(); i++){
        close_[i]->h = minCost - close_[i]->g;
    }
    return minState->state_id;
}

void ims::rtaaStar::constructCurrentPath(int state_id, int targetID, std::vector<StateType>& path){
    std::vector<int> successors;
    std::vector<double> costs;
    int preLen = (int) path.size();
    auto start = getSearchState(state_id);
    while (start->state_id != targetID){
        double maxH = 0;
        action_space_ptr_->getSuccessors(start->state_id,successors,costs);
        for(size_t i = 0; i < successors.size(); i++){
            auto successor = getOrCreateSearchState(successors[i]);
            if (successor->in_closed && successor->h > maxH){
                maxH = h;
                start = successor;
            }
        }
        path.push_back(action_space_ptr_->getRobotState(start->state_id)->state);
    }
    std::reverse(path.begin()+preLen, path.end());
}

void ims::rtaaStar::expand(int state_id){
    auto state_ = getSearchState(state_id);
    close_.push_back(state);
    std::vector<int> successors;
    std::vector<double> costs;
    action_space_ptr_->getSuccessors(state_->state_id, successors, costs);
    for (size_t i = 0; i < successors.size(); i++){
        int successor_id = successors[i];
        double cost = costs[i];
        auto successor = getOrCreateSearchState(successor_id);
        if (successor->in_closed){
            continue;
        }
        if (isGoalState(successor_id) && params_.verbose ){
            std::cout << "Added Goal to open list" << std::endl;
        }
        if (successor->in_open){
           if (successor->g > state_->g + cost){
                successor->parent_id = state_->state_id;
                successor->g = state_->g + cost;
                successor->f = successor->g + successor->h;
                open_.update(successor);
            }else {
            setStateVals(successor->state_id, state_->state_id, cost);
            open_.push(successor);
            successor->setOpen();
        }
        }
    }
    stats_.num_expanded++;
}

void ims::rtaaStar::setStateVals(int state_id, int parent_id, double cost){
    auto state_ = getSearchState(state_id);
    auto parent = getSearchState(parent_id);
    state_->parent_id = parent_id;
    state_->g = parent->g + cost;
    state_->h = computeHeuristic(state_id);
    state_->f = state_->g + state_->h;
}

double ims::rtaaStar::computeHeuristic(int state_id) {
    double dist;
    auto s = action_space_ptr_->getRobotState(state_id);
    if (!heuristic_->getHeuristic(s->state, dist))
        throw std::runtime_error("Heuristic function failed");
    else
        return dist;
}

double ims::rtaaStar::computeHeuristic(int s1_id, int s2_id) {
    double dist;
    auto s1 = action_space_ptr_->getRobotState(s1_id);
    auto s2 = action_space_ptr_->getRobotState(s2_id);
    if (!heuristic_->getHeuristic(s1->state, s2->state, dist))
        throw std::runtime_error("Heuristic function failed");
    else
        return dist;
}

void ims::rtaaStar::reconstructPath(std::vector<StateType>& path) {
    SearchState* state = getSearchState(goal_);
    while (state->parent_id != -1){
        path.push_back(action_space_ptr_->getRobotState(state->state_id)->state);
        state = getSearchState(state->parent_id);
    }
    path.push_back(action_space_ptr_->getRobotState(state->state_id)->state);
    std::reverse(path.begin(), path.end());
}

bool ims::rtaaStar::isGoalState(int state_id){
    return std::any_of(goals_.begin(), goals_.end(), [&s_id](int goal_ind) {return s_id == goal_ind;});
}