#include <search/planners/lss_lrta.hpp>

using namespace ims;

ims::lssLRTA::lssLRTA(const lssLRTAPlannerParams &params) : realTimePlanner(realTimePlannerParams(params.heuristic_, params.N_, false)),
                                                            params_(params){
    heuristic_ = params.heuristic_;
}

ims::lssLRTA::~lssLRTA() {
    for (auto &state : states_){
        delete state;
    }
}

void ims::lssLRTA::initializePlanner(const std::shared_ptr<ActionSpace> &action_space_ptr,
                                             const std::vector<StateType> &starts,
                                             const std::vector<StateType> &goals) {
    action_space_ptr_ = action_space_ptr;

    action_space_ptr_->resetPlanningData();
    resetPlanningData();
    if (goals.empty() || starts.empty()) {
        throw std::runtime_error("Starts or goals are empty");
    }

    if (goals.size() > 1) {
        throw std::runtime_error("Currently, only one goal is supported");
    }

    int goal_ind_ = action_space_ptr_->getOrCreateRobotState(goals[0]);
    auto goal_ = getOrCreateSearchState(goal_ind_);
    goals_.push_back(goal_ind_);

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
        open_H.push(start_);
        start_->setOpen();
    }

}


void ims::lssLRTA::initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
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
    open_H.push(start_);
    start_->setOpen();
                                             

}

auto ims::lssLRTA::getSearchState(int state_id) -> ims::lssLRTA::SearchState * {
    return states_[state_id];
}

auto ims::lssLRTA::getOrCreateSearchState(int state_id) -> ims::lssLRTA::SearchState *{
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


/// @brief given a budget of N, expand N states from start
/// @param start start state of the agent
/// @param path the path that the robot took updated cumulatively
/// @return "found" if a solution is found, "failed" if there is no solution, "continue" if the current plan is not finished
std::string ims::lssLRTA::budgetedPopulateOpen(SearchState* start, std::vector<StateType>& path){
    int count = 0;
    int iter {0};
    SearchState* state;
    open_.clear();
    open_H.clear();
    open_.push(start);
    open_H.push(start);

    while (!open_.empty() && count < params_.N_){
        state = open_.min();
        open_.pop();
        open_H.erase(state);
        state->setClosed();
        if (isGoalState(state->state_id)){
            goal_ = state->state_id;
            getTimeFromStart(stats_.time);
            constructPartialPath(goal_,start->state_id,path);
            path.push_back(action_space_ptr_->getRobotState(goal_)->state);
            stats_.cost = state->g;
            stats_.path_length = (int)path.size();
            stats_.num_generated = (int)action_space_ptr_->states_.size();
            return "found";
        }
        expand(state->state_id);
        ++iter;
        count++;
    }
    if (open_.empty() ){
        return "failed";
    }
    return "continue";
}


/// @brief moves onto the most promising adjacent state and construct the current partial path
/// @param start the starting state of the current iteration of budgeted plan
/// @param path the path that the robot took updated cumulatively
/// @return the current state of the agent after execution
auto ims::lssLRTA::executePartialPlan(SearchState* start, std::vector<StateType>& path){
    int sID = open_.min()->state_id;
    constructPartialPath(sID,start->state_id,path);
    close_.clear();
    return getSearchState(sID);
}


/// @brief plan for a path
/// @param path the solution of the path
/// @return if a solution was found
bool ims::lssLRTA::plan(std::vector<StateType>& path){
    startTimer();
    SearchState *start = open_.min();
    std::string res = "continue";
    int count = 0;
    while (!isTimeOut() && res == "continue"){
        res = budgetedPopulateOpen(start,path);
        count++;
        if (res == "failed"){
            break;
        }else if (res == "found"){
            return true;
        }
        updateHeuristicLRTA();
        start = executePartialPlan(start, path);
    }
    getTimeFromStart(stats_.time);
    return false;
}


/// @brief update the heuristic of visited states in the dictionary using the LRTA* algorithm
void ims::lssLRTA::updateHeuristicLRTA(){ //naming change
    for (SearchState *s : close_){
        heuristic_dict_[s] = INF_DOUBLE;
    }
    while (!open_H.empty()){
        SearchState* min_state = open_H.min();
        open_H.pop();
        auto pos = close_.end();
        if ((pos = close_.find(min_state)) != close_.end()) {
            close_.erase(pos);
        }

        std::vector<std::tuple<SearchState*, double>> parents = parent_[min_state];

        for (size_t i = 0; i < parents.size(); i++){
            SearchState* s_parent = get<0>(parents[i]);
            double s_heu = computeHeuristic(min_state->state_id);
            double sp_heu = computeHeuristic(s_parent->state_id);
            if (close_.find(s_parent) != close_.end() && sp_heu > get<1>(parents[i]) + s_heu){
                heuristic_dict_[s_parent] = get<1>(parents[i]) + s_heu;
                auto sp_state = getOrCreateSearchState(s_parent->state_id);
                if (!open_H.contains(sp_state)){
                    open_H.push(sp_state);
                }
            }
        }
    }
    //copy queue for open_H to open_
}

//void updateHeuristicLRTA_K(int k){
//    std::queue<SearchState*> Q;
//    k -= 1;
//    while (!Q.empty()){
//        SearchState* curr = Q.pop();
//        std::vector<int> successors;
//        std::vector<double> costs;
//        action_space_ptr_->getSuccessors(curr->state_id, successors, costs);
//        double min = INF_DOUBLE;
//        for (size_t i = 0; i < successors.size(); i++){
//            int s_next = successors[i];
//            double y = costs[i] + computeHeuristic(s_next);
//        }
//        if (computeHeuristic(curr) < min){
//            heuristic_dict_[curr] = min;
//            for (size_t i = 0; i < successors.size(); i++){
//                SearchState* s_next = getOrCreateSearchState(successors[i]);
//                if (k > 0){
//                    Q.push(s_next);
//                    k -= 1;
//                }
//            }
//        }
//    }
//}

/// @brief construct the path from current iteration of budgeted planning
/// @param state_id the starting position of the current plan
/// @param target_id the end position of the current plan
/// @param path the path that the robot took updated cumulatively
void ims::lssLRTA::constructPartialPath(int state_id, int target_id, std::vector<StateType>& path){
    int preLen = (int) path.size();
    auto start = getSearchState(state_id);
    while (start->state_id != target_id){
        path.push_back(action_space_ptr_->getRobotState(start->state_id)->state);
        start = getSearchState(start->parent_id);
    }
    std::reverse(path.begin()+preLen, path.end());

}

/// @brief expand the neighbors of the current state by updating the close and open list
/// @param state_id current state to be expanded
void ims::lssLRTA::expand(int state_id){
    auto state = getSearchState(state_id);
    close_.insert(state);
    std::vector<int> successors;
    std::vector<double> costs;
    action_space_ptr_->getSuccessors(state->state_id, successors, costs);
    for (size_t i = 0; i < successors.size(); i++){
        int successor_id = successors[i];
        double cost = costs[i];
        auto successor = getOrCreateSearchState(successor_id);
        // if (!parent_.contains(successor)){ 
        //     parent_[successor] = std::vector<SearchState*>();
        // }
        parent_[successor].emplace_back(state, cost);     //consider changing this to map
        if (close_.find(successor) != close_.end()){
            continue;
        }
        if (isGoalState(successor_id) && params_.verbose ){
            std::cout << "Added Goal to open list" << std::endl;
        }
        if (open_.contains(successor)){
           if (successor->g > state->g + cost){
                successor->parent_id = state->state_id;
                successor->g = state->g + cost;
                successor->f = successor->g + successor->h;
                open_.update(successor);
                open_H.update(successor);
            }
        }else{
            setSearchStateVals(successor->state_id, state->state_id, cost);
            open_.push(successor);
            open_H.push(successor);
            successor->setOpen();
        }
    }
    stats_.num_expanded++;
}

void ims::lssLRTA::setSearchStateVals(int state_id, int parent_id, double cost){
    auto state_ = getSearchState(state_id);
    auto parent = getSearchState(parent_id);
    state_->parent_id = parent_id;
    state_->g = parent->g + cost;
    state_->h = computeHeuristic(state_id);
    state_->f = state_->g + state_->h;
}

double ims::lssLRTA::computeHeuristic(int state_id) {
    double dist;
    auto s = action_space_ptr_->getRobotState(state_id);
    auto look_up = getSearchState(state_id);
    if (heuristic_dict_.find(look_up) == heuristic_dict_.end()){
        if (!heuristic_->getHeuristic(s->state, dist))
            throw std::runtime_error("Heuristic function failed");
        else
            heuristic_dict_[look_up] = dist;
            return dist;
    }else{
        return heuristic_dict_.at(look_up);
    }
}


void ims::lssLRTA::reconstructPath(std::vector<StateType>& path) {
    SearchState* state = getSearchState(goal_);
    while (state->parent_id != -1){
        path.push_back(action_space_ptr_->getRobotState(state->state_id)->state);
        state = getSearchState(state->parent_id);
    }
    path.push_back(action_space_ptr_->getRobotState(state->state_id)->state);
    std::reverse(path.begin(), path.end());
}

bool ims::lssLRTA::isGoalState(int state_id){
    return std::any_of(goals_.begin(), goals_.end(), [&state_id](int goal_ind) {return state_id == goal_ind;});
}

void ims::lssLRTA::resetPlanningData(){
    for (auto state_ : states_){
        delete state_;
    }
    states_.clear();
    open_.clear();
    open_H.clear();
    close_.clear();
    goals_.clear();
    heuristic_dict_.clear();
    goal_ = -1;
    stats_ = PlannerStats();
}

