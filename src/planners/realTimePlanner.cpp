#include <search/planners/realTimePlanner.hpp>

using namespace ims;

ims::realTimePlanner::realTimePlanner(const realTimePlannerParams &params) : params_(params), Planner(PlannerParams()){
    heuristic_ = params.heuristic_;
    params_.time_limit_ = 10000;
}

ims::realTimePlanner::~realTimePlanner() {
    for (auto &state : states_){
        delete state;
    }
}

void ims::realTimePlanner::initializePlanner(const std::shared_ptr<ActionSpace> &action_space_ptr,
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
        start_->setOpen();
    }

}


void ims::realTimePlanner::initializePlanner(const std::shared_ptr<ActionSpace>& action_space_ptr,
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

auto ims::realTimePlanner::getSearchState(int state_id) -> ims::realTimePlanner::SearchState * {
    return states_[state_id];
}

auto ims::realTimePlanner::getOrCreateSearchState(int state_id) -> ims::realTimePlanner::SearchState *{
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
std::string ims::realTimePlanner::budgetedPopulateOpen(SearchState* start, std::vector<StateType>& path){
    int count = 0;
    int iter {0};
    SearchState* state;
    open_.clear();
    open_.push(start);

    while (!open_.empty() && count < params_.N_){
        state = open_.min();
        open_.pop();
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
auto ims::realTimePlanner::executePartialPlan(SearchState* start, std::vector<StateType>& path){
    int sID = open_.min()->state_id;
    constructPartialPath(sID,start->state_id,path);
    close_.clear();
    return getSearchState(sID);
}


/// @brief plan for a path
/// @param path the solution of the path
/// @return if a solution was found
bool ims::realTimePlanner::plan(std::vector<StateType>& path){
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
        if (params_.rtaa_){
            updateHeuristicRTAA();
        }else{
            updateHeuristicLRTA();
        }
        start = executePartialPlan(start, path);
    }
    getTimeFromStart(stats_.time);
    return false;
}


/// @brief update the heuristic of visited states in the dictionary using the RTAA* algorithm
void ims::realTimePlanner::updateHeuristicRTAA(){
    OpenList temp;
    SearchState *min_state = open_.min();
    double min_cost = min_state->f;
    for (size_t i = 0; i < close_.size(); i++){
        heuristicDict[close_[i]] = min_cost - close_[i]->g;
    }
}

/// @brief update the heuristic of visited states in the dictionary using the LRTA* algorithm
void ims::realTimePlanner::updateHeuristicLRTA(){
    std::map<SearchState*, double> closed_heu;
    for (int i = close_.size()-1; i >= 0; i--) {
        closed_heu[close_[i]] = INF_DOUBLE;
    }
    std::map<SearchState*, double> h_value_record;
    while (h_value_record != closed_heu) {      //add a link to documentation
        h_value_record = closed_heu;
        for (int i = close_.size()-1; i >= 0; i--) {
            std::vector<double> f_neighbours;
            std::vector<int> successors;
            std::vector<double> costs;
            SearchState* curr = close_[i];
            action_space_ptr_->getSuccessors(curr->state_id, successors, costs);
            for (size_t i = 0; i < successors.size(); i++){
                int s_next = successors[i];
                if (std::find(close_.begin(), close_.end(), getOrCreateSearchState(s_next)) == close_.end()){
                    f_neighbours.push_back(costs[i] + computeHeuristic(s_next));
                }
                else{
                    double tmp = costs[i] + closed_heu[getOrCreateSearchState(s_next)];
                    f_neighbours.push_back(tmp);
                }
            }
            closed_heu[curr] = *std::min_element(f_neighbours.begin(), f_neighbours.end());
        }
    }
    for (int i = close_.size()-1; i >= 0; i--){
        heuristicDict[close_[i]] = closed_heu[close_[i]];
    }
}

/// @brief construct the path from current iteration of budgeted planning
/// @param state_id the starting position of the current plan
/// @param target_id the end position of the current plan
/// @param path the path that the robot took updated cumulatively
void ims::realTimePlanner::constructPartialPath(int state_id, int target_id, std::vector<StateType>& path){
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
void ims::realTimePlanner::expand(int state_id){
    auto state_ = getSearchState(state_id);
    close_.push_back(state_);
    std::vector<int> successors;
    std::vector<double> costs;
    action_space_ptr_->getSuccessors(state_->state_id, successors, costs);
    for (size_t i = 0; i < successors.size(); i++){
        int successor_id = successors[i];
        double cost = costs[i];
        auto successor = getOrCreateSearchState(successor_id);
        if (std::find(close_.begin(), close_.end(),successor) != close_.end()){
            continue;
        }
        if (isGoalState(successor_id) && params_.verbose ){
            std::cout << "Added Goal to open list" << std::endl;
        }
        if (open_.contains(successor)){
           if (successor->g > state_->g + cost){
                successor->parent_id = state_->state_id;
                successor->g = state_->g + cost;
                successor->f = successor->g + successor->h;
                open_.update(successor);
            }
        }else{
            setSearchStateVals(successor->state_id, state_->state_id, cost);
            open_.push(successor);
            successor->setOpen();
        }
    }
    stats_.num_expanded++;
}

void ims::realTimePlanner::setSearchStateVals(int state_id, int parent_id, double cost){
    auto state_ = getSearchState(state_id);
    auto parent = getSearchState(parent_id);
    state_->parent_id = parent_id;
    state_->g = parent->g + cost;
    state_->h = computeHeuristic(state_id);
    state_->f = state_->g + state_->h;
}

double ims::realTimePlanner::computeHeuristic(int state_id) {
    double dist;
    auto s = action_space_ptr_->getRobotState(state_id);
    auto look_up = getSearchState(state_id);
    if (heuristicDict.find(look_up) == heuristicDict.end()){
        if (!heuristic_->getHeuristic(s->state, dist))
            throw std::runtime_error("Heuristic function failed");
        else
            heuristicDict[look_up] = dist;
            return dist;
    }else{
        return heuristicDict.at(look_up);
    }
}


void ims::realTimePlanner::reconstructPath(std::vector<StateType>& path) {
    SearchState* state = getSearchState(goal_);
    while (state->parent_id != -1){
        path.push_back(action_space_ptr_->getRobotState(state->state_id)->state);
        state = getSearchState(state->parent_id);
    }
    path.push_back(action_space_ptr_->getRobotState(state->state_id)->state);
    std::reverse(path.begin(), path.end());
}

bool ims::realTimePlanner::isGoalState(int state_id){
    return std::any_of(goals_.begin(), goals_.end(), [&state_id](int goal_ind) {return state_id == goal_ind;});
}

void ims::realTimePlanner::resetPlanningData(){
    for (auto state_ : states_){
        delete state_;
    }
    states_.clear();
    open_.clear();
    close_.clear();
    goals_.clear();
    goal_ = -1;
    stats_ = PlannerStats();
}

