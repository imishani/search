#include <search/planners/RTAA.hpp>

using namespace ims;

ims::rtaaStar::rtaaStar(const rtaaStarParams &params) : params_(params), Planner(PlannerParams()){
    heuristic_ = params.heuristic_;
}

ims::rtaaStar::~rtaaStar() {
    for (auto &state : states_){
        delete state;
    }
}

void ims::rtaaStar::initializePlanner(const std::shared_ptr<ActionSpace> &action_space_ptr,
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

auto ims::rtaaStar::getSearchState(int state_id) -> ims::rtaaStar::SearchState * {
    return states_[state_id];
}

auto ims::rtaaStar::getOrCreateSearchState(int state_id) -> ims::rtaaStar::SearchState *{
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

//@TODO: no autos!!!! use strings instead of int

/// @brief given a budget of N, expand N states from start
/// @param start start state of the agent
/// @param path the path that the robot took updated cumulatively
/// @return "found" if a solution is found, "failed" if there is no solution, "continue" if the current plan is not finished
std::string ims::rtaaStar::budgetedPlan(SearchState* start, std::vector<StateType>& path){
    int count = 0;
    int iter {0};
    SearchState* state = start;

    while (!open_.empty() && !isTimeOut() && count < params_.N_){
        if (count != 0){
            state = open_.min();
            open_.pop();
        }
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
    if (!open_.empty() || !isTimeOut() ){
        return "failed";
    }
    return "continue";
}


/// @brief moves onto the most promising adjacent state and construct the current partial path
/// @param start the starting state of the current iteration of budgeted plan
/// @param path the path that the robot took updated cumulatively
/// @return the current state of the agent after execution
auto ims::rtaaStar::executePartialPlan(SearchState* start, std::vector<StateType>& path){
    int sID = open_.min()->state_id;
    constructPartialPath(sID,start->state_id,path);
    close_.clear();
    return getSearchState(sID);
}

/// @brief plan for a path
/// @param path the solution of the path
/// @return if a solution was found
bool ims::rtaaStar::plan(std::vector<StateType>& path){
    startTimer();
    auto start = open_.min();
    open_.pop();
    std::string res = "continue";
    while (res == "continue"){
        res = budgetedPlan(start,path);
        updateHeuristicRTAA();
        start = executePartialPlan(start, path);
    }
    if (res == "found"){
        return true;
    }else{
        getTimeFromStart(stats_.time);
        return false;
    }
}


/// @brief udpate the heuristic of visited states in the dictionary using the RTAA* algorithm
void ims::rtaaStar::updateHeuristicRTAA(){
    OpenList temp;
    SearchState *min_state = open_.min();
    double min_cost = min_state->f;
    for (size_t i = 0; i < close_.size(); i++){
        heuristicDict[close_[i]] = min_cost - close_[i]->g;
    }
}

/// @brief udpate the heuristic of visited states in the dictionary using the LRTA* algorithm
void ims::rtaaStar::updateHeuristicLRTA(){
    std::map<SearchState*, double> closed_heu;
    for (auto& item: close_)
    {
        closed_heu[item] = INF_DOUBLE;
    }
    std::map<SearchState*, double> h_value_record;
    while (h_value_record != closed_heu){
        h_value_record = closed_heu;
        for (size_t i = 0; i < close_.size(); i++){
            std::vector<double> f_neighbours;
            std::vector<int> successors;
            std::vector<double> costs;
            action_space_ptr_->getSuccessors(close_[i]->state_id, successors, costs);
            for(auto s_next : successors)
            {
                if (std::find(close_.begin(), close_.end(), getSearchState(s_next)) == close_.end())
                {
                    f_neighbours.push_back(costs[s_next] + computeHeuristic(s_next));
                }
                else
                {
                    f_neighbours.push_back(costs[s_next] + closed_heu[getSearchState(s_next)]);
                }
                // We need to dereference the iterator to get the value, as min_element returns an iterator.
                closed_heu[getSearchState(s_next)] = *std::min_element(f_neighbours.begin(), f_neighbours.end());
            }
        }
    }
    for (size_t i = 0; i < close_.size(); i++){
        heuristicDict[close_[i]] = closed_heu[close_[i]];
    }
}

/// @brief construct the path from current iteration of budgeted planning
/// @param state_id the starting position of the current plan
/// @param target_id the end position of the current plan
/// @param path the path that the robot took updated cumulatively
void ims::rtaaStar::constructPartialPath(int state_id, int target_id, std::vector<StateType>& path){
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
void ims::rtaaStar::expand(int state_id){
    auto state_ = getSearchState(state_id);
    close_.push_back(state_);
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
    auto look_up = getSearchState(state_id);
    if (heuristicDict.find(look_up) == heuristicDict.end()){
        if (!heuristic_->getHeuristic(s->state, dist))
            throw std::runtime_error("Heuristic function failed");
        else
            heuristicDict[look_up] = dist;
            return dist;
    }else{
        return heuristicDict.at(look_up); //use at instead
    }
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
    return std::any_of(goals_.begin(), goals_.end(), [&state_id](int goal_ind) {return state_id == goal_ind;});
}

void ims::rtaaStar::resetPlanningData(){
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

