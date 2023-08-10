//
// Created by itamar on 4/1/23.
//

#ifndef SEARCH_ACTIONSCENE2DROB_HPP
#define SEARCH_ACTIONSCENE2DROB_HPP

#include "search/action_space/action_space.hpp"
#include <search/common/scene_interface.hpp>


class scene2DRob : public ims::SceneInterface {
public:
    explicit scene2DRob(std::vector<std::vector<int>> &map_) : ims::SceneInterface(){
        this->map = &map_;
        this->map_size = {map->size(), map[0].size()};
    }

    std::vector<std::vector<int>>* map;
    std::vector<size_t> map_size;
};

struct actionType2dRob : public ims::ActionType {

    actionType2dRob() : ims::ActionType() {
        this->name = "actionType2dRob";
        this->num_actions = 8;
        this->action_names = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
        this->action_costs = {1, 1.414, 1, 1.414, 1, 1.414, 1, 1.414};
        this->action_deltas = {{0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}};
        this->state_discretization_ = {1, 1};
    }

    std::vector<Action> getActions() override{
        std::vector<Action> actions;
        return this->action_deltas;
    }

    void Discretization(StateType& state_des) override{
        state_discretization_ = state_des;
    }

    std::string name;
    int num_actions;
    std::vector<std::string> action_names;
    std::vector<double> action_costs;
    std::vector<std::vector<double>> action_deltas;

};

class actionSpace2dRob : public ims::ActionSpace {

private:
    std::shared_ptr<scene2DRob> env_;
    std::shared_ptr<actionType2dRob> actions_;

public:
    actionSpace2dRob(const scene2DRob& env,
                     const actionType2dRob& actions_ptr) : ims::ActionSpace(){
        this->env_ = std::make_shared<scene2DRob>(env);
        this->actions_ = std::make_shared<actionType2dRob>(actions_ptr);
    }

    bool isStateValid(const StateType& state_val) override{
        if (state_val[0] < 0 || state_val[0] >= (double)env_->map_size[0] || state_val[1] < 0 || state_val[1] >= (double)env_->map_size[1]){
            return false;
        }
        auto map_val = env_->map->at((size_t)state_val[0]).at((size_t)state_val[1]);
        if (map_val == 100){
            return false;
        }
        return true;
    }

    bool isPathValid(const PathType& path) override{
         return std::all_of(path.begin(), path.end(), [this](const StateType& state_val){return isStateValid(state_val);});
    }

    bool getSuccessors(int curr_state_ind,
                       std::vector<int>& successors,
                       std::vector<double>& costs) override{
        auto curr_state = this->getRobotState(curr_state_ind);
        auto actions = actions_->getActions();
        for (int i {0} ; i < actions_->num_actions ; i++){
            auto action = actions[i];
            auto next_state_val = StateType(curr_state->state.size());
            std::transform(curr_state->state.begin(), curr_state->state.end(), action.begin(), next_state_val.begin(), std::plus<>());
            if (isStateValid(next_state_val)){
                int next_state_ind = getOrCreateRobotState(next_state_val);
                successors.push_back(next_state_ind);
                costs.push_back(actions_->action_costs[i]);
            }
        }
        return true;
    }
};


#endif //SEARCH_ACTIONSCENE2DROB_HPP
