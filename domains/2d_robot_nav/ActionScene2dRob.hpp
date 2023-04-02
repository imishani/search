//
// Created by itamar on 4/1/23.
//

#ifndef SEARCH_ACTIONSCENE2DROB_HPP
#define SEARCH_ACTIONSCENE2DROB_HPP

#include <common/actionSpace.hpp>
#include <common/SceneInterface.hpp>


class scene2DRob : public ims::SceneInterface {
public:
    explicit scene2DRob(std::vector<std::vector<int>> &map_) : ims::SceneInterface(){
        this->map = &map_;
        this->map_size = {map->size(), map[0].size()};
    }

    std::vector<std::vector<int>>* map;
    std::vector<size_t> map_size;
};

struct actionType2dRob : public ims::actionType {

    actionType2dRob() : ims::actionType() {
        this->name = "actionType2dRob";
        this->num_actions = 8;
        this->action_names = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
        this->action_costs = {1, 1.414, 1, 1.414, 1, 1.414, 1, 1.414};
        this->action_deltas = {{0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}};
        this->mStateDiscretization = {1, 1};
    }

    std::vector<action> getActions() override{
        std::vector<action> actions;
        return this->action_deltas;
    }

    void Discretization(stateType& state_des) override{
        mStateDiscretization = state_des;
    }

    std::string name;
    int num_actions;
    std::vector<std::string> action_names;
    std::vector<double> action_costs;
    std::vector<std::vector<double>> action_deltas;

};

class actionSpace2dRob : public ims::actionSpace {

private:
    std::shared_ptr<scene2DRob> m_env;
    std::shared_ptr<actionType2dRob> m_actions;

public:
    actionSpace2dRob(const scene2DRob& env,
                     const actionType2dRob& actions_ptr) : ims::actionSpace(){
        this->m_env = std::make_shared<scene2DRob>(env);
        this->m_actions = std::make_shared<actionType2dRob>(actions_ptr);
    }

    bool isStateValid(const stateType& state_val) override{
        if (state_val[0] < 0 || state_val[0] >= m_env->map_size[0] || state_val[1] < 0 || state_val[1] >= m_env->map_size[1]){
            return false;
        }
        auto map_val = m_env->map->at(state_val[0]).at(state_val[1]);
        if (map_val == 100){
            return false;
        }
        return true;
    }

    bool isPathValid(const pathType& path) override{
         return std::all_of(path.begin(), path.end(), [this](const stateType& state_val){return isStateValid(state_val);});
    }

    bool getSuccessors(int curr_state_ind,
                       std::vector<ims::state*>& successors,
                       std::vector<double>& costs) override{
        auto curr_state = this->getState(curr_state_ind);
        auto curr_state_val = curr_state->getState();
        auto actions = m_actions->getActions();
        for (int i {0} ; i < m_actions->num_actions ; i++){
            auto action = actions[i];
            auto next_state_val = stateType(curr_state_val.size());
            std::transform(curr_state_val.begin(), curr_state_val.end(), action.begin(), next_state_val.begin(), std::plus<double>());
            if (isStateValid(next_state_val)){
                int next_state_ind = getOrCreateState(next_state_val);
                auto next_state = this->getState(next_state_ind);
                successors.push_back(next_state);
                costs.push_back(m_actions->action_costs[i]);
            }
        }
        return true;
    }
};


#endif //SEARCH_ACTIONSCENE2DROB_HPP
