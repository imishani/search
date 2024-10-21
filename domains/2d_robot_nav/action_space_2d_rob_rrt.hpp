//
// Created by spencer on 10/21/24.
//

#ifndef ACTION_SPACE_2D_ROB_RRT_HPP
#define ACTION_SPACE_2D_ROB_RRT_HPP



#include "search/action_space/rrt_action_space.hpp"
#include <search/common/scene_interface.hpp>
#include <filesystem>
#include <boost/algorithm/string.hpp>


class Scene2DRob : public ims::SceneInterface {
public:
    explicit Scene2DRob(std::vector<std::vector<int>> &map_) : ims::SceneInterface(){
        map = &map_;
        map_size = {map->size(), map->at(0).size()};
    }

    std::vector<std::vector<int>>* map;
    std::vector<size_t> map_size;
};


struct ActionType2dRob : public ims::ActionType {
    ActionType2dRob() : ims::ActionType() {
        name = "ActionType2dRob";
        num_actions = 8;
        action_names = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
        action_seqs_transition_costs = {{1.0, 0.0},
                                        {1.414, 0.0},
                                        {1, 0.0},
                                        {1.414, 0.0},
                                        {1, 0.0},
                                        {1.414, 0.0},
                                        {1, 0.0},
                                        {1.414, 0.0}};
        action_seq_prims = {{{0, 0}, {0, 1}},
                            {{0, 0}, {1, 1}},
                            {{0, 0}, {1, 0}},
                            {{0, 0}, {1, -1}},
                            {{0, 0}, {0, -1}},
                            {{0, 0}, {-1, -1}},
                            {{0, 0}, {-1, 0}},
                            {{0, 0}, {-1, 1}}};
        state_discretization_ = {1, 1};
    }

    std::vector<Action> getPrimActions() override {
        std::vector<Action> actions;
        actions.reserve(action_seq_prims.size());
        for (const ActionSequence& a : action_seq_prims) {
            actions.push_back(a.back());
        }
        return actions;
    }

    void getPrimActions(std::vector<ActionSequence>& action_seqs,
                        std::vector<std::vector<double>>& action_transition_costs) override {
        action_seqs = action_seq_prims;
        action_transition_costs = action_seqs_transition_costs;
    }

    void Discretization(StateType& state_des) override {
        state_discretization_ = state_des;
    }

    std::string name;
    int num_actions;
    std::vector<std::string> action_names;
    std::vector<std::vector<double>> action_seqs_transition_costs;
    std::vector<ActionSequence> action_seq_prims;
};

class ActionSpaceRRT2DRob : public ims::RRTActionSpace{

protected:
    std::shared_ptr<Scene2DRob> env_;
    std::shared_ptr<ActionType2dRob> action_type_;

public:
    ActionSpaceRRT2DRob(const Scene2DRob& env,
                          const ActionType2dRob& actions_ptr) : ims::RRTActionSpace(){
        this->env_ = std::make_shared<Scene2DRob>(env);
        this->action_type_ = std::make_shared<ActionType2dRob>(actions_ptr);
    }

    ~ActionSpaceRRT2DRob() override = default;

    void sampleState(StateType& state) override {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> width(0, env_->map_size[0]);
        std::uniform_real_distribution<> height(0, env_->map_size[1]);
        state.clear();
        state.push_back(width(gen));
        state.push_back(height(gen));
    }

    void getSuccessorInDirection(int cur_state_id, const StateType& target_state, StateType& succ_state, double step_size) override {
        succ_state.clear();
        double resolution = 0.1;
        ims::RobotState* cur_robot_state = getRobotState(cur_state_id);
        StateType cur_state = cur_robot_state->state;
        double dist = std::sqrt(std::pow(cur_state[0] - target_state[0], 2) + std::pow(cur_state[1] - target_state[1], 2));
        int num_steps = std::ceil(dist / resolution);
        double dist_per_step = dist / num_steps;
        // Make sure the new state generated is not the same as our current state
        if (dist_per_step == 0) {
            return;
        }
        StateType probe_state = cur_state;

        for (int i = 1; i < num_steps; ++i) {
            double ratio = dist_per_step / dist;
            probe_state[0] = cur_state[0] + i * ratio * (target_state[0] - cur_state[0]);
            probe_state[1] = cur_state[1] + i * ratio * (target_state[1] - cur_state[1]);
            if(isStateValid(probe_state) && i*dist_per_step <= step_size) {
                succ_state = probe_state;
            } else {
                break;
            }
        }
    }

    void getActions(int state_id, std::vector<ActionSequence>& actions_seq, bool check_validity) override {
        actions_seq.clear();
    }

    bool getSuccessors(int curr_state_ind, std::vector<std::vector<int>>& seqs_state_ids, std::vector<std::vector<double>>& seqs_transition_costs) override {
        seqs_state_ids.clear();
        seqs_transition_costs.clear();
        return false;
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

    /// @brief interpolate between two states
    /// @param state_1 - first state
    /// @param state_2 - second state
    /// @param path - the interpolated path
    /// @resolution - resolution of the interpolation
    void interpolatePath(const StateType& state_1, const StateType& state_2, std::vector<StateType>& path,
                         double resolution = 1) {
        path.clear();
        path.push_back(state_1);
        double dist = std::sqrt(std::pow(state_1[0] - state_2[0], 2) + std::pow(state_1[1] - state_2[1], 2));
        int num_steps = std::ceil(dist / resolution);
        double step_size = dist / num_steps;
        for (int i = 1; i < num_steps; ++i) {
            StateType state = state_1;
            double ratio = i * step_size / dist;
            state[0] = state_1[0] + ratio * (state_2[0] - state_1[0]);
            state[1] = state_1[1] + ratio * (state_2[1] - state_1[1]);
            // discretize the state
            state[0] = std::round(state[0] / action_type_->state_discretization_[0]) * action_type_->state_discretization_[0];
            state[1] = std::round(state[1] / action_type_->state_discretization_[1]) * action_type_->state_discretization_[1];
            path.push_back(state);
        }
        path.push_back(state_2);
    }

    /// @brief check if state to state action is valid
    /// @param state_1 - first state
    /// @param state_2 - second state
    /// @return true if the action is valid, false otherwise
    bool isStateToStateValid(const StateType& state_1, const StateType& state_2) {
        std::vector<StateType> path;
        interpolatePath(state_1, state_2, path);
        return isPathValid(path);
    }

    bool isSameState(const StateType& state_1, const StateType& state_2) {
        return ((size_t)state_1[0] == (size_t)state_2[0]) && ((size_t)state_1[1] == (size_t)state_2[1]);
    }

};


#endif //ACTION_SPACE_2D_ROB_RRT_HPP
