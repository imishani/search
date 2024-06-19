//
// Created by spencer on 2024-06-13.
//

#ifndef SEARCH_ACTION_SPACE_2D_ROB_EXPERIENCE_HPP
#define SEARCH_ACTION_SPACE_2D_ROB_EXPERIENCE_HPP


#include "search/action_space/egraph_action_space.hpp"
#include <search/common/scene_interface.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <search/common/experience_graph.hpp>
#include <search/common/intrusive_heap.h>


class Scene2DRob : public ims::SceneInterface {
public:
    explicit Scene2DRob(std::vector<std::vector<int>> &map_) : ims::SceneInterface(){
        map = &map_;
        map_size = {map->size(), map[0].size()};
    }

    std::vector<std::vector<int>>* map;
    std::vector<size_t> map_size;
};

struct ActionType2dRob : public ims::ActionType {

    ActionType2dRob() : ims::ActionType() {
        name = "ActionType2dRob";
        num_actions = 8;
        action_names = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
        action_costs = {1, 1.414, 1, 1.414, 1, 1.414, 1, 1.414};
        action_prims = {{0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}};
        state_discretization_ = {1, 1};
    }

    std::vector<Action> getPrimActions() override{
        return action_prims;
    }

    void Discretization(StateType& state_des) override{
        state_discretization_ = state_des;
    }

    std::string name;
    int num_actions;
    std::vector<std::string> action_names;
    std::vector<double> action_costs;
    std::vector<Action> action_prims;

};

class ActionSpaceEGraph2DRob : public ims::EGraphActionSpace{

protected:
    std::shared_ptr<Scene2DRob> env_;
    std::shared_ptr<ActionType2dRob> action_type_;


public:
    ActionSpaceEGraph2DRob(const Scene2DRob& env,
                           const ActionType2dRob& actions_ptr) : ims::EGraphActionSpace(){
        this->env_ = std::make_shared<Scene2DRob>(env);
        this->action_type_ = std::make_shared<ActionType2dRob>(actions_ptr);
    }

    void getActions(int state_id,
                    std::vector<ActionSequence> &actions_seq,
                    bool check_validity) override {
        auto actions = action_type_->getPrimActions();
        for (int i{0}; i<action_type_->num_actions; i++){
            auto action = actions[i];
            if (check_validity){
                auto curr_state = this->getRobotState(state_id);
                auto next_state_val = StateType(curr_state->state.size());
                std::transform(curr_state->state.begin(), curr_state->state.end(), action.begin(), next_state_val.begin(), std::plus<>());
                if (!isStateValid(next_state_val)){
                    continue;
                }
            }
            ActionSequence action_seq;
            action_seq.push_back(action);
            actions_seq.push_back(action_seq);
        }
    }







    // TODO: copied from before
    bool getSuccessors(int curr_state_ind,
                       std::vector<int>& successors,
                       std::vector<double>& costs) override{
        auto curr_state = this->getRobotState(curr_state_ind);
        std::vector<ActionSequence> actions;
        getActions(curr_state_ind, actions, false);

        for (int i {0} ; i < actions.size() ; i++){
            auto action = actions[i][0];
            auto next_state_val = StateType(curr_state->state.size());
            std::transform(curr_state->state.begin(), curr_state->state.end(), action.begin(), next_state_val.begin(), std::plus<>());
            if (isStateValid(next_state_val)){
                int next_state_ind = getOrCreateRobotState(next_state_val);
                successors.push_back(next_state_ind);
                costs.push_back(action_type_->action_costs[i]);
            }
        }
        return true;
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

    /// override functions from ActionSpaceEgraphMixin
    bool loadEGraph(const std::string& path) override {
        // The path needs to be a directory containing the experience files
        // check if path is a directory
        boost::filesystem::path p(path);
        if (!boost::filesystem::is_directory(p)) {
            std::cout << RED << "[ERROR]: Path in loadEGraph is not a directory" << RESET << std::endl;
            return false;
        }

        // loop through all files in the directory and parse them
        for (auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(p), {})) {
            if (entry.path().extension() == ".csv") {
                std::vector<StateType> egraph_states;
                if (!parseEGraphFile(entry.path().string(), egraph_states) || egraph_states.empty()) {
                    continue;
                }
                auto& prev_state = egraph_states.front();
                auto pid = egraph_.insert_node(prev_state);
                state_to_nodes_map_[prev_state].push_back(pid);

                int entry_s_id = getOrCreateRobotState(prev_state);

                // map the state id to the node id in the experience graph
                egraph_state_ids_.resize(pid + 1, -1);
                egraph_state_ids_[pid] = entry_s_id;
                states_to_nodes_[entry_s_id] = pid;

                std::vector<StateType> edge_data;
                for (size_t i = 1; i < egraph_states.size(); ++i) {
                    auto& curr_state = egraph_states[i];
                    StateType cs = curr_state;
                    if (curr_state != prev_state) { // TODO: check if its fine
                        auto cid = egraph_.insert_node(curr_state);
                        state_to_nodes_map_[curr_state].push_back(cid);

                        int curr_s_id = getOrCreateRobotState(curr_state);

                        // map the state id to the node id in the experience graph
                        egraph_state_ids_.resize(cid + 1, -1);
                        egraph_state_ids_[cid] = curr_s_id;
                        states_to_nodes_[curr_s_id] = cid;

                        // add edge
                        egraph_.insert_edge(pid, cid, edge_data);
                        pid = cid;
                        prev_state = cs;
                    } else {
                        edge_data.push_back(curr_state);
                    }
                }
            }
        }
        return true;
    }

    void getEGraphNodes(int state_id,
                        std::vector<ims::smpl::ExperienceGraph::node_id> &nodes) override {
        auto it = states_to_nodes_.find(state_id);
        if (it != states_to_nodes_.end()) {
            nodes.push_back(it->second);
        }
    }

    bool shortcut(int first_id, int second_id, int& cost) override {
        auto* state_1 = getRobotHashEntry(first_id);
        auto* state_2 = getRobotHashEntry(second_id);

        if (state_1 == nullptr || state_2 == nullptr) {
            return false;
        }
        cost = 1;
        return true;
    }

    bool checkShortcutTransition(int first_id,
                                 int second_id,
                                 PathType& trans_path) override {
        auto prev_nit = std::find(egraph_state_ids_.begin(), egraph_state_ids_.end(), first_id);
        auto curr_nit = std::find(egraph_state_ids_.begin(), egraph_state_ids_.end(), second_id);
        if (prev_nit != egraph_state_ids_.end() &&
            curr_nit != egraph_state_ids_.end()) {
            ims::smpl::ExperienceGraph::node_id prev_nid = std::distance(egraph_state_ids_.begin(), prev_nit);
            ims::smpl::ExperienceGraph::node_id curr_nid = std::distance(egraph_state_ids_.begin(), curr_nit);
            std::vector<ims::smpl::ExperienceGraph::node_id> node_path;
            bool found = findShortestExperienceGraphPath(prev_nid, curr_nid, node_path);
            if (found){
                for (ims::smpl::ExperienceGraph::node_id n : node_path){
                    int s_id = egraph_state_ids_[n];
                    auto* entry = getRobotHashEntry(s_id);
                    assert(entry);
                    trans_path.push_back(entry->state);
                }
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }

    bool snap(int first_id, int second_id, int& cost) override {
        auto* state_1 = getRobotHashEntry(first_id);
        auto* state_2 = getRobotHashEntry(second_id);

        if (state_1 == nullptr || state_2 == nullptr) {
            return false;
        }

        if (isStateToStateValid(state_1->state, state_2->state)) {
            cost = 1;
            return true;
        } else {
            return false;
        }
    }

    bool checkSnapTransition(int first_id,
                             int second_id,
                             PathType& trans_path) override {
        int cost;
        if (snap(first_id, second_id, cost)){
            auto* entry = getRobotHashEntry(second_id);
            assert(entry);
            trans_path.push_back(entry->state);
            return true;
        } else {
            return false;
        }
    }

    const std::shared_ptr<ims::smpl::ExperienceGraph> getExperienceGraph() const override {
        std::shared_ptr<ims::smpl::ExperienceGraph> egraph_ptr = std::make_shared<ims::smpl::ExperienceGraph>(egraph_);
        return egraph_ptr;
    }

    std::shared_ptr<ims::smpl::ExperienceGraph> getExperienceGraph() override {
        std::shared_ptr<ims::smpl::ExperienceGraph> egraph_ptr = std::make_shared<ims::smpl::ExperienceGraph>(egraph_);
        return egraph_ptr;
    }

    int getStateID(ims::smpl::ExperienceGraph::node_id n) const override {
        if (n < egraph_state_ids_.size()) {
            return egraph_state_ids_[n];
        } else {
            return -1;
        }
    }

private:
    hash_map<int, ims::smpl::ExperienceGraph::node_id> states_to_nodes_;
    ims::smpl::ExperienceGraph egraph_;
    std::vector<int> egraph_state_ids_;

    typedef hash_map<
            StateType ,
            std::vector<ims::smpl::ExperienceGraph::node_id>,
            StateTypeHash> StateToEGraphNodesMap;

    StateToEGraphNodesMap state_to_nodes_map_;

    /// @brief Parsing function of experience graph file
    /// @param filepath - path to experience graph file
    /// @param egraph_states - vector of states in the experience graph
    /// @return true if parsing was successful, false otherwise
    static bool parseEGraphFile(const std::string& filepath,
                                PathType& egraph_states) {
        std::ifstream egraph_file(filepath);
        if (!egraph_file.is_open()) {
            std::cout << RED << "[ERROR]: Failed to open experience graph file: " << filepath << RESET << std::endl;
            return false;
        }

        /* parse CSV
            the format of the CSV file is as follows:
                1. header line: "Experience, N (number of states), dim (dimension of state)"
                2. state lines: "state_1, state_2, ..., state_n"
        */

        std::string line;
        std::vector<std::string> tokens;
        std::vector<std::string> state_tokens;
        StateType state_values;
        int num_states = 0;
        int dim = 0;
        while (std::getline(egraph_file, line)) {
            // read line and tokenize on commas ',' and make sure no empty tokens are created
            boost::split(tokens, line, boost::is_any_of(","));
            tokens.erase(std::remove_if(tokens.begin(), tokens.end(), [](const std::string& s) { return s.empty(); }), tokens.end());

            if (tokens[0] == "Experience") {
                num_states = std::stoi(tokens[1]);
                dim = std::stoi(tokens[2]);
                egraph_states.reserve(num_states);
            } else {
                state_tokens.clear();
                state_tokens = tokens;
                state_values.clear();
                state_values.reserve(dim);
                for (const auto& token : state_tokens) {
                    state_values.push_back(std::stod(token));
                }
                egraph_states.emplace_back(state_values);
            }
        }
        return true;
    }

    /// @brief Find the shortest experience path between two states
    /// @param start_node - start node id
    /// @param goal_node - goal node id
    /// @param path - vector of states in the shortest experience path
    /// @return true if path was found, false otherwise
    bool findShortestExperienceGraphPath(
            ims::smpl::ExperienceGraph::node_id start_node,
            ims::smpl::ExperienceGraph::node_id goal_node,
            std::vector<ims::smpl::ExperienceGraph::node_id>& path)
    {
        struct ExperienceGraphSearchNode : smpl::HeapElement
        {
            int g;
            bool closed;
            ExperienceGraphSearchNode* bp;
            ExperienceGraphSearchNode() :
                    g(std::numeric_limits<int>::max()),
                    closed(false),
                    bp(nullptr)
            { }
        };

        struct NodeCompare
        {
            bool operator()(
                    const ExperienceGraphSearchNode& a,
                    const ExperienceGraphSearchNode& b)
            {
                return a.g < b.g;
            }
        };

        typedef smpl::IntrusiveHeap<ExperienceGraphSearchNode, NodeCompare> heap_type;

        std::vector<ExperienceGraphSearchNode> search_nodes(egraph_.num_nodes());

        heap_type open;

        search_nodes[start_node].g = 0;
        open.push(&search_nodes[start_node]);
        int exp_count = 0;
        while (!open.empty()) {
            ++exp_count;
            ExperienceGraphSearchNode* min = open.min();
            open.pop();
            min->closed = true;

            if (min == &search_nodes[goal_node]) {
                std::cout << RED << "[ERROR]: Found shortest experience graph path" << RESET << std::endl;
                ExperienceGraphSearchNode* ps = nullptr;
                for (ExperienceGraphSearchNode* s = &search_nodes[goal_node];
                     s; s = s->bp)
                {
                    if (s != ps) {
                        path.push_back(std::distance(search_nodes.data(), s));
                        ps = s;
                    } else {
                        std::cout << RED << "[ERROR]: Cycle detected!" << RESET << std::endl;
                    }
                }
                std::reverse(path.begin(), path.end());
                return true;
            }

            ims::smpl::ExperienceGraph::node_id n = std::distance(search_nodes.data(), min);
            auto adj = egraph_.adjacent_nodes(n);
            for (auto ait = adj.first; ait != adj.second; ++ait) {
                ExperienceGraphSearchNode& succ = search_nodes[*ait];
                if (succ.closed) {
                    continue;
                }
                int new_cost = min->g + 1;
                if (new_cost < succ.g) {
                    succ.g = new_cost;
                    succ.bp = min;
                    if (open.contains(&succ)) {
                        open.decrease(&succ);
                    } else {
                        open.push(&succ);
                    }
                }
            }
        }

        std::cout << "Expanded " << exp_count << " nodes looking for shortcut" << std::endl;
        return false;
    }

};




#endif //SEARCH_ACTION_SPACE_2D_ROB_EXPERIENCE_HPP
