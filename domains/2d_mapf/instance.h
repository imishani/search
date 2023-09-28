#pragma once

#include <memory>
#include <fstream>
#include <search/common/types.hpp>
#include "collision_checker_2d.h"

class MAPFInstance {
private:
    std::shared_ptr<CollisionChecker2D> m_collision_checker;
    vector<StateType> m_starts;
    vector<StateType> m_goals;

    vector<StateType> addToEnd(const vector<StateType>& states, double val);

public:
    string m_map_file;
    void loadCustomInstance(int map_index, int num_agents);
    void loadBenchmarkInstance(const string& map_file, 
                const string& agent_file, int num_rows);

    std::shared_ptr<CollisionChecker2D> getCC() {return m_collision_checker;}
    vector<StateType> getRawStarts() {return m_starts;}
    vector<StateType> getRawGoals() {return m_goals;}
    vector<StateType> getStartsWithTime() {return addToEnd(m_starts, 0);}
    vector<StateType> getGoalsWithTime() {return addToEnd(m_goals, -1);}
};



////////////////// Implementations Below //////////////////////

vector<StateType> MAPFInstance::addToEnd(const vector<StateType>& states, double val) {
    vector<StateType> new_states;
    for (const auto& state : states) {
        StateType new_state = state;
        new_state.push_back(val);
        new_states.push_back(new_state);
    }
    return new_states;
}

const vector<string> idxToMapName = {
    "../domains/2d_robot_nav/data/hrt201n/hrt201n.map",
    "../domains/2d_robot_nav/data/den501d/den501d.map",
    "../domains/2d_robot_nav/data/den520d/den520d.map",
    "../domains/2d_robot_nav/data/ht_chantry/ht_chantry.map",
    "../domains/2d_robot_nav/data/brc203d/brc203d.map",
    "../domains/2d_mapf/data/corridor10/corridor10.map",
    "../domains/2d_mapf/data/corridor20/corridor20.map",
    "../domains/2d_mapf/data/clutter32/clutter32.map",
    "../domains/2d_mapf/data/hallway_6/hallway_6.map"
};
const vector<string> idxToStartGoal = {
    "../domains/2d_robot_nav/data/hrt201n/",
    "../domains/2d_robot_nav/data/den501d/",
    "../domains/2d_robot_nav/data/den520d/",
    "../domains/2d_robot_nav/data/ht_chantry/",
    "../domains/2d_robot_nav/data/brc203d/",
    "../domains/2d_mapf/data/corridor10/",
    "../domains/2d_mapf/data/corridor20/",
    "../domains/2d_mapf/data/clutter32/",
    "../domains/2d_mapf/data/hallway_6/"
};

void MAPFInstance::loadCustomInstance(int map_index, int num_agents) {
    string path = idxToStartGoal[map_index];
    m_map_file = idxToMapName[map_index];

    ///////////////////////// Load the agents /////////////////////////
    std::ifstream starts_fin(path + "nav2d_starts.txt");
    std::ifstream goals_fin(path + "nav2d_goals.txt");

    for (int j = 0; j < num_agents; ++j) {
        if (starts_fin.eof() || goals_fin.eof()) {
            throw std::runtime_error("Error: not enough starts/goals for " 
                                    + std::to_string(num_agents) + " agents");
        }
        StateType start, goal;
        double val_start, val_goal;
        for (int i = 0; i < 2; ++i) {
            starts_fin >> val_start;
            goals_fin >> val_goal;
            start.push_back(val_start);
            goal.push_back(val_goal);
        }

        m_starts.push_back(start);
        m_goals.push_back(goal);

        double cost, length;
        starts_fin >> cost;
        starts_fin >> length;
    }
    starts_fin.close();
    goals_fin.close();

    ///////////////////////// Load the map /////////////////////////
    m_collision_checker = std::make_shared<CollisionChecker2D>();
    m_collision_checker->loadMap(m_map_file);
}