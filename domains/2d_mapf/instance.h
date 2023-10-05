/*
 * Copyright (C) 2023, Rishi V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Carnegie Mellon University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/*!
 * \file   instance.hpp
 * \author Rishi V. (...@cmu.edu)
 * \date   Oct 04 2023
 */

#pragma once

#include<boost/tokenizer.hpp> // For parsing
#include <memory>
#include <fstream>
#include <search/common/types.hpp>
#include "scene_interface_2d_rob.hpp"

/// @brief Helper class that loads MAPF instances from files.
/// @note This class is used for loading the map and agents from a file.
/// @note Handles parsing custom and benchmark instances.
class MAPFInstance {
private:
    std::shared_ptr<Scene2DRob> scene_;
    vector<StateType> starts_;
    vector<StateType> goals_;

    vector<StateType> addToEnd(const vector<StateType>& states, double val);

public:
    string map_file_;
    void loadInstanceFromArguments(const string& workspace_path, const string& map_file, 
                    const string& agent_file, int num_agents);
    void loadCustomInstance(const string& workspace_path, int map_index, int num_agents);
    void loadBenchmarkInstance(const string& map_file, 
                const string& agent_file, int num_agents);

    std::shared_ptr<Scene2DRob> getSceneInterface2D() {return scene_;}
    vector<StateType> getRawStarts() {return starts_;}
    vector<StateType> getRawGoals() {return goals_;}
    vector<StateType> getStartsWithTime() {return addToEnd(starts_, 0);}
    vector<StateType> getGoalsWithTime() {return addToEnd(goals_, -1);}
};

/// @brief Returns if a string is a number or not.
/// @param s 
/// @return
/// @note https://stackoverflow.com/questions/4654636/how-to-determine-if-a-string-is-a-number-with-c 
bool is_number(const std::string& s) {
    string::const_iterator it = s.begin();
    while (it != s.end() && std::isdigit(*it)) ++it;
    return !s.empty() && it == s.end();
}

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

void MAPFInstance::loadInstanceFromArguments(const string& workspace_path,
                const string& map_file, const string& agent_file, int num_agents) {
    if (is_number(map_file)) {
        loadCustomInstance(workspace_path, std::stoi(map_file), num_agents);
    }
    else {
        string map_path = workspace_path + map_file;
        string agent_path = workspace_path + agent_file;
        loadBenchmarkInstance(map_path, agent_path, num_agents);
    }
}

void MAPFInstance::loadBenchmarkInstance(const string& map_file, 
                                    const string& agent_file, int num_agents) {
    map_file_ = map_file;

    ///////////////////////// Load the map /////////////////////////
    scene_ = std::make_shared<Scene2DRob>();
    scene_->loadMap(map_file_);

    ///////////////////////// Load the agents /////////////////////////
    /// Copied from EECBS Instance::loadAgents
    string line;
	std::ifstream myfile(agent_file.c_str());
	if (!myfile.is_open()) {
        throw std::runtime_error("Could not open file " + agent_file);
    }

	getline(myfile, line);
	assert(line[0] == 'v'); // Nathan's benchmark
    if (num_agents <= 0) {
        throw std::runtime_error("The number of agents should be larger than 0");
    }
    boost::char_separator<char> sep("\t");
    for (int i = 0; i < num_agents; i++) {
        getline(myfile, line);
        boost::tokenizer< boost::char_separator<char> > tok(line, sep);
        boost::tokenizer< boost::char_separator<char> >::iterator beg = tok.begin();
        beg++; // skip the first number
        beg++; // skip the map name
        beg++; // skip the columns
        beg++; // skip the rows

        // read start [row,col] for agent i
        int col = atoi((*beg).c_str());
        beg++;
        int row = atoi((*beg).c_str());
        if (!scene_->isCellValid(row, col)) {
            throw std::runtime_error("Error: goal " + std::to_string(row) 
                        + ", " + std::to_string(col) + " is not valid");
        }
        starts_.push_back({col*1.0, row*1.0});

        // read goal [row,col] for agent i
        beg++;
        col = atoi((*beg).c_str());
        beg++;
        row = atoi((*beg).c_str());
        // goal_locations[i] = linearizeCoordinate(row, col);
        if (!scene_->isCellValid(row, col)) {
            throw std::runtime_error("Error: goal " + std::to_string(row) 
                        + ", " + std::to_string(col) + " is not valid");
        }
        goals_.push_back({col*1.0, row*1.0});
    }
}

const vector<string> idxToMapName = {
    "domains/2d_robot_nav/data/hrt201n/hrt201n.map",
    "domains/2d_robot_nav/data/den501d/den501d.map",
    "domains/2d_robot_nav/data/den520d/den520d.map",
    "domains/2d_robot_nav/data/ht_chantry/ht_chantry.map",
    "domains/2d_robot_nav/data/brc203d/brc203d.map",
    "domains/2d_mapf/data/corridor10/corridor10.map",
    "domains/2d_mapf/data/corridor20/corridor20.map",
    "domains/2d_mapf/data/clutter32/clutter32.map",
    "domains/2d_mapf/data/hallway_6/hallway_6.map"
};
const vector<string> idxToStartGoal = {
    "domains/2d_robot_nav/data/hrt201n/",
    "domains/2d_robot_nav/data/den501d/",
    "domains/2d_robot_nav/data/den520d/",
    "domains/2d_robot_nav/data/ht_chantry/",
    "domains/2d_robot_nav/data/brc203d/",
    "domains/2d_mapf/data/corridor10/",
    "domains/2d_mapf/data/corridor20/",
    "domains/2d_mapf/data/clutter32/",
    "domains/2d_mapf/data/hallway_6/"
};

void MAPFInstance::loadCustomInstance(const string& workspace_path, int map_index, int num_agents) {
    string path = workspace_path + idxToStartGoal[map_index];
    map_file_ = workspace_path + idxToMapName[map_index];

    ///////////////////////// Load the map /////////////////////////
    scene_ = std::make_shared<Scene2DRob>();
    scene_->loadMap(map_file_);

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
        if (!scene_->isCellValid(start[1], start[0])) {
            throw std::runtime_error("Error: start " + std::to_string(start[0]) 
                        + ", " + std::to_string(start[1]) + " is not valid");
        }
        if (!scene_->isCellValid(goal[1], goal[0])) {
            throw std::runtime_error("Error: goal " + std::to_string(goal[0]) 
                        + ", " + std::to_string(goal[1]) + " is not valid");
        }

        starts_.push_back(start);
        goals_.push_back(goal);

        double cost, length;
        starts_fin >> cost;
        starts_fin >> length;
    }
    starts_fin.close();
    goals_fin.close();
}