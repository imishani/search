/*
 * Copyright (C) 2024, Itamar Mishani
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
 * \file   generate_data_fqi.cpp
 * \brief  Generate data for the FQI (Fitted Q-Iteration) algorithm
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   8/11/24
*/


#include <boost/filesystem.hpp>
#include <vector>
#include <memory>
#include <iostream>
#include <string>
#include <cmath>
#include <random>
#include <hdf5.h>

// project includes
#include <search/planners/dijkstra.hpp>

#include "../action_space_2d_rob.hpp"
#include "../utils.hpp"



/// @brief generate a start and goal pair
/// @param scene the scene
/// @param action_space the action space
/// @param number_of_goals the number of goals to generate
std::vector<StateType> generateGoals(const Scene2DRob& scene,
                                              const std::shared_ptr<actionSpace2dRob>& action_space,
                                              int number_of_goals) {
    std::vector<StateType> goals;
    std::random_device rd;
    std::mt19937 gen(rd());
    while (goals.size() < number_of_goals) {
        // generate a random start and goal
        StateType goal = {std::round(gen() % scene.map_size.at(0)), std::round(gen() % scene.map_size.at(1))};
        // check if the start and goal are valid
        if (action_space->isStateValid(goal)) {
            goals.push_back(goal);
        }
    }
    return goals;
}



int main(int argc, char** argv) {

    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <map_file> <scale> <number_of_goals>" << std::endl;
        return 0;
    }
    std::vector<std::string> maps;

    boost::filesystem::path full_path( boost::filesystem::current_path() );
    std::cout << "Current path is : " << full_path.string() << std::endl;
    // At each emplace_back, use the full pathh and concatenate the map name
    maps.emplace_back(full_path.string() + "/../domains/2d_robot_nav/data/hrt201n/hrt201n.map");
    maps.emplace_back(full_path.string() + "/../domains/2d_robot_nav/data/den501d/den501d.map");
    maps.emplace_back(full_path.string() + "/../domains/2d_robot_nav/data/den520d/den520d.map");
    maps.emplace_back(full_path.string() + "/../domains/2d_robot_nav/data/ht_chantry/ht_chantry.map");
    maps.emplace_back(full_path.string() + "/../domains/2d_robot_nav/data/brc203d/brc203d.map");

    std::vector<std::string> dirs = {full_path.string() + "/../domains/2d_robot_nav/data/hrt201n/",
                                     full_path.string() + "/../domains/2d_robot_nav/data/den501d/",
                                     full_path.string() + "/../domains/2d_robot_nav/data/den520d/",
                                     full_path.string() + "/../domains/2d_robot_nav/data/ht_chantry/",
                                     full_path.string() + "/../domains/2d_robot_nav/data/brc203d/",
    };

    int map_index = std::stoi(argv[1]);
    int scale = std::stoi(argv[2]);
    int number_of_goals = std::stoi(argv[3]);

    std::string path = dirs[map_index];
    std::string map_file = maps[map_index];

    std::string type;
    int width, height;
    std::vector<std::vector<int>> map = loadMap(map_file.c_str(), type, width, height, scale);


    // construct the planner
    std::cout << "Constructing planner..." << std::endl;
    // construct planner params
    // initialize the heuristic
    ims::DijkstraParams params;
    params.verbose = false;
    // construct the scene and the action space
    Scene2DRob scene (map);
    ActionType2dRob action_type;
    std::shared_ptr<actionSpace2dRob> ActionSpace = std::make_shared<actionSpace2dRob>(scene, action_type);
    std::vector<StateType> goals = generateGoals(scene, ActionSpace, number_of_goals);

    // log the results
    std::unordered_map<int, PlannerStats> logs;
    std::shared_ptr<std::vector<ims::Policy>> policies = std::make_shared<std::vector<ims::Policy>>();
    for (int i {0}; i < goals.size(); i++) {

        // construct planner
        ims::Dijkstra planner(params);
        // catch the exception if the start or goal is not valid
        try {
            planner.initializePolicyPlanner(ActionSpace, goals[i]);
        }
        catch (std::exception& e) {
            std::cout << RED << e.what() << RESET << std::endl;
            continue;
        }
        // plan
        std::cout << "Planning..." << std::endl;
        if (!planner.exhaustPlan()) {
            std::cout << RED << "Couldn't finish policy generation for all map!" << RESET << std::endl;
            continue;
        }
        else {
            std::cout << GREEN << "Policy found!" << RESET << std::endl;
        }

        PlannerStats stats = planner.reportStats();
        std::cout << GREEN << "Planning time: " << stats.time << " sec" << std::endl;
        std::cout << "Number of nodes expanded: " << stats.num_expanded << std::endl;
        logs[i] = stats; // log the stats
        auto all_states = planner.getAllSearchStates();

        // sample N state actions and generate a policy
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, static_cast<int>(all_states.size()) - 1);
        for (int sample_idx {0}; sample_idx < 20000; sample_idx++) {
            int state_id = dis(gen);
            std::vector<ims::Policy> policy = planner.getSuccessorPolicies(state_id);
            // pick a random policy
            std::uniform_int_distribution<> dis_policy(0, static_cast<int>(policy.size()) - 1);
            const ims::Policy& sampled_policy = policy[dis_policy(gen)];
            policies->push_back(sampled_policy);
        }
        // for (auto& state : planner.getAllSearchStates()) {
        //     if (state->parent_id == PARENT_TYPE::START) {
        //         continue;
        //     }
        //     ims::Policy policy = planner.getPolicy(state->state_id);
        //     policies->push_back(policy);
        // }
    }
    // save the paths to a file
    std::string path_file_ = path + "data/" + "fqi_replay_buffer.h5";
    hid_t file = H5Fcreate(path_file_.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
    hid_t dataset_id;
//    hsize_t dims[2] = {policies->size(), 10};
    hsize_t dims[2] = {policies->size(), 9};
    hid_t dataspace = H5Screate_simple(2, dims, nullptr);
    dataset_id = H5Dcreate(file, "data", H5T_NATIVE_DOUBLE, dataspace, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);

//    auto *data = new double[policies->size() * 10];
    auto *data = new double[policies->size() * 9];

    for (int i {0}; i < policies->size(); i++) {\
        data[i * 9] = (*policies)[i].state[0];
        data[i * 9 + 1] = (*policies)[i].state[1];
        data[i * 9 + 2] = (*policies)[i].goal_state[0];
        data[i * 9 + 3] = (*policies)[i].goal_state[1];
        // check action index
        bool found = false;
        for (int j {0}; j < action_type.action_prims.size(); j++) {
            auto action = action_type.action_prims[j];
            if (action[0] == (*policies)[i].action[0] && action[1] == (*policies)[i].action[1]) {
                data[i * 9 + 4] = j;
                found = true;
                break;
            }
        }
        assert(found);

//        data[i * 10 + 4] = (*policies)[i].action[0];
//        data[i * 10 + 5] = (*policies)[i].action[1];
        data[i * 9 + 5] = (*policies)[i].next_state[0];
        data[i * 9 + 6] = (*policies)[i].next_state[1];
        data[i * 9 + 7] = (*policies)[i].cost;
        data[i * 9 + 8] = (*policies)[i].q_value;
    }
    H5Dwrite(dataset_id, H5T_NATIVE_DOUBLE, H5S_ALL, H5S_ALL, H5P_DEFAULT, data);

    // Create metadata group
    hid_t group_id = H5Gcreate(file, "metadata", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);

    // add data size, and state dim to the metadata
    hsize_t dims_ = 1;
    dataspace = H5Screate_simple(1, &dims_, nullptr);
    hid_t dataset_size = H5Dcreate(group_id, "size", H5T_NATIVE_INT, dataspace, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    int size = (int)policies->size();
    H5Dwrite(dataset_size, H5T_NATIVE_INT, H5S_ALL, H5S_ALL, H5P_DEFAULT, &size);
    H5Dclose(dataset_size);
    H5Sclose(dataspace);

    hsize_t dims_3 = 1;
    hid_t dataspace_3 = H5Screate_simple(1, &dims_3, nullptr);
    hid_t dataset_state_dim = H5Dcreate(group_id, "state_dim", H5T_NATIVE_INT, dataspace_3, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    int state_dim = 2;
    H5Dwrite(dataset_state_dim, H5T_NATIVE_INT, H5S_ALL, H5S_ALL, H5P_DEFAULT, &state_dim);
    H5Dclose(dataset_state_dim);
    H5Sclose(dataspace_3);

    // action dim
    hsize_t dims_4 = 1;
    hid_t dataspace_4 = H5Screate_simple(1, &dims_4, nullptr);
    hid_t dataset_action_dim = H5Dcreate(group_id, "action_dim", H5T_NATIVE_INT, dataspace_4, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    int action_dim = 8; // 8-connected grid
    H5Dwrite(dataset_action_dim, H5T_NATIVE_INT, H5S_ALL, H5S_ALL, H5P_DEFAULT, &action_dim);
    H5Dclose(dataset_action_dim);
    H5Sclose(dataspace_4);

    // add a subgroup within metadata named domain
    hid_t domain_group = H5Gcreate(group_id, "domain", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    // add the domain name, map index and scale to the domain group
    hid_t domain_dataspace = H5Screate(H5S_SCALAR);
    hid_t domain_dataset = H5Dcreate(domain_group, "name", H5T_C_S1, domain_dataspace, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    H5Dwrite(domain_dataset, H5T_C_S1, H5S_ALL, H5S_ALL, H5P_DEFAULT, "2d_robot_nav");
    H5Dclose(domain_dataset);
    H5Sclose(domain_dataspace);

    hid_t map_index_dataspace = H5Screate(H5S_SCALAR);
    hid_t map_index_dataset = H5Dcreate(domain_group, "map_index", H5T_NATIVE_INT, map_index_dataspace, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    H5Dwrite(map_index_dataset, H5T_NATIVE_INT, H5S_ALL, H5S_ALL, H5P_DEFAULT, &map_index);
    H5Dclose(map_index_dataset);
    H5Sclose(map_index_dataspace);

    H5Dclose(dataset_id);
    H5Fclose(file);


    return 0;
}

