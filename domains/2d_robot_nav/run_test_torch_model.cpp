//
// Created by itamar on 8/20/24.
//


#include <torch/torch.h>
#include <torch/script.h>
#include <boost/filesystem.hpp>
#include <random>

#include "action_space_2d_rob.hpp"
#include "utils.hpp"

void load2DGridWorld(const std::string& map_file, std::vector<std::vector<int>>& map) {
    // check if the file exists
    if (!boost::filesystem::exists(map_file)) {
        std::cout << "The map file does not exist!" << std::endl;
        return;
    }
    FILE *f;
    f = fopen(map_file.c_str(), "r");
    if (f)
    {
        std::vector<int> row;
        int val;
        while (fscanf(f, "%d", &val) != EOF) {
            // check if end of line
            row.push_back(val);
            if (fgetc(f) == '\n') {
                map.push_back(row);
                row.clear();
            }
        }
        fclose(f);
    }
}

void convertMapToTensor(const std::vector<std::vector<int>>& map, torch::Tensor& map_tensor) {
    // convert the map to a tensor
    map_tensor = torch::zeros({static_cast<long>(map.size()), static_cast<long>(map.at(0).size())});
    for (int i {0}; i < map.size(); i++) {
        for (int j {0}; j < map.at(0).size(); j++) {
            map_tensor[i][j] = map.at(i).at(j);
        }
    }
}


void loadTorchModel(const std::string& model_path, torch::jit::script::Module& model) {
    try {
        // Deserialize the ScriptModule from a file using torch::jit::load().
        model = torch::jit::load(model_path);
        // Move the model to the GPU
        model.to(torch::kCUDA);
    }
    catch (const c10::Error& e) {
        std::cerr << "error loading the model\n";
    }
}


void runVIN(torch::jit::script::Module& model,
    const torch::Tensor& map_tensor,
    const std::vector<std::vector<double>> &starts,
    const std::vector<std::vector<double>> &goals,
    std::unordered_map<int, PathType>& paths,
    int max_horizon=100) {

    // convert the starts and goals to tensors
    torch::Tensor starts_tensor = torch::zeros({static_cast<long>(starts.size()), 2});
    torch::Tensor goals_tensor = torch::zeros({static_cast<long>(goals.size()), 2});
    for (int i {0}; i < starts.size(); i++) {
        starts_tensor[i][0] = starts.at(i).at(0);
        starts_tensor[i][1] = starts.at(i).at(1);
        goals_tensor[i][0] = goals.at(i).at(0);
        goals_tensor[i][1] = goals.at(i).at(1);
    }
    auto action_type = ActionType2dRob();

    for (int i {0}; i < starts.size(); i++) {
        // generate value prior map (everything is -1 besides the goal which is 10)
        torch::Tensor value_prior = torch::full({map_tensor.size(0), map_tensor.size(1)}, -1);
        value_prior[static_cast<long>(goals.at(i).at(0))][static_cast<long>(goals.at(i).at(1))] = 10;
        auto start = starts_tensor[i];
        auto goal = goals_tensor[i];
        // reshape the map tensor to be 1x1xHxW and the value prior to be 1x1xHxW and append them together in dim 1
        torch::Tensor input = torch::cat({map_tensor.unsqueeze(0).unsqueeze(0),
            value_prior.unsqueeze(0).unsqueeze(0)}, 1);
        auto state_x = torch::zeros({1});
        auto state_y = torch::zeros({1});
        state_x[0] = start[0];
        state_y[0] = start[1];
        // to device
        input = input.to(torch::kCUDA);
        state_x = state_x.to(torch::kCUDA);
        state_y = state_y.to(torch::kCUDA);
        torch::Tensor k = torch::tensor({36}).to(torch::kCUDA);
        // run the model
        PathType path;
        // check the time it takes
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        for (int t {0}; t < max_horizon; t++) {
            std::vector<torch::jit::IValue> inputs;
            inputs.emplace_back(input);
            inputs.emplace_back(state_x);
            inputs.emplace_back(state_y);
            inputs.emplace_back(k);
            // forward pass
            auto output = model.forward(inputs).toTuple();
            // Get the action probabilities (the second element in the tuple)
            auto action_probs = output->elements()[1].toTensor();
            // Get the max action
            auto max_action = action_probs.max(1);
            int action_index = std::get<1>(max_action).item<int>();
            auto action = action_type.getPrimActions().at(action_index);

            // Get the next state
            auto next_state = torch::zeros({2});
            next_state[0] = state_x[0] + action[0];
            next_state[1] = state_y[0] + action[1];
            path.emplace_back(std::vector<double>{next_state[0].item<double>(), next_state[1].item<double>()});
            // check if the goal is reached
            if (next_state[0].item<double>() == goal[0].item<double>() &&
                next_state[1].item<double>() == goal[1].item<double>()) {
                // paths[i] = path;
                std::cout << GREEN << "Goal reached!" << RESET << std::endl;
                break;
            }
            state_x = next_state[0].unsqueeze(0);
            state_y = next_state[1].unsqueeze(0);
        }
        path.push_back(std::vector{goal[0].item<double>(), goal[1].item<double>()});
        paths[i] = path;
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
    }

}


// void runVINBatch(torch::jit::script::Module& model,
//     torch::Tensor& map_tensor,
//     const std::vector<std::vector<double>> &starts,
//     const std::vector<std::vector<double>> &goals,
//     std::unordered_map<int, PathType>& paths,
//     int max_horizon=100) {
//
//     auto action_type = ActionType2dRob();
//     // convert the starts and goals to tensors
//     torch::Tensor starts_tensor = torch::zeros({static_cast<long>(starts.size()), 2});
//     torch::Tensor goals_tensor = torch::zeros({static_cast<long>(goals.size()), 2});
//     for (int i {0}; i < starts.size(); i++) {
//         starts_tensor[i][0] = starts.at(i).at(0);
//         starts_tensor[i][1] = starts.at(i).at(1);
//         goals_tensor[i][0] = goals.at(i).at(0);
//         goals_tensor[i][1] = goals.at(i).at(1);
//     }
//     torch::Tensor value_prior = torch::full({map_tensor.size(0), map_tensor.size(1)}, -1);
//     // repeat the value prior tensor to be the same size as the batch size, which is the number of starts
//     value_prior = value_prior.unsqueeze(0).repeat({static_cast<long>(starts.size()), 1, 1});
//     // set the goal values
//     for (int i {0}; i < goals.size(); i++) {
//         value_prior[i][static_cast<long>(goals.at(i).at(0))][static_cast<long>(goals.at(i).at(1))] = 10;
//     }
//     // reshape the map tensor to be Bx1xHxW and the value prior to be Bx1xHxW and append them together in dim 1
//     map_tensor = map_tensor.unsqueeze(0).repeat({static_cast<long>(starts.size()), 1, 1});
//     torch::Tensor input = torch::cat({map_tensor.unsqueeze(1), value_prior.unsqueeze(1)}, 1);
//     auto state_x = torch::zeros({static_cast<long>(starts.size()), 1});
//     auto state_y = torch::zeros({static_cast<long>(starts.size()), 1});
//     for (int i {0}; i < starts.size(); i++) {
//         state_x[i][0] = starts.at(i).at(0);
//         state_y[i][0] = starts.at(i).at(1);
//     }
//     // to device
//     std::cout << "Moving: " << input << std::endl;
//     input = input.to(torch::kCUDA);
//     std::cout << "Moving: " << state_x << std::endl;
//
//     state_x = state_x.to(torch::kCUDA);
//     state_y = state_y.to(torch::kCUDA);
//     torch::Tensor k = torch::tensor({36}).to(torch::kCUDA);
//     // run the model
//     PathType path;
//     // check the time it takes
//     std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
//     for (int t {0}; t < max_horizon; t++) {
//         std::vector<torch::jit::IValue> inputs;
//         inputs.emplace_back(input);
//         inputs.emplace_back(state_x);
//         inputs.emplace_back(state_y);
//         inputs.emplace_back(k);
//         // forward pass
//         auto output = model.forward(inputs).toTuple();
//         // Get the action probabilities (the second element in the tuple)
//         auto action_probs = output->elements()[1].toTensor();
//         // Get the max action
//         auto max_action = action_probs.max(1);
//         auto action_indices = std::get<1>(max_action);
//         auto actions = action_type.getPrimActions();
//         for (int i {0}; i < action_indices.size(0); i++) {
//             int action_index = action_indices[i].item<int>();
//             auto action = actions.at(action_index);
//             // Get the next state
//             auto next_state = torch::zeros({2});
//             next_state[0] = state_x[i][0] + action[0];
//             next_state[1] = state_y[i][0] + action[1];
//             path.emplace_back(std::vector<double>{next_state[0].item<double>(), next_state[1].item<double>()});
//             // check if the goal is reached
//             if (next_state[0].item<double>() == goals_tensor[i][0].item<double>() &&
//                 next_state[1].item<double>() == goals_tensor[i][1].item<double>()) {
//                 // paths[i] = path;
//                 std::cout << GREEN << "Goal reached!" << RESET << std::endl;
//                 break;
//             }
//             state_x[i][0] = next_state[0];
//             state_y[i][0] = next_state[1];
//         }
//     }
//     path.push_back(std::vector{goals_tensor[0][0].item<double>(), goals_tensor[0][1].item<double>()});
//     paths[0] = path;
//     std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
//     std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
//
// }


int main(int argc, char** argv) {

    if (argc < 4) {
        std::cout << "Usage: " << argv[0] << " <map_file> <model_path> <num_runs>" << std::endl;
        return 0;
    }

    std::string map_file = argv[1];
    std::string model_path = argv[2];
    int num_runs = std::stoi(argv[3]);

    std::vector<std::vector<int>> map;
    load2DGridWorld(map_file, map);

    torch::Tensor map_tensor;
    convertMapToTensor(map, map_tensor);

    torch::jit::script::Module model;
    loadTorchModel(model_path, model);

    std::vector<std::vector<double>> starts, goals;
    sampleStartsGoals(map, starts, goals, num_runs);

    // std::vector<std::vector<std::vector<int>>> path;
    std::unordered_map<int, PathType> paths;
    runVIN(model, map_tensor, starts, goals, paths);
    // runVINBatch(model, map_tensor, starts, goals, paths);

    // the map index is the number before the file extension which is .map
    std::string map_index_str = map_file.substr(map_file.find_last_of("/") + 1);
    map_index_str = map_index_str.substr(0, map_index_str.find_last_of("."));
    map_index_str = map_index_str.substr(map_index_str.find_first_of("0123456789"));
    int map_index = std::stoi(map_index_str);
    std::string path_file = logPaths(paths, map_index, 28);
    boost::filesystem::path full_path(boost::filesystem::current_path() );

    std::string plot_path = full_path.string() + "/../domains/2d_robot_nav/scripts/visualize_paths.py";
    //    std::string command = "python3 " + plot_path + " --filepath " + path_file + " --path_ids 49";
    std::string command = "python3 " + plot_path + " --filepath " + path_file + " --map_type gridworld";
    std::cout << "Running the plot script..." << std::endl;
    auto boo = system(command.c_str());

    return 0;
}