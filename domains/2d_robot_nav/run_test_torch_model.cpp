//
// Created by itamar on 8/20/24.
//

#include <torch/torch.h>
#include <torch/script.h>
#include <boost/filesystem.hpp>
#include <random>
//
//
// void load2DGridWorld(const std::string& map_file, std::vector<std::vector<int>>& map) {
//     // check if the file exists
//     if (!boost::filesystem::exists(map_file)) {
//         std::cout << "The map file does not exist!" << std::endl;
//         return;
//     }
//     FILE *f;
//     f = fopen(map_file.c_str(), "r");
//     if (f)
//     {
//         std::vector<int> row;
//         int val;
//         while (fscanf(f, "%d", &val) != EOF) {
//             // check if end of line
//             row.push_back(val);
//             if (fgetc(f) == '\n') {
//                 map.push_back(row);
//                 row.clear();
//             }
//         }
//         fclose(f);
//     }
// }
//
// void convertMapToTensor(const std::vector<std::vector<int>>& map, torch::Tensor& map_tensor) {
//     // convert the map to a tensor
//     map_tensor = torch::zeros({static_cast<long>(map.size()), static_cast<long>(map.at(0).size())});
//     for (int i {0}; i < map.size(); i++) {
//         for (int j {0}; j < map.at(0).size(); j++) {
//             map_tensor[i][j] = map.at(i).at(j);
//         }
//     }
// }
//
// bool sampleStartsGoals(const std::vector<std::vector<int>>& map, std::vector<std::vector<double>>& starts,
//                        std::vector<std::vector<double>>& goals, int num_runs) {
//     // sample the starts and goals
//     std::random_device rd;
//     std::mt19937 gen(rd());
//     std::uniform_int_distribution<> dis(0, static_cast<int>(map.size()) - 1);
//     for (int i {0}; i < num_runs; i++) {
//         std::vector<double> start = {static_cast<double>(dis(gen)), static_cast<double>(dis(gen))};
//         std::vector<double> goal = {static_cast<double>(dis(gen)), static_cast<double>(dis(gen))};
//         while ((map.at(static_cast<int>(start[0])).at(static_cast<int>(start[1])) > 0) ||
//             (map.at(static_cast<int>(goal[0])).at(static_cast<int>(goal[1])) > 0)) {
//
//             start = {static_cast<double>(dis(gen)), static_cast<double>(dis(gen))};
//             goal = {static_cast<double>(dis(gen)), static_cast<double>(dis(gen))};
//         }
//         starts.push_back(start);
//         goals.push_back(goal);
//     }
//     return true;
// }
//
//
//
// void loadTorchModel(const std::string& model_path, torch::jit::script::Module& model) {
//     try {
//         // Deserialize the ScriptModule from a file using torch::jit::load().
//         model = torch::jit::load(model_path);
//         // Move the model to the GPU
//         model.to(torch::kCUDA);
//     }
//     catch (const c10::Error& e) {
//         std::cerr << "error loading the model\n";
//     }
// }
//
//
// void runVIN(const torch::jit::script::Module& model,
//     const torch::Tensor& map_tensor,
//     const std::vector<std::vector<double>> &starts,
//     const std::vector<std::vector<double>> &goals,
//     std::vector<std::vector<std::vector<int>>>& path,
//     int max_horizon=100) {
//
//     // convert the starts and goals to tensors
//     torch::Tensor starts_tensor = torch::zeros({static_cast<long>(starts.size()), 2});
//     torch::Tensor goals_tensor = torch::zeros({static_cast<long>(goals.size()), 2});
//     for (int i {0}; i < starts.size(); i++) {
//         starts_tensor[i][0] = starts.at(i).at(0);
//         starts_tensor[i][1] = starts.at(i).at(1);
//         goals_tensor[i][0] = goals.at(i).at(0);
//         goals_tensor[i][1] = goals.at(i).at(1);
//     }
//
//     for (int i {0}; i < starts.size(); i++) {
//         // generate value prior map (everything is -1 besides the goal which is 10)
//         torch::Tensor value_prior = torch::full({map_tensor.size(0), map_tensor.size(1)}, -1);
//         value_prior[static_cast<long>(goals.at(i).at(0))][static_cast<long>(goals.at(i).at(1))] = 10;
//         auto start = starts_tensor[i];
//         auto goal = goals_tensor[i];
//         // reshape the map tensor to be 1x1xHxW and the value prior to be 1x1xHxW and append them together in dim 1
//         torch::Tensor input = torch::cat({map_tensor.unsqueeze(0).unsqueeze(0), value_prior.unsqueeze(0).unsqueeze(0)}, 1);
//         auto state_x = start[0];
//         auto state_y = start[1];
//         // to device
//         input = input.to(torch::kCUDA);
//         start = start.to(torch::kCUDA);
//         goal = goal.to(torch::kCUDA);
//         // run the model
//         for (int t {0}; t < max_horizon; t++) {
//             // forward pass
//             auto output = model.forward({input, start, goal});
//         }
//     }
//
// }
//
//
// int main(int argc, char** argv) {
//
//     if (argc < 4) {
//         std::cout << "Usage: " << argv[0] << " <map_file> <model_path> <num_runs>" << std::endl;
//         return 0;
//     }
//
//     std::string map_file = argv[1];
//     std::string model_path = argv[2];
//     int num_runs = std::stoi(argv[3]);
//
//     std::vector<std::vector<int>> map;
//     load2DGridWorld(map_file, map);
//
//     torch::Tensor map_tensor;
//     convertMapToTensor(map, map_tensor);
//
//     torch::jit::script::Module model;
//     loadTorchModel(model_path, model);
//
//     std::vector<std::vector<double>> starts, goals;
//     sampleStartsGoals(map, starts, goals, num_runs);
//
//     std::vector<std::vector<std::vector<int>>> path;
//     runVIN(model, map_tensor, starts, goals, path);
//
//     return 0;
// }