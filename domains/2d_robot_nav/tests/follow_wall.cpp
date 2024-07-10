
#include <iostream>
#include <vector>
#include <boost/filesystem.hpp>
#include <chrono>

#include "../utils.hpp"
#include "../controllers_2d.hpp"

bool saveMap(const std::vector<std::vector<int>>& map, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cout << "Error opening file " << filename << std::endl;
        return false;
    }

    for (const auto & i : map) {
        for (int j : i) {
            file << j << " ";
        }
        file << std::endl;
    }

    file.close();
    return true;
}

bool saveTrajectories(const std::vector<std::vector<std::pair<int, int>>>& trajectories, const std::string& filename) {
std::ofstream file(filename);
    if (!file.is_open()) {
        std::cout << "Error opening file " << filename << std::endl;
        return false;
    }

    for (const auto & trajectory : trajectories) {
        for (const auto& point : trajectory) {
            file << point.first << " " << point.second << " ";
        }
        file << std::endl;
    }

    file.close();
    return true;
}

int main(int argc, char** argv) {

    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <map_file> <num_runs> <scale> <path>" << std::endl;
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

    std::vector<std::string> starts_goals_path = {full_path.string() + "/../domains/2d_robot_nav/data/hrt201n/",
                                                  full_path.string() + "/../domains/2d_robot_nav/data/den501d/",
                                                  full_path.string() + "/../domains/2d_robot_nav/data/den520d/",
                                                  full_path.string() + "/../domains/2d_robot_nav/data/ht_chantry/",
                                                  full_path.string() + "/../domains/2d_robot_nav/data/brc203d/",
    };

    int map_index = std::stoi(argv[1]);
    int scale = std::stoi(argv[3]);
    std::string path = starts_goals_path[map_index];

    std::string map_file = maps[map_index];

    std::string type;
    int width, height;
    std::vector<std::vector<int>> gridMap = loadMap(map_file.c_str(), type, width, height, scale);

    std::cout << "Width: " << width << " Height: " << height << std::endl;

    // time to detect walls
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
    std::vector<std::vector<int>> wallMap;
    std::vector<std::vector<std::pair<int, int>> > trajectories;
    ims::detectWalls(gridMap, wallMap, trajectories, 10);
    end = std::chrono::system_clock::now();
    std::cout << "Time taken to detect walls: " <<
    std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;

    std::cout << "Detected Wall Map" << std::endl;
    // save the wall map to a file
    saveMap(wallMap, "../domains/2d_robot_nav/scripts/wall_map.txt");
    saveTrajectories(trajectories, "../domains/2d_robot_nav/scripts/trajectories.txt");
}