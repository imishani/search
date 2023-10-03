#pragma once

#include <search/common/scene_interface.hpp>

#include <vector>
#include <string>
#include <assert.h>
#include <stdexcept>
using std::string;
using std::vector;

/// @brief Simple 2D CollisionChecker that parses a map file and checks if a cell is valid or not.
/// @note Saving data in row,col format, not x,y to prevent confusion.
class SceneInterface2DGrid : public ims::SceneInterface {
private:
    int num_rows_, num_cols_;
    vector<vector<bool>> occupancy_map_; // Keep separate from cost map for now
    vector<vector<double>> cost_map_;

public:
    SceneInterface2DGrid() {}

    /// @brief Takes in a filepath and parses and loads it into the occupancy map.
    /// @param filename 
    void loadMap(const string& filename);

    /// @brief Returns if the cell is valid or not (in bounds, not an obstacle).
    /// @param row 
    /// @param col 
    /// @return 
    bool isCellValid(double row, double col);

    /// @brief TODO: NOT SUPPORT YET BUT KEEPING FOR FUTURE PROOFING.
    /// @param row 
    /// @param col 
    /// @return 
    double getCellCost(double row, double col);

    // TODO: Add visualization functions later on
};



////////////////// Implementations Below //////////////////////

void SceneInterface2DGrid::loadMap(const string& filename) {
    FILE *f;
    f = fopen(filename.c_str(), "r");

    if (f) {
        if (fscanf(f, "type octile\nheight %d\nwidth %d\nmap\n", &num_rows_, &num_cols_)) {
            occupancy_map_.resize(num_rows_, vector<bool>(num_cols_));

            for (int row = 0; row < num_rows_; row++) {
                for (int col = 0; col < num_cols_; col++) {
                    char c;
                    do {
                        fscanf(f, "%c", &c);
                    } while (isspace(c));

                    if (c == '.' || c == 'G' || c == 'S' || c == 'T') {
                        occupancy_map_[row][col] = false;
                    } else {
                        occupancy_map_[row][col] = true;
                    }
                }
            }
        }
        fclose(f);
    } else {
        throw std::runtime_error("Could not open file " + filename);
    }
}

bool SceneInterface2DGrid::isCellValid(double row, double col) {
    if (row < 0 || row >= num_rows_ || col < 0 || col >= num_cols_) {
        return false;
    }
    return !occupancy_map_[int(row)][int(col)];
}

double SceneInterface2DGrid::getCellCost(double row, double col) {
    assert(isCellValid(row, col));
    throw std::runtime_error("Not implemented");
}