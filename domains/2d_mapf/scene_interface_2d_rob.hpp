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
 * \file   scene_interface_2d_rob.hpp
 * \author Rishi V. (...@cmu.edu)
 * \date   Oct 04 2023
 */

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
class Scene2DRob : public ims::SceneInterface {
private:
    int num_rows_, num_cols_;
    vector<vector<bool>> occupancy_map_; // Keep separate from cost map for now
    vector<vector<double>> cost_map_;

public:
    Scene2DRob() {}

    explicit Scene2DRob(std::vector<std::vector<int>>& map_) : ims::SceneInterface() {
        std::cout << "Creating scene with map of shape " << map_.size() << ", " << map_[0].size() << std::endl;
        this->map = &map_;
        this->map_size = {(*map).size(), (*map)[0].size()};
    }

    std::vector<std::vector<int>>* map;
    std::vector<size_t> map_size;

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

void Scene2DRob::loadMap(const string& filename) {
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

bool Scene2DRob::isCellValid(double row, double col) {
    if (row < 0 || row >= num_rows_ || col < 0 || col >= num_cols_) {
        return false;
    }
    return !occupancy_map_[int(row)][int(col)];
}

double Scene2DRob::getCellCost(double row, double col) {
    assert(isCellValid(row, col));
    throw std::runtime_error("Not implemented");
}