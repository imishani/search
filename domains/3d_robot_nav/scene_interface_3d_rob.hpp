/*
 * Copyright (C) 2024, Julius A.
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
 * \file   scene_interface_3d_rob.hpp
 * \author Julius A. (...@cmu.edu)
 * \date   Aug 26 2024
 */

#pragma once

#include <search/common/scene_interface.hpp>
#include <vector>
#include <string>
#include <assert.h>
#include <stdexcept>

using std::string;
using std::vector;

class Scene3DRob : public ims::SceneInterface {
private:
    int num_rows_, num_cols_, num_layers_;
    vector<vector<vector<bool>>> occupancy_map_; // 3D occupancy map
    vector<vector<vector<double>>> cost_map_;    // 3D cost map

public:
    Scene3DRob() = default;
    explicit Scene3DRob(const vector<vector<vector<int>>>& map);

    void loadMap(const string& filename);
    bool isCellValid(double row, double col, double layer);
    double getCellCost(double row, double col, double layer);
    vector<vector<vector<bool>>> getOccupancyMap();
};

// Implementation

Scene3DRob::Scene3DRob(const vector<vector<vector<int>>>& map) {
    num_layers_ = map.size();
    num_rows_ = map[0].size();
    num_cols_ = map[0][0].size();

    occupancy_map_.resize(num_layers_, vector<vector<bool>>(num_rows_, vector<bool>(num_cols_)));

    for (int z = 0; z < num_layers_; ++z) {
        for (int y = 0; y < num_rows_; ++y) {
            for (int x = 0; x < num_cols_; ++x) {
                occupancy_map_[z][y][x] = (map[z][y][x] == 0);
            }
        }
    }
}

void Scene3DRob::loadMap(const string& filename) {
    FILE *f;
    f = fopen(filename.c_str(), "r");
    if (f) {
        if (fscanf(f, "type octile3D\nheight %d\nwidth %d\ndepth %d\nmap\n", 
                   &num_rows_, &num_cols_, &num_layers_)) {
            
            occupancy_map_.resize(num_layers_, 
                vector<vector<bool>>(num_rows_, vector<bool>(num_cols_)));

            for (int layer = 0; layer < num_layers_; layer++) {
                for (int row = 0; row < num_rows_; row++) {
                    for (int col = 0; col < num_cols_; col++) {
                        char c;
                        do {
                            fscanf(f, "%c", &c);
                        } while (isspace(c));
                        
                        occupancy_map_[layer][row][col] = (c == '.' || c == 'G' || c == 'S' || c == 'T');
                    }
                }
            }
        }
        fclose(f);
    } else {
        throw std::runtime_error("Could not open file " + filename);
    }
}

bool Scene3DRob::isCellValid(double row, double col, double layer) {
    if (row < 0 || row >= num_rows_ || col < 0 || col >= num_cols_ || layer < 0 || layer >= num_layers_) {
        return false;
    }
    return occupancy_map_[int(layer)][int(row)][int(col)];
}

double Scene3DRob::getCellCost(double row, double col, double layer) {
    assert(isCellValid(row, col, layer));
    throw std::runtime_error("Not implemented");
}

vector<vector<vector<bool>>> Scene3DRob::getOccupancyMap() {
    return occupancy_map_;
}