#pragma once

#include <vector>
#include <string>
#include <assert.h>
#include <stdexcept>
using std::string;
using std::vector;

/// @brief 
/// @note Saving data in row,col format, not x,y to prevent confusion
class CollisionChecker2D {
private:
    int m_num_rows, m_num_cols;
    vector<vector<bool>> m_occupancy_map; // Keep separate from cost map for now
    vector<vector<double>> m_cost_map;

public:
    CollisionChecker2D() {}

    void loadMap(const string& filename);
    bool isCellValid(double row, double col);
    double getCellCost(double row, double col);

    // TODO: Add visualization functions later on
};



////////////////// Implementations Below //////////////////////

void CollisionChecker2D::loadMap(const string& filename) {
    FILE *f;
    f = fopen(filename.c_str(), "r");

    if (f) {
        if (fscanf(f, "type octile\nheight %d\nwidth %d\nmap\n", &m_num_rows, &m_num_cols)) {
            m_occupancy_map.resize(m_num_rows, vector<bool>(m_num_cols));

            for (int row = 0; row < m_num_rows; row++) {
                for (int col = 0; col < m_num_cols; col++) {
                    char c;
                    do {
                        fscanf(f, "%c", &c);
                    } while (isspace(c));

                    if (c == '.' || c == 'G' || c == 'S' || c == 'T') {
                        m_occupancy_map[row][col] = false;
                    } else {
                        m_occupancy_map[row][col] = true;
                    }
                }
            }
        }
        fclose(f);
    } else {
        throw std::runtime_error("Could not open file " + filename);
    }
}

bool CollisionChecker2D::isCellValid(double row, double col) {
    if (row < 0 || row >= m_num_rows || col < 0 || col >= m_num_cols) {
        return false;
    }
    return !m_occupancy_map[int(row)][int(col)];
}

double CollisionChecker2D::getCellCost(double row, double col) {
    assert(isCellValid(row, col));
    throw std::runtime_error("Not implemented");
}