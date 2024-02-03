#pragma once

#include <search/common/types.hpp>

StateType getStateAtTime(const PathType& path, int time) {
    if (time >= path.size()) {
        return path.back();
    }
    else if (time == -1) {
        return path.back();
    } else if (time < 0) {
        throw std::invalid_argument("Time cannot be negative");
    }
    return path[time];
}

bool isSameState(const StateType& s1, const StateType& s2, bool ignoreTime) {
    for (int i = 0; i < s1.size(); i++) {
        // Ignore last index of the state if ignoreTime is true
        if (ignoreTime && i == s1.size() - 1) {
            continue;
        }

        if (s1[i] != s2[i]) {
            return false;
        }
    }
    return true;
}

bool isVertexConflict(const StateType& s1, const StateType& s2) {
    return isSameState(s1, s2, true);
}

/// @brief Note: Assumes states are from consecutive time steps
/// @param agentA_s1 
/// @param agentA_s2 
/// @param agentB_s1 
/// @param agentB_s2 
/// @return 
bool isEdgeConflict(const StateType& agentA_s1, const StateType& agentA_s2, 
                    const StateType& agentB_s1, const StateType& agentB_s2) {
    return isSameState(agentA_s1, agentB_s2, true) && isSameState(agentA_s2, agentB_s1, true);
}