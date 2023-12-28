/*
 * Copyright (C) 2023, Yorai Shaoul
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
 * \file   distance.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   Nov 2 2023
*/

#pragma once

// Project includes.
#include <search/common/types.hpp>

namespace ims {

double euclideanDistance(const StateType& point1, const StateType& point2){
    double sum = 0;
    for (int i = 0; i < point1.size(); i++){
        sum += pow(point1[i] - point2[i], 2);
    }
    return sqrt(sum);
}

double euclideanDistanceRemoveTime (const StateType& point1, const StateType& point2){
    double sum = 0;
    for (int i = 0; i < point1.size() - 1; i++){
        sum += pow(point1[i] - point2[i], 2);
    }
    return sqrt(sum);
}

double manhattanDistance(const StateType& point1, const StateType& point2){
    double sum = 0;
    for (int i = 0; i < point1.size(); i++){
        sum += abs(point1[i] - point2[i]);
    }
    return sum;
}

double manhattanDistanceRemoveTime(const StateType& point1, const StateType& point2){
    double sum = 0;
    for (int i = 0; i < point1.size() - 1; i++){
        sum += abs(point1[i] - point2[i]);
    }
    return sum;
}

}  // namespace ims