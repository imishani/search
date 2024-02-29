/*
 * Copyright (C) 2024, Yorai Shaoul
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
 * \file   test_ecbs.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   2024-02-29
*/

// Test includes.
#include <catch2/catch.hpp>

// Project includes.
#include <search/common/distance.hpp>
#include <search/common/types.hpp>

TEST_CASE("Test Euclidean Distance.") {
    StateType state0 = {0, 0, 0};
    StateType state1 = {1, 1, 1};
    double distance = ims::euclideanDistance(state0, state1);
    REQUIRE(distance == Approx(1.73205));
}

TEST_CASE("Test Euclidean Distance Remove Time.") {
    StateType state0 = {0, 0, 0, 0};
    StateType state1 = {1, 1, 1, 100};
    double distance = ims::euclideanDistanceRemoveTime(state0, state1);
    REQUIRE(distance == Approx(1.73205));
}

TEST_CASE("Test Manhattan Distance.") {
    StateType state0 = {0, 0, 0};
    StateType state1 = {1, 1, 1};
    double distance = ims::manhattanDistance(state0, state1);
    REQUIRE(distance == 3);
}

TEST_CASE("Test Manhattan Distance Remove Time.") {
    StateType state0 = {0, 0, 0, 0};
    StateType state1 = {1, 2, 1, 100};
    double distance = ims::manhattanDistanceRemoveTime(state0, state1);
    REQUIRE(distance == 4);
}

