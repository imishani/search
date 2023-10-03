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
 * \file   world_objects.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   August 15 2023
*/


#ifndef SEARCH_WORLD_OBJECTS_HPP
#define SEARCH_WORLD_OBJECTS_HPP

#include <vector>
#include <limits>
#include <unordered_map>
#include <Eigen/Dense>

enum class WorldObjectType{
    UNKNOWN,
    SPHERE,
    BOX
};

struct WorldObject{
    WorldObjectType type = WorldObjectType::UNKNOWN;
    std::string name;
};

struct SphereWorldObject : public WorldObject{
    double radius;
    Eigen::Vector3d origin;
    std::string name;

    SphereWorldObject(Eigen::Vector3d origin, double radius) : origin(origin), radius(radius) {
        type = WorldObjectType::SPHERE;
    }

    SphereWorldObject(Eigen::Vector3d origin, double radius, std::string name) : origin(origin), radius(radius) {
        type = WorldObjectType::SPHERE;
        this->name = name;
    }
};

struct BoxWorldObject : public WorldObject{
    Eigen::Vector3d origin;
    Eigen::Vector3d size;
    std::string name;

    BoxWorldObject(Eigen::Vector3d origin, Eigen::Vector3d size) : origin(origin), size(size) {
        type = WorldObjectType::BOX;
    }

    BoxWorldObject(Eigen::Vector3d origin, Eigen::Vector3d size, std::string name) : origin(origin), size(size) {
        type = WorldObjectType::BOX;
        this->name = name;
    }
};

#endif //SEARCH_WORLD_OBJECTS_HPP
