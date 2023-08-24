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
 * \file   conflicts.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   August 14 2023
*/

#ifndef SEARCH_COMMON_COLLISIONS_HPP
#define SEARCH_COMMON_COLLISIONS_HPP

// standard includes
#include <functional>
#include <algorithm>
#include <utility>
#include <vector>
#include <memory>
#include <eigen3/Eigen/Dense>

namespace ims {
    
// TODO(yoraish): change to enum class.
enum class CollisionType {
    UNSET = -1,
    POINT = 0
};

// TODO(yoraish): change to enum class.
enum class BodyType {
    UNSET = -1,
    ROBOT = 0,
    ROBOT_ATTACHED = 1,
    WORLD_OBJECT = 2
};

/// @brief A struct to hold the information about a contact. Its location, the bodies that are in contact, their types, and the normal.
struct Contact{
    // The bodies that are in contact.
    BodyType body_type_0 = BodyType::UNSET;
    BodyType body_type_1 = BodyType::UNSET;
    std::string body_name_0;
    std::string body_name_1;

    // The points of contact from a collision.
    Eigen::Vector3d point;
    Eigen::Vector3d normal;

    // Constructor.
    Contact(std::string body_name_0, std::string body_name_1, BodyType body_type_0, BodyType body_type_1, const Eigen::Vector3d& point, const Eigen::Vector3d& normal) : body_name_0(body_name_0), body_name_1(body_name_1), body_type_0(body_type_0), body_type_1(body_type_1), point(point), normal(normal) {}

    // Another constructor that allows input of the bodies as ints.
    Contact(std::string body_name_0, std::string body_name_1, int body_type_0, int body_type_1, const Eigen::Vector3d& point, const Eigen::Vector3d& normal) : body_name_0(body_name_0), body_name_1(body_name_1), body_type_0(static_cast<BodyType>(body_type_0)), body_type_1(static_cast<BodyType>(body_type_1)), point(point), normal(normal) {}
};

/// @brief A struct to hold the information about a collision. This object, as well as much of this file, is similar in spirit to a MoveIt! Contact object that is within a MoveIt! collision results object.
struct Collision {
    // The type of the collision. 
    CollisionType type = CollisionType::UNSET;

    // The bodies that are in collision.
    BodyType body_type_0 = BodyType::UNSET;
    BodyType body_type_1 = BodyType::UNSET;
    std::string body_name_0;
    std::string body_name_1;

    // The points of contact from a collision.
    std::vector<Contact> contacts;
};

/// @brief A struct to hold the information about a set of collisions. This is a collective, hence not merely a containter, since it also provides some useful functions for working with and organizing the data.
struct CollisionsCollective{
    // Members.
private:
    /// @brief The collisions.
    std::vector<Collision> collisions;

public:
    // Methods.
    /// @brief Add a collision to the collective.
    void addCollision(Collision collision) {
        collisions.push_back(collision);
    }

    /// @brief Get the collisions without making a copy. The returned vector is const to keep the user from modifying the collisions within the collective.
    const std::vector<Collision>& getCollisions() const {
        return collisions;
    }    

    /// @brief Clear the collisions.
    void clear() {
        collisions.clear();
    }

    /// @brief Merge another collective into this one.
    void mergeIn(const CollisionsCollective& other) {
        collisions.insert(collisions.end(), other.collisions.begin(), other.collisions.end());
    }

    /// @brief Get the number of collisions.
    int size() const {
        return collisions.size();
    }
};

} // namespace ims
#endif //SEARCH_COMMON_COLLISIONS_HPP