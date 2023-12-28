/*
 * Copyright (C) 2023, Itamar Mishani
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
 * \file   action_space_egraph_mixin.hpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   8/10/23
*/

#ifndef SEARCH_SEARCH_INCLUDE_SEARCH_COMMON_ACTION_SPACE_MIXIN_HPP_
#define SEARCH_SEARCH_INCLUDE_SEARCH_COMMON_ACTION_SPACE_MIXIN_HPP_

#include "search/common/types.hpp"
#include "search/common/experience_graph.hpp"

namespace ims {

/// @class ActionSpaceEgraphMixin class
/// @brief This class is used to define the action space for the experience graph
class ActionSpaceEgraphMixin {
public:

    /// @brief Constructor
    ActionSpaceEgraphMixin() = default;

    /// @brief Destructor
    virtual ~ActionSpaceEgraphMixin() = default;

    /// @brief Load the experience graph from a file
    /// @param path The path to the file
    virtual bool loadEGraph(const std::string &path) = 0;

    /// @brief Get the E-graph nodes
    /// @param nodes The vector to store the nodes in
    virtual void getEGraphNodes(int state_id,
                                std::vector<smpl::ExperienceGraph::node_id> &nodes) = 0;

    /// @brief Definition of the shortcut function
    /// @param first_id The first node id
    /// @param second_id The second node id
    /// @param cost The cost of the shortcut
    virtual bool shortcut(int first_id, int second_id, int &cost) = 0;

    /// @brief Check for shortcut transition
    /// @param first_id The first state id
    /// @param second_id The second state id
    /// @param trans_path The transition path
    /// @return True if shortcut transition exists false otherwise
    virtual bool checkShortcutTransition(int first_id,
                                         int second_id,
                                         PathType &trans_path) = 0;

    /// @brief Definition of the snap function
    /// @param first_id The first node id
    /// @param second_id The second node id
    /// @param cost The cost of the snap
    virtual bool snap(int first_id, int second_id, int &cost) = 0;

    /// @brief Check for snap transition
    /// @param first_id The first state id
    /// @param second_id The second state id
    /// @param trans_path The transition path
    /// @return True if snap transition exists false otherwise
    virtual bool checkSnapTransition(int first_id,
                                     int second_id,
                                     PathType &trans_path) = 0;

    /// @brief Get the experience graph as a const pointer
    virtual const std::shared_ptr<smpl::ExperienceGraph> getExperienceGraph() const = 0;

    /// @brief Get the experience graph as a pointer
    virtual std::shared_ptr<smpl::ExperienceGraph> getExperienceGraph() = 0;

    /// @brief Get the state id of a node
    virtual int getStateID(smpl::ExperienceGraph::node_id n) const = 0;

};
}


#endif //SEARCH_SEARCH_INCLUDE_SEARCH_COMMON_ACTION_SPACE_MIXIN_HPP_
