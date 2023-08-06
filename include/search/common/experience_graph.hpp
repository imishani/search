////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Andrew Dornbush
/// \editor Itamar Mishani
/// \date 8/4/23

#ifndef SEARCH_SEARCH_INCLUDE_SEARCH_COMMON_EXPERIENCE_GRAPH_HPP_
#define SEARCH_SEARCH_INCLUDE_SEARCH_COMMON_EXPERIENCE_GRAPH_HPP_

// standard includes
#include <cstdlib>
#include <utility>
#include <vector>

// Package includes
#include <search/common/types.hpp>

namespace ims {
namespace smpl {

// undirected
struct ExperienceGraph {
    struct Node;
    struct Edge;

    using node_container = std::vector<Node>;
    using edge_container = std::vector<Edge>;

    using node_id = node_container::size_type;
    using edge_id = edge_container::size_type;

    struct Node {
        using adjacency = std::pair<edge_id, node_id>;
        using adjacent_edge_container = std::vector<adjacency>;

        StateType state;
        adjacent_edge_container edges;

        explicit Node(StateType state) : state(std::move(state)) {}
    };

    struct Edge {
        std::vector<StateType> waypoints;
        node_id snode;
        node_id tnode;

        Edge(node_id uid, node_id vid) : waypoints(), snode(uid), tnode(vid) {}

        Edge(const std::vector<StateType> &waypoints, node_id uid, node_id vid)
            : waypoints(waypoints), snode(uid), tnode(vid) {}
    };

    using nodes_size_type = node_container::size_type;
    using edges_size_type = edge_container::size_type;
    using degree_size_type = Node::adjacent_edge_container::size_type;

    struct node_iterator : std::iterator<std::random_access_iterator_tag, node_id> {
        node_id m_id;

        explicit node_iterator(node_id id) : m_id(id) {}

        auto operator*() const -> const value_type;
        auto operator++(int) -> node_iterator;
        auto operator++() -> node_iterator &;

        auto operator+=(difference_type n) -> node_iterator &;
        auto operator-=(difference_type n) -> node_iterator &;
        auto operator-(node_iterator it) -> difference_type;
        bool operator==(node_iterator it) const;
        bool operator!=(node_iterator it) const;
    };

    struct edge_iterator : std::iterator<std::random_access_iterator_tag, edge_id> {
        edge_id m_id;

        explicit edge_iterator(edge_id id) : m_id(id) {}

        auto operator*() const -> const value_type;
        auto operator++(int) -> edge_iterator;
        auto operator++() -> edge_iterator &;

        auto operator+=(difference_type n) -> edge_iterator &;
        auto operator-=(difference_type n) -> edge_iterator &;
        auto operator-(edge_iterator it) -> difference_type;
        bool operator==(edge_iterator it) const;
        bool operator!=(edge_iterator it) const;
    };

    using adjacent_edge_iterator = Node::adjacent_edge_container::const_iterator;

    struct incident_edge_iterator : std::iterator<std::random_access_iterator_tag, edge_id> {
        adjacent_edge_iterator m_it;

        explicit incident_edge_iterator(adjacent_edge_iterator it) : m_it(it) {}

        auto operator*() const -> const value_type;
        auto operator++(int) -> incident_edge_iterator;
        auto operator++() -> incident_edge_iterator &;

        auto operator+=(difference_type n) -> incident_edge_iterator &;
        auto operator-=(difference_type n) -> incident_edge_iterator &;
        auto operator-(incident_edge_iterator it) -> difference_type;
        bool operator==(incident_edge_iterator it) const;
        bool operator!=(incident_edge_iterator it) const;
    };

    struct adjacency_iterator : std::iterator<std::random_access_iterator_tag, node_id> {
        adjacent_edge_iterator m_it;

        explicit adjacency_iterator(adjacent_edge_iterator it) : m_it(it) {}

        auto operator*() const -> const value_type;
        auto operator++(int) -> adjacency_iterator;
        auto operator++() -> adjacency_iterator &;

        auto operator+=(difference_type n) -> adjacency_iterator &;
        auto operator-=(difference_type n) -> adjacency_iterator &;
        auto operator-(adjacency_iterator it) -> difference_type;
        bool operator==(adjacency_iterator it) const;
        bool operator!=(adjacency_iterator it) const;
    };

    std::vector<Node> m_nodes;
    std::vector<Edge> m_edges;

    // cached storage for mapping edge ids to their id adjustments during node
    // removal
    std::vector<std::ptrdiff_t> m_shift;

    auto nodes() const -> std::pair<node_iterator, node_iterator>;
    auto edges() const -> std::pair<edge_iterator, edge_iterator>;

    auto edges(node_id id) const
    -> std::pair<incident_edge_iterator, incident_edge_iterator>;
    auto adjacent_nodes(node_id id) const
    -> std::pair<adjacency_iterator, adjacency_iterator>;

    auto degree(node_id id) const -> degree_size_type;
    auto source(edge_id id) const -> node_id;
    auto target(edge_id id) const -> node_id;

    auto num_nodes() const -> nodes_size_type { return m_nodes.size(); }
    auto num_edges() const -> edges_size_type { return m_edges.size(); }

    bool edge(node_id uid, node_id vid) const;

    auto insert_node(const StateType &state) -> node_id;
    void erase_node(node_id id);

    auto insert_edge(node_id uid, node_id vid) -> edge_id;
    auto insert_edge(
        node_id uid,
        node_id vid,
        const std::vector<StateType> &path)
    -> edge_id;

    void erase_edge(node_id uid, node_id vid);
    void erase_edge(edge_id id);

    void clear();

    auto state(node_id id) const -> const StateType &;
    auto state(node_id id) -> StateType &;

    auto waypoints(edge_id id) const -> const std::vector<StateType> &;
    auto waypoints(edge_id id) -> std::vector<StateType> &;
};
} // namespace smpl
}

#endif //SEARCH_SEARCH_INCLUDE_SEARCH_COMMON_EXPERIENCE_GRAPH_HPP_
