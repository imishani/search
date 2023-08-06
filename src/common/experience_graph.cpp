#include "search/common/experience_graph.hpp"

#include <assert.h>
#include <algorithm>
#include <stdexcept>

namespace ims {
namespace smpl {

///////////////////
// node_iterator //
///////////////////

auto ExperienceGraph::node_iterator::operator*() const -> const value_type {
    return m_id;
}

auto ExperienceGraph::node_iterator::operator++(int) -> node_iterator {
    return node_iterator(m_id++);
}

auto ExperienceGraph::node_iterator::operator++() -> node_iterator & {
    ++m_id;
    return *this;
}

auto ExperienceGraph::node_iterator::operator+=(difference_type n)
-> node_iterator & {
    m_id += n;
    return *this;
}

auto ExperienceGraph::node_iterator::operator-=(difference_type n)
-> node_iterator & {
    m_id -= n;
    return *this;
}

auto ExperienceGraph::node_iterator::operator-(node_iterator it)
-> difference_type {
    return (difference_type) m_id - (difference_type) it.m_id;
}

bool ExperienceGraph::node_iterator::operator==(node_iterator it) const {
    return m_id == it.m_id;
}

bool ExperienceGraph::node_iterator::operator!=(node_iterator it) const {
    return m_id != it.m_id;
}

///////////////////
// edge_iterator //
///////////////////

auto ExperienceGraph::edge_iterator::operator*() const -> const value_type {
    return m_id;
}

auto ExperienceGraph::edge_iterator::operator++(int) -> edge_iterator {
    return edge_iterator(m_id++);
}

auto ExperienceGraph::edge_iterator::operator++() -> edge_iterator & {
    ++m_id;
    return *this;
}

auto ExperienceGraph::edge_iterator::operator+=(difference_type n)
-> edge_iterator & {
    m_id += n;
    return *this;
}

auto ExperienceGraph::edge_iterator::operator-=(difference_type n)
-> edge_iterator & {
    m_id -= n;
    return *this;
}

auto ExperienceGraph::edge_iterator::operator-(edge_iterator it)
-> difference_type {
    return (difference_type) m_id - (difference_type) it.m_id;
}

bool ExperienceGraph::edge_iterator::operator==(edge_iterator it) const {
    return m_id == it.m_id;
}

bool ExperienceGraph::edge_iterator::operator!=(edge_iterator it) const {
    return m_id != it.m_id;
}

////////////////////////
// adjacency_iterator //
////////////////////////

auto ExperienceGraph::adjacency_iterator::operator*() const -> const value_type {
    return m_it->second;
}

auto ExperienceGraph::adjacency_iterator::operator++(int) -> adjacency_iterator {
    adjacency_iterator it(m_it);
    ++m_it;
    return it;
}

auto ExperienceGraph::adjacency_iterator::operator++() -> adjacency_iterator & {
    ++m_it;
    return *this;
}

auto ExperienceGraph::adjacency_iterator::operator+=(difference_type n)
-> adjacency_iterator & {
    m_it += n;
    return *this;
}

auto ExperienceGraph::adjacency_iterator::operator-=(difference_type n)
-> adjacency_iterator & {
    return operator+=(-n);
}

auto ExperienceGraph::adjacency_iterator::operator-(adjacency_iterator it)
-> difference_type {
    return m_it - it.m_it;
}

bool ExperienceGraph::adjacency_iterator::operator==(
    adjacency_iterator it) const {
    return it.m_it == m_it;
}

bool ExperienceGraph::adjacency_iterator::operator!=(
    adjacency_iterator it) const {
    return it.m_it != m_it;
}

////////////////////////////
// incident_edge_iterator //
////////////////////////////

auto ExperienceGraph::incident_edge_iterator::operator*() const
-> const value_type {
    return m_it->first;
}

auto ExperienceGraph::incident_edge_iterator::operator++(int)
-> incident_edge_iterator {
    incident_edge_iterator it(m_it);
    ++m_it;
    return it;
}

auto ExperienceGraph::incident_edge_iterator::operator++()
-> incident_edge_iterator & {
    ++m_it;
    return *this;
}

auto ExperienceGraph::incident_edge_iterator::operator+=(difference_type n)
-> incident_edge_iterator & {
    m_it += n;
    return *this;
}

auto ExperienceGraph::incident_edge_iterator::operator-=(difference_type n)
-> incident_edge_iterator & {
    return operator+=(-n);
}

auto ExperienceGraph::incident_edge_iterator::operator-(
    incident_edge_iterator it)
-> difference_type {
    return m_it - it.m_it;
}

bool ExperienceGraph::incident_edge_iterator::operator==(
    incident_edge_iterator it) const {
    return it.m_it == m_it;
}

bool ExperienceGraph::incident_edge_iterator::operator!=(
    incident_edge_iterator it) const {
    return it.m_it != m_it;
}

/////////////////////
// ExperienceGraph //
/////////////////////

static
void insert_incident_edge(
    ExperienceGraph *egraph,
    ExperienceGraph::edge_id eid,
    ExperienceGraph::node_id uid,
    ExperienceGraph::node_id vid) {
    if (vid != uid) {
        egraph->m_nodes[uid].edges.emplace_back(eid, vid);
        egraph->m_nodes[vid].edges.emplace_back(eid, uid);
    } else {
        egraph->m_nodes[uid].edges.emplace_back(eid, vid);
    }
}

/// Return a pair of iterators to the range of nodes in the graph.
auto ExperienceGraph::nodes() const -> std::pair<node_iterator, node_iterator> {
    return std::make_pair(
        node_iterator(0),
        node_iterator(m_nodes.size()));
}

/// Return a pair of iterators to the range of edges in the graph.
auto ExperienceGraph::edges() const -> std::pair<edge_iterator, edge_iterator> {
    return std::make_pair(
        edge_iterator(0),
        edge_iterator(m_edges.size()));
}

/// Return a pair of iterators to the range of incident edges for a node.
auto ExperienceGraph::edges(node_id id) const
-> std::pair<incident_edge_iterator, incident_edge_iterator> {
    return std::make_pair(
        incident_edge_iterator(m_nodes[id].edges.begin()),
        incident_edge_iterator(m_nodes[id].edges.end()));
}

/// Return a pair of iterators to the range of adjacent nodes for a node.
auto ExperienceGraph::adjacent_nodes(node_id id) const
-> std::pair<adjacency_iterator, adjacency_iterator> {
    return std::make_pair(
        adjacency_iterator(m_nodes[id].edges.begin()),
        adjacency_iterator(m_nodes[id].edges.end()));
}

/// Return the degree (number of incident edges) of a node.
auto ExperienceGraph::degree(node_id id) const -> degree_size_type {
    return m_nodes[id].edges.size();
}

/// Return the id of an edge's source node.
auto ExperienceGraph::source(edge_id id) const -> node_id {
    return m_edges[id].snode;
}

/// Return the id of an edge's target node.
auto ExperienceGraph::target(edge_id id) const -> node_id {
    return m_edges[id].tnode;
}

/// Test if an edge exists between two nodes.
bool ExperienceGraph::edge(node_id uid, node_id vid) const {
    if (uid >= m_nodes.size() || vid >= m_nodes.size()) {
        throw std::out_of_range("ExperienceGraph::edge called with invalid node ids");
    }

    if (m_nodes[uid].edges.size() < m_nodes[vid].edges.size()) {
        for (auto &adj : m_nodes[uid].edges) {
            if (adj.second == vid) {
                return true;
            }
        }
    } else {
        for (auto &adj : m_nodes[vid].edges) {
            if (adj.second == uid) {
                return true;
            }
        }
    }
    return false;
}

/// Insert a node.
auto ExperienceGraph::insert_node(const StateType &state) -> node_id {
    m_nodes.emplace_back(state);
    return m_nodes.size() - 1;
}

/// Erase a node. Node iterators and ids after the erased node are invalidated.
/// Edge iterators and ids after the lowest removed incident edge are
/// invalidated. All incident edge and adjacenct node iterators are invalidated.
void ExperienceGraph::erase_node(node_id id) {
    if (id >= m_nodes.size()) {
        throw std::out_of_range("ExperienceGraph::erase_node called with invalid node id");
    }

    auto &rem_node = m_nodes[id];

    // the number of edges to be removed and the smallest id, for updating
    // adjacency edge ids
    if (!rem_node.edges.empty()) {
        auto will_remove = [id](const Edge &e) {
            return e.snode == id | e.tnode == id;
        };

        // map from edge id to how many positions it will shift down
        m_shift.assign(m_edges.size(), 0);
        std::ptrdiff_t shift = 0;

        // simultaneously remove deleted edges and map from original edge ids
        // to the amount they must be shifted later
        auto first = std::find_if(m_edges.begin(), m_edges.end(), will_remove);
        if (first != m_edges.end()) {
            // for every element later in the sequence
            auto i = first;
            auto j = std::next(m_shift.begin(), std::distance(m_edges.begin(), first));
            ++shift;
            // i = remainder range iterator
            // j = corresponding shift array tracking following i
            for (; ++j, ++i != m_edges.end();) {
                *j = shift;
                if (!(will_remove(*i))) {
                    // shift this element and update the retained range
                    *first++ = std::move(*i);
                } else {
                    ++shift;
                }
            }
        }
        m_edges.erase(first, m_edges.end());

        for (auto &node : m_nodes) {
            // skip the node to be removed for a small optimization
            if (&node == &rem_node) {
                continue;
            }

            // remove adjacency entries
            auto it = std::remove_if(
                node.edges.begin(), node.edges.end(),
                [&](const Node::adjacency &a) {
                    return a.second == id;
                });
            node.edges.resize(std::distance(node.edges.begin(), it));

            // update node and edge ids
            for (Node::adjacency &a : node.edges) {
                a.first -= m_shift[a.first];
                if (a.second > id) {
                    --a.second;
                }
            }
        }
    }

    // update the node ids stored in edges
    for (auto &edge : m_edges) {
        if (edge.snode > id) {
            --edge.snode;
        }
        if (edge.tnode > id) {
            --edge.tnode;
        }
    }

    // remove the node
    m_nodes.erase(m_nodes.begin() + id);
}

/// Insert an edge, allowing parallel edges and self-loops.
/// \return The ID of the inserted edge
auto ExperienceGraph::insert_edge(node_id uid, node_id vid) -> edge_id {
    if (uid >= m_nodes.size() || vid >= m_nodes.size()) {
        throw std::out_of_range("ExperienceGraph::insert_edge called with invalid node ids");
    }

    m_edges.emplace_back(uid, vid);
    ExperienceGraph::edge_id eid = m_edges.size() - 1;
    insert_incident_edge(this, eid, uid, vid);
    return eid;
}

/// Insert an edge, allowing parallel edges and self-loops.
/// \return The ID of the inserted edge
auto ExperienceGraph::insert_edge(
    node_id uid,
    node_id vid,
    const std::vector<StateType> &path)
-> edge_id {
    if (uid >= m_nodes.size() || vid >= m_nodes.size()) {
        throw std::out_of_range("ExperienceGraph::insert_edge called with invalid node ids");
    }

    m_edges.emplace_back(path, uid, vid);
    ExperienceGraph::edge_id eid = m_edges.size() - 1;
    insert_incident_edge(this, eid, uid, vid);
    return eid;
}

/// Erase the edge between two nodes, if one exists. All incident edge and
/// adjacent node iterators for the source and target nodes, edge iterators
/// pointing to edges after the erased edge, and edge ids greater than the
/// erased edge id are invalidated. TODO: erase all parallel edges?
void ExperienceGraph::erase_edge(node_id uid, node_id vid) {
    if (uid >= m_nodes.size() || vid >= m_nodes.size()) {
        throw std::out_of_range("ExperienceGraph::erase_edge called with invalid node ids");
    }

    // search through the smaller of the two adjacency lists for the edge id
    // and call erase_edge on it when found
    if (m_nodes[uid].edges.size() < m_nodes[vid].edges.size()) {
        auto &out_edges = m_nodes[uid].edges;
        auto it = std::find_if(
            out_edges.begin(),
            out_edges.end(),
            [&](const Node::adjacency &a) {
                return a.second == vid;
            });
        if (it != out_edges.end()) {
            erase_edge(it->first);
        }
    } else {
        auto &out_edges = m_nodes[vid].edges;
        auto it = std::find_if(
            out_edges.begin(),
            out_edges.end(),
            [&](const Node::adjacency &a) {
                return a.second == uid;
            });
        if (it != out_edges.end()) {
            erase_edge(it->first);
        }
    }
}

/// Erase an edge. All incident edge and adjacent node iterators for the source
/// and target nodes, edge iterators pointing to edges after the erased edge,
/// and edge ids greater than the erased edge id are invalidated.
void ExperienceGraph::erase_edge(edge_id id) {
    if (id >= m_edges.size()) {
        throw std::out_of_range("ExperienceGraph::erase_edge called with invalid edge id");
    }

    auto &e = m_edges[id];

    // remove incident edge from source node and update edge ids
    assert(e.snode < m_nodes.size());
    auto ait = std::remove_if(
        m_nodes[e.snode].edges.begin(),
        m_nodes[e.snode].edges.end(),
        [&](Node::adjacency &a) {
            bool rem = a.first == id;
            if (a.first > id) {
                --a.first;
            }
            return rem;
        });
    m_nodes[e.snode].edges.erase(ait, m_nodes[e.snode].edges.end());

    // for non-self-loops, remove incident edge from target node and update edge
    // ids
    if (e.tnode != e.snode) {
        assert(e.tnode < m_nodes.size());
        ait = std::remove_if(
            m_nodes[e.tnode].edges.begin(),
            m_nodes[e.tnode].edges.end(),
            [&](Node::adjacency &a) {
                bool rem = a.first == id;
                if (a.first > id) {
                    --a.first;
                }
                return rem;
            });
        m_nodes[e.tnode].edges.erase(ait, m_nodes[e.tnode].edges.end());
    }

    // remove the edge
    m_edges.erase(std::next(m_edges.begin(), id));
}

void ExperienceGraph::clear() {
    m_nodes.clear();
    m_edges.clear();
    m_shift.clear();
}

auto ExperienceGraph::state(node_id id) const -> const StateType & {
    return m_nodes[id].state;
}

auto ExperienceGraph::state(node_id id) -> StateType & {
    return m_nodes[id].state;
}

auto ExperienceGraph::waypoints(edge_id id) const
-> const std::vector<StateType> & {
    return m_edges[id].waypoints;
}

auto ExperienceGraph::waypoints(edge_id id) -> std::vector<StateType> & {
    return m_edges[id].waypoints;
}

} // namespace smpl
} // namespace ims