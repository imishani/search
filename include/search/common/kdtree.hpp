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
 * \file   action_space.hpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   April 16 2023
 */

#pragma once

#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>


/// \brief A class that represents a node in the KDTree
/// \tparam T The type of the data that the node holds
template <typename T>
class KDNode{
public:
    /// \brief The constructor of the class
    /// \param data The data that the node holds
    /// \param parent The parent of the node
    /// \param left The left child of the node
    /// \param right The right child of the node
    /// \param depth The depth of the node
    KDNode(const T& data, KDNode<T>* parent, KDNode<T>* left, KDNode<T>* right, int depth) :
            data(data), parent(parent), left(left), right(right), depth(depth) {}

    /// \brief The data that the node holds
    T data;

    /// \brief The parent of the node
    KDNode<T>* parent;

    /// \brief The left child of the node
    KDNode<T>* left;

    /// \brief The right child of the node
    KDNode<T>* right;

    /// \brief The depth of the node
    int depth;
};

/// \brief A class that represents a KDTree
/// \tparam T The type of the data that the tree holds
template <typename T>
class KDTree{
public:
    /// \brief The constructor of the class
    /// \param data The data that the tree holds
    /// \param k The number of dimensions of the data
    KDTree(const std::vector<T>& data, int k) : data(data), k(k) {
        root = buildTree(data, 0);
    }

    /// \brief The destructor of the class
    ~KDTree(){
        deleteTree(root);
    }

    /// \brief A function that returns the root of the tree
    /// \return The root of the tree
    KDNode<T>* getRoot(){
        return root;
    }

    /// \brief A function that returns the data that the tree holds
    /// \return The data that the tree holds
    std::vector<T> getData(){
        return data;
    }

    /// \brief A function that returns the number of dimensions of the data
    /// \return The number of dimensions of the data
    int getK(){
        return k;
    }

    /// \brief A function that returns the nearest neighbor of a given point
    /// \param point The given point
    /// \return The nearest neighbor of the given point
    T nearestNeighbor(const T& point){
        return nearestNeighbor(root, point, root->data);
    }

    /// \brief A function that returns the nearest neighbor of a given point
    /// \param node The current node
    /// \param point The given point
    /// \param best The current best neighbor
    /// \return The nearest neighbor of the given point
    T nearestNeighbor(KDNode<T>* node, const T& point, const T& best){
        if (node == nullptr){
            return best;
        }
        if (node->data == point){
            return node->data;
        }
        auto bestDistance = euclidean_distance(best, point);
        auto nodeDistance = euclidean_distance(node->data, point);
        if (nodeDistance < bestDistance){
            return nearestNeighbor(node->left, point, node->data);
        }
        else{
            return nearestNeighbor(node->right, point, best);
        }
    }

    /// \brief A function that returns the distance between two points
    /// \param point1 The first point
    /// \param point2 The second point
    /// \return The distance between the two points
    double distance(const T& point1, const T& point2){
        return acos(cos(point1[0])*cos(point2[0]) + sin(point1[0])*sin(point2[0])*cos(point1[1] - point2[1] + point1[2] - point2[2]));
    }

    double euclidean_distance(const T& point1, const T& point2){
        double sum = 0;
        for (int i = 0; i < k; i++){
            sum += pow(point1[i] - point2[i], 2);
        }
        return sqrt(sum);
    }

private:

    /// \brief The data that the tree holds
    std::vector<T> data;

    /// \brief The number of dimensions of the data
    int k;

    /// \brief The root of the tree
    KDNode<T>* root;

    /// \brief A function that builds the tree
    /// \param data_ The data that the tree holds
    /// \param depth The depth of the current node
    /// \return The root of the tree
    KDNode<T>* buildTree(const std::vector<T>& data_, int depth){
        if (data_.empty()){
            return nullptr;
        }
        int axis = depth % k;
        std::vector<T> sortedData = data_;
        std::sort(sortedData.begin(), sortedData.end(), [axis](const T& point1, const T& point2){
            return point1[axis] < point2[axis];
        });

        int median = sortedData.size() / 2;
        auto* node = new KDNode<T>(sortedData[median], nullptr, nullptr, nullptr, depth);
        std::vector<T> leftData(sortedData.begin(), sortedData.begin() + median);
        std::vector<T> rightData(sortedData.begin() + median + 1, sortedData.end());
        node->left = buildTree(leftData, depth + 1);
        node->right = buildTree(rightData, depth + 1);
        if (node->left != nullptr){
            node->left->parent = node;
        }
        if (node->right != nullptr){
            node->right->parent = node;
        }
        return node;
    }

    /// \brief A function that deletes the tree
    /// \param node The current node
    void deleteTree(KDNode<T>* node){
        if (node == nullptr){
            return;
        }
        deleteTree(node->left);
        deleteTree(node->right);
        delete node;
    }
};



