/*
 * Copyright (C) 2024, Itamar Mishani
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
 * \file   queue_general.hpp
 * \author Itamar Mishani (imishani@cmu.edu)
 * \date   2024-02-02
 */

#pragma once
#include <search/common/queue_general.h>

namespace ims {

///////////////////////////////////////////////////////////////
////////////////////// SimpleQueue below //////////////////////


template <class T, class CompareMain>
T* SimpleQueue<T, CompareMain>::min() const {
    if(open_.empty()){
        throw std::runtime_error("Open list is empty. Nothing to work with.");
    }
    return open_.min();
}

template <class T, class CompareMain>
void SimpleQueue<T, CompareMain>::pop() {
    if(open_.empty()){
        throw std::runtime_error("Open list is empty. Nothing to work with.");
    }
    return open_.pop();
}

template <class T, class CompareMain>
void SimpleQueue<T, CompareMain>::push(T* e) {
    open_.push(e);
}

template <class T, class CompareMain>
void SimpleQueue<T, CompareMain>::erase(T* e) {
    assert(open_.contains(e));
    open_.erase(e);
}

template <class T, class CompareMain>
bool SimpleQueue<T, CompareMain>::empty() const {
    return open_.empty();
}

template <class T, class CompareMain>
void SimpleQueue<T, CompareMain>::clear() {
    open_.clear();
}

template <class T, class CompareMain>
void SimpleQueue<T, CompareMain>::update(T* e) {
    open_.update(e);
}

template <class T, class CompareMain>
size_t SimpleQueue<T, CompareMain>::size() const {
    return open_.size();
}

template <class T, class CompareMain>
void SimpleQueue<T, CompareMain>::updateWithBound(double lower_bound) {
    throw std::runtime_error("Not supposed to be called");
}

template <class T, class CompareMain>
double SimpleQueue<T, CompareMain>::getLowerBound() const {
    if(open_.empty()){
        throw std::runtime_error("Open list is empty. Nothing to work with.");
    }
    return open_.min()->getLowerBound();
}

template <class T, class CompareMain>
bool SimpleQueue<T, CompareMain>::contains(T* e) const {
    return open_.contains(e);
}

///////////////////////////////////////////////////////////////
////////////////////// FocalQueue below ///////////////////////

template <class T, class CompareMain, class CompareFocal>
T* FocalQueue<T, CompareMain, CompareFocal>::min() const {
    return focal_.min();
}

template <class T, class CompareMain, class CompareFocal>
void FocalQueue<T, CompareMain, CompareFocal>::pop() {
    focal_.pop(); // Remove from focal
}

template <class T, class CompareMain, class CompareFocal>
void FocalQueue<T, CompareMain, CompareFocal>::push(T* e) {
    waitlist_.push(e);
}

template <class T, class CompareMain, class CompareFocal>
void FocalQueue<T, CompareMain, CompareFocal>::erase(T* e) {
    if (waitlist_.contains(e))
        waitlist_.erase(e);
    if (focal_.contains(e))
        focal_.erase(e);
}

template <class T, class CompareMain, class CompareFocal>
bool FocalQueue<T, CompareMain, CompareFocal>::empty() const {
    return focal_.empty() && waitlist_.empty();
}

template <class T, class CompareMain, class CompareFocal>
void FocalQueue<T, CompareMain, CompareFocal>::clear() {
    focal_.clear();
    waitlist_.clear();
}

template <class T, class CompareMain, class CompareFocal>
void FocalQueue<T, CompareMain, CompareFocal>::update(T* e){
    bool updated_at_least_one = false;
    if (focal_.contains(e)){
        focal_.update(e);
        updated_at_least_one = true;
    }

    if (waitlist_.contains(e)){
        waitlist_.update(e);
        updated_at_least_one = true;
    }

    if (!updated_at_least_one){
        throw std::runtime_error("Element not found in focal queue");
    }
}

template <class T, class CompareMain, class CompareFocal>
size_t FocalQueue<T, CompareMain, CompareFocal>::size() const {
    return focal_.size() + waitlist_.size();
}

template <class T, class CompareMain, class CompareFocal>
void FocalQueue<T, CompareMain, CompareFocal>::updateWithBound(double lower_bound) {
    // Only allow update if the lower bound is larger than or equal to the previous lower bound. That is, it is monotonically increasing. Otherwise reset the focal queue.
    if (lower_bound < previous_lower_bound_){
        // Take all the nodes (later: that do not satisfy the bound) out from focal and put them in the waitlist.
        std::string error_string = "Lower bound is not monotonically increasing. Previous lower bound: " + std::to_string(previous_lower_bound_) + ", new lower bound: " + std::to_string(lower_bound);
        // std::cout << RED << error_string << RESET << std::endl;
        // throw std::runtime_error(error_string);
    }

    // Populate the focal queue with all the nodes that satisfy the lower bound.
    while (!waitlist_.empty() && waitlist_.min()->getLowerBound() <= lower_bound) {
        focal_.push(waitlist_.min());
        waitlist_.pop();
    }

    previous_lower_bound_ = lower_bound;
}

template <class T, class CompareMain, class CompareFocal>
double FocalQueue<T, CompareMain, CompareFocal>::getLowerBound() const {
    throw std::runtime_error("getLowerBound() not supported for Focal Queue");
}

template <class T, class CompareMain, class CompareFocal>
bool FocalQueue<T, CompareMain, CompareFocal>::contains(T* e) const {
    return focal_.contains(e) || waitlist_.contains(e);
}


///////////////////////////////////////////////////////////////
////////////////////// FocalAndAnchorQueueWrapper below ///////////////////////


template <class T, class CompareMain, class CompareFocal>
T* FocalAndAnchorQueueWrapper<T, CompareMain, CompareFocal>::min() const {
    return focalQ_.min();
}

template <class T, class CompareMain, class CompareFocal>
void FocalAndAnchorQueueWrapper<T, CompareMain, CompareFocal>::pop() {
    T* e = focalQ_.min();
    focalQ_.pop(); // Remove from focal
    anchorQ_.erase(e); // Remove from anchor
}

template <class T, class CompareMain, class CompareFocal>
void FocalAndAnchorQueueWrapper<T, CompareMain, CompareFocal>::push(T* e) {
    anchorQ_.push(e);
    focalQ_.push(e);
}

template <class T, class CompareMain, class CompareFocal>
void FocalAndAnchorQueueWrapper<T, CompareMain, CompareFocal>::erase(T* e) {
    anchorQ_.erase(e);
    focalQ_.erase(e);
}

template <class T, class CompareMain, class CompareFocal>
bool FocalAndAnchorQueueWrapper<T, CompareMain, CompareFocal>::empty() const {
    return focalQ_.empty();
}

template <class T, class CompareMain, class CompareFocal>
void FocalAndAnchorQueueWrapper<T, CompareMain, CompareFocal>::clear() {
    focalQ_.clear();
    anchorQ_.clear();
}

template <class T, class CompareMain, class CompareFocal>
void FocalAndAnchorQueueWrapper<T, CompareMain, CompareFocal>::update(T* e) {
    focalQ_.update(e);
    anchorQ_.update(e);
}
template <class T, class CompareMain, class CompareFocal>
size_t FocalAndAnchorQueueWrapper<T, CompareMain, CompareFocal>::size() const {
    return focalQ_.size();
}

template <class T, class CompareMain, class CompareFocal>
void FocalAndAnchorQueueWrapper<T, CompareMain, CompareFocal>::updateWithBound(double lower_bound) {
    if (lower_bound < anchorQ_.getLowerBound()){
        lower_bound = anchorQ_.getLowerBound();
    }

    focalQ_.updateWithBound(lower_bound);
}

template <class T, class CompareMain, class CompareFocal>
double FocalAndAnchorQueueWrapper<T, CompareMain, CompareFocal>::getLowerBound() const {
    return anchorQ_.getLowerBound();
}

template <class T, class CompareMain, class CompareFocal>
bool FocalAndAnchorQueueWrapper<T, CompareMain, CompareFocal>::contains(T* e) const {
    return focalQ_.contains(e);
}


///////////////////////////////////////////////////////////////
////////////////////// MultiFocalAndAnchorQueueWrapper below ///////////////////////

template <class T, class CompareMain>
template <class CompareFocal>
void MultiFocalAndAnchorQueueWrapper<T, CompareMain>::createNewFocalQueueFromComparator() {
    std::cout << "Creating a new focal queue" << std::endl;
    focalQs_.push_back(new FocalQueue<T, CompareMain, CompareFocal>());
    std::cout << "Focal queue created, now there are " << getNumFocalQueues() << " focal queues" << std::endl;
}

template <class T, class CompareMain>
T* MultiFocalAndAnchorQueueWrapper<T, CompareMain>::min(int focalQ_index) const {
    return focalQs_[focalQ_index]->min();
}

template <class T, class CompareMain>
void MultiFocalAndAnchorQueueWrapper<T, CompareMain>::pop(int focalQ_index) {

    T* e = focalQs_[focalQ_index]->min();
    focalQs_[focalQ_index]->pop(); // Remove from focal
    // Remove from all other focals.
    for (int i = 0; i < focalQs_.size(); i++){
        if (i != focalQ_index){
            focalQs_[i]->erase(e);
        }
    }
    anchorQ_.erase(e); // Remove from anchor
}

template <class T, class CompareMain>
void MultiFocalAndAnchorQueueWrapper<T, CompareMain>::push(T* e) {
    anchorQ_.push(e);
    for (int i = 0; i < focalQs_.size(); i++){
        focalQs_[i]->push(e);
    }
}

template <class T, class CompareMain>
void MultiFocalAndAnchorQueueWrapper<T, CompareMain>::erase(T* e) {
    anchorQ_.erase(e);
    for (int i = 0; i < focalQs_.size(); i++){
        focalQs_[i]->erase(e);
    }
}

template <class T, class CompareMain>
bool MultiFocalAndAnchorQueueWrapper<T, CompareMain>::empty() const {
    bool empty = true;
    for (int i = 0; i < focalQs_.size(); i++){
        empty = empty && focalQs_[i]->empty();
    }
    return empty;
}

template <class T, class CompareMain>
void MultiFocalAndAnchorQueueWrapper<T, CompareMain>::clear() {
    anchorQ_.clear();
    for (int i = 0; i < focalQs_.size(); i++){
        focalQs_[i]->clear();
    }
}

template <class T, class CompareMain>
void MultiFocalAndAnchorQueueWrapper<T, CompareMain>::update(T* e) {
    anchorQ_.update(e);
    for (int i = 0; i < focalQs_.size(); i++){
        focalQs_[i]->update(e);
    }
}
template <class T, class CompareMain>
size_t MultiFocalAndAnchorQueueWrapper<T, CompareMain>::size() const {
    size_t size = 0;
    for (int i = 0; i < focalQs_.size(); i++){
        size += focalQs_[i]->size();
    }
    return size;
}

template <class T, class CompareMain>
void MultiFocalAndAnchorQueueWrapper<T, CompareMain>::updateWithBound(double lower_bound) {
    if (lower_bound < anchorQ_.getLowerBound()){
        lower_bound = anchorQ_.getLowerBound();
    }
    for (int i = 0; i < focalQs_.size(); i++){
        focalQs_[i]->updateWithBound(lower_bound);
    }
}

template <class T, class CompareMain>
double MultiFocalAndAnchorQueueWrapper<T, CompareMain>::getLowerBound() const {
    return anchorQ_.getLowerBound();
}

template <class T, class CompareMain>
bool MultiFocalAndAnchorQueueWrapper<T, CompareMain>::contains(T* e) const {
    bool contains = false;
    for (int i = 0; i < focalQs_.size(); i++){
        if (focalQs_[i]->contains(e)){
            contains = true;
        }
    }
    return contains;
}

///////////////////////////////////////////////////////////////
////////////////////// MultiFocalAndAnchorDTSQueueWrapper below ///////////////////////

template <class T, class CompareMain>
template <class CompareFocal>
void MultiFocalAndAnchorDTSQueueWrapper<T, CompareMain>::createNewFocalQueueFromComparator() {
    std::cout << "Creating a new focal queue" << std::endl;
    focalQs_.push_back(new FocalQueue<T, CompareMain, CompareFocal>());
    dts_alpha_beta_.push_back(std::make_pair(1, 2));
    std::cout << "Focal queue created, now there are " << focalQs_.size() << " focal queues" << std::endl;
}

template <class T, class CompareMain>
void MultiFocalAndAnchorDTSQueueWrapper<T, CompareMain>::sampleFocalIndexDTS() {
    // Sample from the beta distributions of each of the focal queues and choose that with the maximum value.
    std::vector<double> samples;
    for (int i = 0; i < focalQs_.size(); i++){
        std::pair<double, double> alpha_beta = dts_alpha_beta_[i];
        // Define the beta distribution.
        boost::random::beta_distribution<> beta_dist(alpha_beta.first, alpha_beta.second);
        // Create a variate generator.
        boost::variate_generator<boost::random::mt19937&, boost::random::beta_distribution<>> sampler(rng_, beta_dist);
        // Generate a sample.
        double sample = sampler();
        samples.push_back(sample);
    }
    current_focalQ_index_ = std::distance(samples.begin(), std::max_element(samples.begin(), samples.end()));
    std::cout << GREEN << "Sampled focal queue index: " << current_focalQ_index_ << " with alpha, beta = " << dts_alpha_beta_[current_focalQ_index_].first << ", " << dts_alpha_beta_[current_focalQ_index_].second << RESET << std::endl;
}

template <class T, class CompareMain>
void MultiFocalAndAnchorDTSQueueWrapper<T, CompareMain>::giveRewardOrPenalty(double reward) {
    // Update the alpha and beta values of the focal queue.
    std::pair<double, double> alpha_beta = dts_alpha_beta_[current_focalQ_index_];
    double alpha = alpha_beta.first;
    double beta = alpha_beta.second;
    if (reward >= 0){
        std::cout << GREEN << "Rewarding focal queue " << current_focalQ_index_ << " with reward " << reward << RESET << std::endl;
        alpha += reward;
        if (alpha + beta > dts_c_ + 1){
            alpha = dts_c_ + 1 - beta;
        }
    } else {
        std::cout << GREEN << "Penalizing focal queue " << current_focalQ_index_ << " with penalty " << reward << RESET << std::endl;
        beta -= reward;
        if (beta + alpha > dts_c_ + 1){
            beta = dts_c_ + 1 - alpha;
        }
    }
    // Cap at alpha+beta = c.
    if (alpha + beta >  dts_c_){
        alpha = dts_c_ * alpha / (dts_c_ + 1);
        beta = dts_c_ * beta / (dts_c_ + 1);
    }
    dts_alpha_beta_[current_focalQ_index_] = std::make_pair(alpha, beta);
    // Given the new distribution change, sample a new active focal queue index.
    sampleFocalIndexDTS(); 
}

template <class T, class CompareMain>
void MultiFocalAndAnchorDTSQueueWrapper<T, CompareMain>::givePenalty() {
    giveRewardOrPenalty(-1);
}

template <class T, class CompareMain>
void MultiFocalAndAnchorDTSQueueWrapper<T, CompareMain>::giveReward() {
    giveRewardOrPenalty(1);
}

template <class T, class CompareMain>
void MultiFocalAndAnchorDTSQueueWrapper<T, CompareMain>::giveRewardOrPenalty(int focalQ_index, double reward) {
    // Set the current focal queue index to focalQ_index.
    current_focalQ_index_ = focalQ_index;
    // Update the alpha and beta values of the focal queue.
    giveRewardOrPenalty(reward);
}

template <class T, class CompareMain>
T* MultiFocalAndAnchorDTSQueueWrapper<T, CompareMain>::min() const {
    std::cout << YELLOW << "Popping from focal queue " << current_focalQ_index_ << RESET << std::endl;
    return focalQs_[current_focalQ_index_]->min();
}

template <class T, class CompareMain>
void MultiFocalAndAnchorDTSQueueWrapper<T, CompareMain>::pop() {
    std::cout << YELLOW << "Popping [DTS] from focal queue " << current_focalQ_index_ << RESET << std::endl;
    T* e = focalQs_[current_focalQ_index_]->min();
    focalQs_[current_focalQ_index_]->pop(); // Remove from focal
    // Remove from all other focals.
    for (int i = 0; i < focalQs_.size(); i++){
        if (i != current_focalQ_index_){
            focalQs_[i]->erase(e);
        }
    }
    anchorQ_.erase(e); // Remove from anchor
    std::cout << " Now the DTS anchor is of size " << anchorQ_.size() << std::endl;
}


template <class T, class CompareMain>
void MultiFocalAndAnchorDTSQueueWrapper<T, CompareMain>::push(T* e) {
    anchorQ_.push(e);
    for (int i = 0; i < focalQs_.size(); i++){
        focalQs_[i]->push(e);
    }
}

template <class T, class CompareMain>
void MultiFocalAndAnchorDTSQueueWrapper<T, CompareMain>::erase(T* e) {
    anchorQ_.erase(e);
    for (int i = 0; i < focalQs_.size(); i++){
        focalQs_[i]->erase(e);
    }
}

template <class T, class CompareMain>
bool MultiFocalAndAnchorDTSQueueWrapper<T, CompareMain>::empty() const {
    bool empty = true;
    for (int i = 0; i < focalQs_.size(); i++){
        empty = empty && focalQs_[i]->empty();
    }
    return empty;
}

template <class T, class CompareMain>
void MultiFocalAndAnchorDTSQueueWrapper<T, CompareMain>::clear() {
    anchorQ_.clear();
    for (int i = 0; i < focalQs_.size(); i++){
        focalQs_[i]->clear();
    }
}

template <class T, class CompareMain>
void MultiFocalAndAnchorDTSQueueWrapper<T, CompareMain>::update(T* e) {
    anchorQ_.update(e);
    for (int i = 0; i < focalQs_.size(); i++){
        focalQs_[i]->update(e);
    }
}
template <class T, class CompareMain>
size_t MultiFocalAndAnchorDTSQueueWrapper<T, CompareMain>::size() const {
    size_t size = 0;
    for (int i = 0; i < focalQs_.size(); i++){
        size += focalQs_[i]->size();
    }
    return size;
}

template <class T, class CompareMain>
void MultiFocalAndAnchorDTSQueueWrapper<T, CompareMain>::updateWithBound(double lower_bound) {
    if (lower_bound < anchorQ_.getLowerBound()){
        lower_bound = anchorQ_.getLowerBound();
    }
    for (int i = 0; i < focalQs_.size(); i++){
        focalQs_[i]->updateWithBound(lower_bound);
    }
}

template <class T, class CompareMain>
double MultiFocalAndAnchorDTSQueueWrapper<T, CompareMain>::getLowerBound() const {
    return anchorQ_.getLowerBound();
}

template <class T, class CompareMain>
bool MultiFocalAndAnchorDTSQueueWrapper<T, CompareMain>::contains(T* e) const {
    bool contains = false;
    for (int i = 0; i < focalQs_.size(); i++){
        if (focalQs_[i]->contains(e)){
            contains = true;
        }
    }
    return contains;
}

} // Namespace ims