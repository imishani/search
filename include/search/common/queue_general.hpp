#pragma once
#include <search/common/queue_general.h>

namespace ims {

///////////////////////////////////////////////////////////////
////////////////////// SimpleQueue below //////////////////////


template <class T, class CompareMain>
T* SimpleQueue<T, CompareMain>::min() const {
    assert(!open_.empty());
    return open_.min();
}

template <class T, class CompareMain>
void SimpleQueue<T, CompareMain>::pop() {
    assert(!open_.empty());
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
    assert(!open_.empty());
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

// template <class T, class CompareMain, class CompareFocal>
// T* FocalAndAnchorQueueWrapper<T, CompareMain, CompareFocal>::minAnchor() const {
//     return anchorQ_.min();
// }

template <class T, class CompareMain, class CompareFocal>
void FocalAndAnchorQueueWrapper<T, CompareMain, CompareFocal>::pop() {
    T* e = focalQ_.min();
    focalQ_.pop(); // Remove from focal
    anchorQ_.erase(e); // Remove from anchor
}

// template <class T, class CompareMain, class CompareFocal>
// void FocalAndAnchorQueueWrapper<T, CompareMain, CompareFocal>::popAnchor() {
//     T* e = anchorQ_.min();
//     anchorQ_.pop(); // Remove from anchor
//     focalQ_.erase(e); // Remove from focal
// }

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
    std::cout << "Focal queue created, now there are " << focalQs_.size() << " focal queues" << std::endl;
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

} // Namespace ims