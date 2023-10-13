#pragma once
#include <search/common/queue_general.h>

namespace ims {

///////////////////////////////////////////////////////////////
////////////////////// SimpleQueue below //////////////////////


template <class T, class CompareMain>
T* SimpleQueue<T, CompareMain>::min() const {
    assert(!m_open.empty());
    return m_open.min();
}

template <class T, class CompareMain>
void SimpleQueue<T, CompareMain>::pop() {
    assert(!m_open.empty());
    return m_open.pop();
}

template <class T, class CompareMain>
void SimpleQueue<T, CompareMain>::push(T* e) {
    m_open.push(e);
}

template <class T, class CompareMain>
void SimpleQueue<T, CompareMain>::erase(T* e) {
    assert(m_open.contains(e));
    m_open.erase(e);
}

template <class T, class CompareMain>
bool SimpleQueue<T, CompareMain>::empty() const {
    return m_open.empty();
}

template <class T, class CompareMain>
void SimpleQueue<T, CompareMain>::clear() {
    m_open.clear();
}

template <class T, class CompareMain>
void SimpleQueue<T, CompareMain>::update(T* e) {
    m_open.update(e);
}

template <class T, class CompareMain>
size_t SimpleQueue<T, CompareMain>::size() const {
    return m_open.size();
}

template <class T, class CompareMain>
void SimpleQueue<T, CompareMain>::updateWithBound(double lower_bound) {
    throw std::runtime_error("Not supposed to be called");
}

template <class T, class CompareMain>
double SimpleQueue<T, CompareMain>::getLowerBound() const {
    assert(!m_open.empty());
    return m_open.min()->getLowerBound();
}

template <class T, class CompareMain>
bool SimpleQueue<T, CompareMain>::contains(T* e) const {
    return m_open.contains(e);
}

///////////////////////////////////////////////////////////////
////////////////////// FocalQueue below ///////////////////////

template <class T, class CompareMain, class CompareFocal>
T* FocalQueue<T, CompareMain, CompareFocal>::min() const {
    return m_focal.min();
}

template <class T, class CompareMain, class CompareFocal>
void FocalQueue<T, CompareMain, CompareFocal>::pop() {
    m_focal.pop(); // Remove from focal
}

template <class T, class CompareMain, class CompareFocal>
void FocalQueue<T, CompareMain, CompareFocal>::push(T* e) {
    m_waitlist.push(e);
}

template <class T, class CompareMain, class CompareFocal>
void FocalQueue<T, CompareMain, CompareFocal>::erase(T* e) {
    if (m_waitlist.contains(e))
        m_waitlist.erase(e);
    if (m_focal.contains(e))
        m_focal.erase(e);
}

template <class T, class CompareMain, class CompareFocal>
bool FocalQueue<T, CompareMain, CompareFocal>::empty() const {
    return m_focal.empty() && m_waitlist.empty();
}

template <class T, class CompareMain, class CompareFocal>
void FocalQueue<T, CompareMain, CompareFocal>::clear() {
    m_focal.clear();
    m_waitlist.clear();
}

template <class T, class CompareMain, class CompareFocal>
void FocalQueue<T, CompareMain, CompareFocal>::update(T* e){
    bool updated_at_least_one = false;
    if (m_focal.contains(e)){
        m_focal.update(e);
        updated_at_least_one = true;
    }

    if (m_waitlist.contains(e)){
        m_waitlist.update(e);
        updated_at_least_one = true;
    }

    if (!updated_at_least_one){
        throw std::runtime_error("Element not found in focal queue");
    }
}

template <class T, class CompareMain, class CompareFocal>
size_t FocalQueue<T, CompareMain, CompareFocal>::size() const {
    return m_focal.size() + m_waitlist.size();
}

template <class T, class CompareMain, class CompareFocal>
void FocalQueue<T, CompareMain, CompareFocal>::updateWithBound(double lower_bound) {
    while (!m_waitlist.empty() && m_waitlist.min()->getLowerBound() <= lower_bound) {
        m_focal.push(m_waitlist.min());
        m_waitlist.pop();
    }
}

template <class T, class CompareMain, class CompareFocal>
double FocalQueue<T, CompareMain, CompareFocal>::getLowerBound() const {
    throw std::runtime_error("getLowerBound() not supported for Focal Queue");
}

template <class T, class CompareMain, class CompareFocal>
bool FocalQueue<T, CompareMain, CompareFocal>::contains(T* e) const {
    return m_focal.contains(e) || m_waitlist.contains(e);
}


///////////////////////////////////////////////////////////////
////////////////////// FocalAndAnchorQueueWrapper below ///////////////////////


template <class T, class CompareMain, class CompareFocal>
T* FocalAndAnchorQueueWrapper<T, CompareMain, CompareFocal>::min() const {
    return m_focalQ.min();
}

template <class T, class CompareMain, class CompareFocal>
void FocalAndAnchorQueueWrapper<T, CompareMain, CompareFocal>::pop() {
    T* e = m_focalQ.min();
    m_focalQ.pop(); // Remove from focal
    m_anchorQ.erase(e); // Remove from anchor
}

template <class T, class CompareMain, class CompareFocal>
void FocalAndAnchorQueueWrapper<T, CompareMain, CompareFocal>::push(T* e) {
    m_anchorQ.push(e);
    m_focalQ.push(e);
}

template <class T, class CompareMain, class CompareFocal>
void FocalAndAnchorQueueWrapper<T, CompareMain, CompareFocal>::erase(T* e) {
    m_anchorQ.erase(e);
    m_focalQ.erase(e);
}

template <class T, class CompareMain, class CompareFocal>
bool FocalAndAnchorQueueWrapper<T, CompareMain, CompareFocal>::empty() const {
    return m_focalQ.empty();
}

template <class T, class CompareMain, class CompareFocal>
void FocalAndAnchorQueueWrapper<T, CompareMain, CompareFocal>::clear() {
    m_focalQ.clear();
    m_anchorQ.clear();
}

template <class T, class CompareMain, class CompareFocal>
void FocalAndAnchorQueueWrapper<T, CompareMain, CompareFocal>::update(T* e) {
    m_focalQ.update(e);
    m_anchorQ.update(e);
}
template <class T, class CompareMain, class CompareFocal>
size_t FocalAndAnchorQueueWrapper<T, CompareMain, CompareFocal>::size() const {
    return m_focalQ.size();
}

template <class T, class CompareMain, class CompareFocal>
void FocalAndAnchorQueueWrapper<T, CompareMain, CompareFocal>::updateWithBound(double lower_bound) {
    m_focalQ.updateWithBound(lower_bound);
}

template <class T, class CompareMain, class CompareFocal>
double FocalAndAnchorQueueWrapper<T, CompareMain, CompareFocal>::getLowerBound() const {
    return m_anchorQ.getLowerBound();
}

template <class T, class CompareMain, class CompareFocal>
bool FocalAndAnchorQueueWrapper<T, CompareMain, CompareFocal>::contains(T* e) const {
    return m_focalQ.contains(e);
}

} // Namespace ims