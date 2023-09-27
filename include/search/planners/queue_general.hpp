#include <search/planners/queue_general.h>

namespace ims {

///////////////////////////////////////////////////////////////
////////////////////// SimpleQueue below //////////////////////


template <class T, class CompareMain>
T* SimpleQueue<T, CompareMain>::min() const {
    return m_open.min();
}

template <class T, class CompareMain>
void SimpleQueue<T, CompareMain>::pop() {
    return m_open.pop();
}

template <class T, class CompareMain>
void SimpleQueue<T, CompareMain>::push(T* e) {
    m_open.push(e);
}

template <class T, class CompareMain>
bool SimpleQueue<T, CompareMain>::empty() const {
    return m_open.empty();
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
void SimpleQueue<T, CompareMain>::updateWithNoBound() {
    return;
}

template <class T, class CompareMain>
double SimpleQueue<T, CompareMain>::getLowerBound() const {
    return m_open.min()->getLowerBound();
}

///////////////////////////////////////////////////////////////
////////////////////// FocalQueue below ///////////////////////

template <class T, class CompareMain, class CompareFocal>
T* FocalQueue<T, CompareMain, CompareFocal>::min() const {
    return m_focal.min();
}

template <class T, class CompareMain, class CompareFocal>
void FocalQueue<T, CompareMain, CompareFocal>::pop() {
    if (!m_waitlist.empty()) {
        m_lower_bound = fmax(m_waitlist.min()->getLowerBound(), m_lower_bound);
    }
    m_focal.pop(); // Remove from focal
}

template <class T, class CompareMain, class CompareFocal>
void FocalQueue<T, CompareMain, CompareFocal>::push(T* e) {
    m_waitlist.push(e);
    m_lower_bound = fmax(m_waitlist.min()->getLowerBound(), m_lower_bound);
}

template <class T, class CompareMain, class CompareFocal>
bool FocalQueue<T, CompareMain, CompareFocal>::empty() const {
    return m_focal.empty();
}

template <class T, class CompareMain, class CompareFocal>
size_t FocalQueue<T, CompareMain, CompareFocal>::size() const {
    return m_focal.size();
}

template <class T, class CompareMain, class CompareFocal>
void FocalQueue<T, CompareMain, CompareFocal>::updateWithBound(double lower_bound) {
    while (!m_waitlist.empty() && m_waitlist.min()->getLowerBound() <= lower_bound) {
        m_focal.push(m_waitlist.min());
        m_waitlist.pop();
    }
}

template <class T, class CompareMain, class CompareFocal>
void FocalQueue<T, CompareMain, CompareFocal>::updateWithNoBound() {
    throw std::runtime_error("Not supposed to be called");
}

template <class T, class CompareMain, class CompareFocal>
double FocalQueue<T, CompareMain, CompareFocal>::getLowerBound() const {
    return m_lower_bound;
}

} // Namespace ims