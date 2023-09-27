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

///////////////////////////////////////////////////////////////
////////////////////// FocalQueue below ///////////////////////

template <class T, class CompareMain, class CompareFocal>
T* FocalQueue<T, CompareMain, CompareFocal>::min() const {
    return m_focal.min();
}

template <class T, class CompareMain, class CompareFocal>
void FocalQueue<T, CompareMain, CompareFocal>::pop() {
    m_waitlist.erase(m_focal.min()); // Remove from waitlist
    m_focal.pop(); // Remove from focal
}

template <class T, class CompareMain, class CompareFocal>
void FocalQueue<T, CompareMain, CompareFocal>::push(T* e) {
    m_waitlist.push(e);
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
    while (m_waitlist.min()->lower_bound <= lower_bound) {
        m_focal.push(m_waitlist.min());
        m_waitlist.pop();
    }
}

template <class T, class CompareMain, class CompareFocal>
void FocalQueue<T, CompareMain, CompareFocal>::updateWithNoBound() {
    throw std::runtime_error("Not supposed to be called");
}

} // Namespace ims