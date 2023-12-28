#include <search/common/intrusive_heap_wrapper.h>

namespace smpl {

template <class T, class Compare>
IntrusiveHeapWrapper<T, Compare>::IntrusiveHeapWrapper() : m_pq() {}

template <class T, class Compare>
IntrusiveHeapWrapper<T, Compare>::~IntrusiveHeapWrapper() {
    for (auto& pair : m_map) {
        delete pair.second;
    }
}

template <class T, class Compare>
T* IntrusiveHeapWrapper<T, Compare>::min() const {
    assert(!m_pq.empty());
    return m_pq.min()->element;
}

template <class T, class Compare>
void IntrusiveHeapWrapper<T, Compare>::pop() {
    assert(!m_pq.empty());
    T* ans = m_pq.min()->element;
    m_pq.pop(); // Remove from queue
    // NOTE: IMPORTANT TO pop() FIRST as it uses HeapElementWrapper
    delete m_map[ans]; // Removes HeapElementWrapper
    m_map.erase(ans); // Remove from map
}

template <class T, class Compare>
void IntrusiveHeapWrapper<T, Compare>::push(T* e) {
    if (m_map.find(e) != m_map.end()) {
        throw std::runtime_error("Element already in heap");
    }
    m_map[e] = new HeapElementWrapper(e); // Add to map
    m_pq.push(m_map[e]); // Add to queue
}

template <class T, class Compare>
void IntrusiveHeapWrapper<T, Compare>::erase(T* e) {
    if (m_map.find(e) == m_map.end()) {
        throw std::runtime_error("Element not in heap");
    }
    m_pq.erase(m_map[e]);
    delete m_map[e];
    m_map.erase(e);
}

template <class T, class Compare>
bool IntrusiveHeapWrapper<T, Compare>::contains(T* e) const {
    return m_map.find(e) != m_map.end();
}

template <class T, class Compare>
bool IntrusiveHeapWrapper<T, Compare>::empty() const {
    return m_pq.empty();
}

template <class T, class Compare>
void IntrusiveHeapWrapper<T, Compare>::clear() {
    m_pq.clear();
    for (auto& pair : m_map) {
        delete pair.second;
    }
    m_map.clear();
}

template <class T, class Compare>
void IntrusiveHeapWrapper<T, Compare>::update(T* e) {
    if (m_map.find(e) == m_map.end()) {
        throw std::runtime_error("Element not in heap");
    }
    m_pq.update(m_map[e]);
}

template <class T, class Compare>
size_t IntrusiveHeapWrapper<T, Compare>::size() const {
    return m_pq.size();
}


} // namespace smpl