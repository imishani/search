#pragma once
#include <search/common/intrusive_heap.h>
#include <unordered_map>

namespace smpl {

template <class T, class Compare>
class IntrusiveHeapWrapper {
private:
    struct HeapElementWrapper : public smpl::HeapElement {
        // This will have heap_index_ internally b/c extending HeapElement
        T* element;
        HeapElementWrapper(T* e) : element(e) {}
    };

    struct CompareWrapper {
        Compare c;
        bool operator()(const HeapElementWrapper& a, const HeapElementWrapper& b) const {
            return c(*(a.element), *(b.element));
        }
    };

    std::unordered_map<T*, HeapElementWrapper*> m_map;
    smpl::IntrusiveHeap<HeapElementWrapper, CompareWrapper> m_pq;

public:
    IntrusiveHeapWrapper();

    virtual T* min() const;
    virtual void pop();
    virtual void push(T* e);
    virtual void erase(T* e);
    virtual bool contains(T* e) const;
    virtual bool empty() const;
    virtual size_t size() const;
};

} // namespace smpl

#include <search/common/intrusive_heap_wrapper.hpp>
