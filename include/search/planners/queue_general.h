#pragma once
#include <utility>
// #include <search/common/intrusive_heap.h>
#include <search/common/intrusive_heap_wrapper.h>

namespace ims {

template <class T>
class AbstractQueue {
public:
    virtual T* min() const = 0;
    virtual void pop() = 0;
    virtual void push(T* e) = 0;
    virtual void erase(T* e) = 0;
    virtual bool empty() const = 0;
    virtual size_t size() const = 0;

    virtual void updateWithBound(double lower_bound) = 0;
    virtual void updateWithNoBound() = 0;
    virtual double getLowerBound() const = 0;
};


template <class T, class CompareMain>
class SimpleQueue : public AbstractQueue<T> {
private:
    smpl::IntrusiveHeapWrapper<T, CompareMain> m_open;

public:
    SimpleQueue() = default;
    virtual ~SimpleQueue() = default;

    virtual T* min() const override;
    virtual void pop() override;
    virtual void push(T* e) override;
    virtual void erase(T* e) override;
    virtual bool empty() const override;
    virtual size_t size() const override;

    virtual void updateWithBound(double lower_bound) override;
    virtual void updateWithNoBound() override;
    virtual double getLowerBound() const override;
};

/// @brief Require LowerBound function for FocalQueue elements
struct HasLowerBound {
    virtual double getLowerBound() const = 0;
};

template <class T, class CompareMain, class CompareFocal>
class FocalQueue : public AbstractQueue<T> {
private:
    smpl::IntrusiveHeapWrapper<T, CompareMain> m_waitlist;
    smpl::IntrusiveHeapWrapper<T, CompareFocal> m_focal;

public:
    static_assert(std::is_base_of<ims::HasLowerBound, T>::value,
                  "T must inherit from ims::HasLowerBound");
    virtual T* min() const override;
    virtual void pop() override;
    virtual void push(T* e) override;
    virtual void erase(T* e) override;
    virtual bool empty() const override;
    virtual size_t size() const override;

    virtual void updateWithBound(double lower_bound) override;
    virtual void updateWithNoBound() override;
    virtual double getLowerBound() const override;
};


template <class T, class CompareMain, class CompareFocal>
class FocalAndAnchorQueueWrapper : public AbstractQueue<T> {
private:
    FocalQueue<T, CompareMain, CompareFocal> m_focalQ;
    SimpleQueue<T, CompareMain> m_anchorQ;

public:
    virtual T* min() const override;
    virtual void pop() override;
    virtual void push(T* e) override;
    virtual void erase(T* e) override;
    virtual bool empty() const override;
    virtual size_t size() const override;

    virtual void updateWithBound(double lower_bound) override;
    virtual void updateWithNoBound() override;
    virtual double getLowerBound() const override;
};



} // Namespace ims


#include <search/planners/queue_general.hpp>
