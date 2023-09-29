#pragma once
#include <utility>
// #include <search/common/intrusive_heap.h>
#include <search/common/intrusive_heap_wrapper.h>

namespace ims {

/// @brief AbstractQueue which supports basic queue functionality for search
/// @tparam T can be any arbitrary type
template <class T>
class AbstractQueue {
public:
    virtual T* min() const = 0;
    virtual void pop() = 0;
    virtual void push(T* e) = 0;
    virtual void erase(T* e) = 0;
    virtual bool empty() const = 0;
    virtual size_t size() const = 0;

    /// @brief Updates the queue to only consider elements with lower bound
    /// @param lower_bound 
    /// @note This is mainly required for focal queues
    virtual void updateWithBound(double lower_bound) = 0;

    /// @brief Returns lower bound value of the queue
    /// @return 
    /// @note This is supported by some queues and not others
    virtual double getLowerBound() const = 0;
};

/// @brief Basic queue which returns min element based on comparator
/// @tparam T
/// @tparam CompareMain which returns which element is smaller
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

    /// @brief Supported
    /// @return 
    virtual double getLowerBound() const override;
};

/// @brief Require LowerBound function for FocalQueue elements
struct LowerBoundInterface {
    virtual double getLowerBound() const = 0;
};

/// @brief FocalQueue which uses both comparators and returns min element 
//         satisfying lower bound
/// @tparam T element type which must support LowerBoundInterface
/// @tparam CompareMain used for anchor queue
/// @tparam CompareFocal used for focal queue
template <class T, class CompareMain, class CompareFocal>
class FocalQueue : public AbstractQueue<T> {
private:
    smpl::IntrusiveHeapWrapper<T, CompareMain> m_waitlist;
    smpl::IntrusiveHeapWrapper<T, CompareFocal> m_focal;

public:
    static_assert(std::is_base_of<ims::LowerBoundInterface, T>::value,
                  "T must inherit from ims::LowerBoundInterface");
    virtual T* min() const override;
    virtual void pop() override;
    virtual void push(T* e) override;
    virtual void erase(T* e) override;
    virtual bool empty() const override;
    virtual size_t size() const override;

    /// @brief Updates queue to add more elements to focal list
    /// @param lower_bound 
    virtual void updateWithBound(double lower_bound) override;

    /// @brief Not supported as FocalQueue does not keep track of this
    /// @return 
    virtual double getLowerBound() const override;
};

/// @brief Wrapper class that enables focal queue expansion and keeps track of lower bound
/// @tparam T element type which must support LowerBoundInterface
/// @tparam CompareMain used for anchor queue
/// @tparam CompareFocal used for focal queue
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

    /// @brief Updates focal queue to add more elements satisfying lower bound
    /// @param lower_bound 
    virtual void updateWithBound(double lower_bound) override;

    /// @brief Supported now as contains anchor queue
    /// @return 
    virtual double getLowerBound() const override;
};



} // Namespace ims


#include <search/common/queue_general.hpp>
