#pragma once
#include <utility>
// #include <search/common/intrusive_heap.h>
#include <search/common/intrusive_heap_wrapper.h>

namespace ims {

/// @brief Require LowerBound function for elements.
struct SearchStateLowerBoundMixin {
    virtual double getLowerBound() const = 0;
};

/// @brief AbstractQueue which supports basic queue functionality for search.
/// @tparam T must support SearchStateLowerBoundMixin for getLowerBound() functionality.
template <class T>
class AbstractQueue {
public:
    static_assert(std::is_base_of<ims::SearchStateLowerBoundMixin, T>::value,
                  "T must inherit from ims::SearchStateLowerBoundMixin");

    virtual T* min() const = 0;
    virtual void pop() = 0;
    virtual void push(T* e) = 0;
    virtual void erase(T* e) = 0;
    virtual bool empty() const = 0;
    virtual void clear() = 0;
    virtual void update(T* e) = 0;
    virtual size_t size() const = 0;
    virtual bool contains(T* e) const = 0;

    /// @brief Updates the queue to only consider elements that satisfy the lower bound.
    /// @param lower_bound_threshold is the absolute value, not a suboptimality factor, queue
    /// should then only pop() elements that satisfy this bound.
    /// @note This is mainly required for focal queues.
    virtual void updateWithBound(double lower_bound_threshold) = 0;

    /// @brief Returns the lower bound value of the queue (e.g. min->getLowerBound()).
    /// In papers this is typically the min f-value of the anchor queue.
    /// @return 
    /// @note This is supported by some queues and not others.
    virtual double getLowerBound() const = 0;
};

/// @brief Basic queue which returns min element based on comparator.
/// @tparam T element type which must support SearchStateLowerBoundMixin.
/// @tparam CompareMain which returns which element is smaller.
/// @note CompareMain should sort by lower bound!
/// TODO: MAKE SURE THIS IS THE CASE! Might want to have CompareMain just for tie-breaking?
template <class T, class CompareMain>
class SimpleQueue : public AbstractQueue<T> {
private:
    ::smpl::IntrusiveHeapWrapper<T, CompareMain> open_;

public:
    SimpleQueue() = default;
    virtual ~SimpleQueue() = default;

    virtual T* min() const override;
    virtual void pop() override;
    virtual void push(T* e) override;
    virtual void erase(T* e) override;
    virtual bool empty() const override;
    virtual void clear() override;
    virtual void update(T* e) override;
    virtual size_t size() const override;
    virtual bool contains(T* e) const override;

    /// @brief Updates the queue to only consider elements that satisfy the lower bound.
    /// @param lower_bound_threshold 
    /// @note NOT SUPPORTED for SimpleQueue as queue contains all elements regardless of bound.
    virtual void updateWithBound(double lower_bound_threshold) override;

    /// @brief Supported, returns the lower bound of the min element.
    /// @return 
    virtual double getLowerBound() const override;
};

/// @brief FocalQueue which uses both comparators and returns min element 
//         satisfying lower bound.
/// @tparam T element type which must support SearchStateLowerBoundMixin.
/// @tparam CompareMain used for anchor queue.
/// @tparam CompareFocal used for focal queue.
template <class T, class CompareMain, class CompareFocal>
class FocalQueue : public AbstractQueue<T> {
private:
    ::smpl::IntrusiveHeapWrapper<T, CompareMain> waitlist_;
    ::smpl::IntrusiveHeapWrapper<T, CompareFocal> focal_;
    double previous_lower_bound_ = -std::numeric_limits<double>::infinity();

public:
    virtual T* min() const override;
    virtual void pop() override;
    virtual void push(T* e) override;
    virtual void erase(T* e) override;
    virtual bool empty() const override;
    virtual void clear() override;
    virtual void update(T* e) override;
    virtual size_t size() const override;
    virtual bool contains(T* e) const override;

    /// @brief Updates the queue. The focal list will include all the elements from the waitlist that satisfy lower bound threshold.
    /// @param lower_bound_threshold is the absolute value, not a suboptimality factor, queue
    /// should then only pop() elements that satisfy this bound.
    virtual void updateWithBound(double lower_bound_threshold) override;

    /// @brief Not supported as FocalQueue does not keep track of this (as the focal queue is sorted by
    /// other values).
    /// @return 
    virtual double getLowerBound() const override;
};

/// @brief Wrapper class that enables focal queue expansion and keeps track of lower bound.
/// @tparam T element type which must support SearchStateLowerBoundMixin.
/// @tparam CompareMain used for anchor queue.
/// @tparam CompareFocal used for focal queue.
template <class T, class CompareMain, class CompareFocal>
class FocalAndAnchorQueueWrapper : public AbstractQueue<T> {
private:
    FocalQueue<T, CompareMain, CompareFocal> focalQ_;
    SimpleQueue<T, CompareMain> anchorQ_;

public:
    virtual T* min() const override;
    // virtual T* minAnchor() const;
    virtual void pop() override;
    // virtual void popAnchor();
    virtual void push(T* e) override;
    virtual void erase(T* e) override;
    virtual bool empty() const override;
    virtual void clear() override;
    virtual void update(T* e) override;
    virtual size_t size() const override;
    virtual bool contains(T* e) const override;

    /// @brief Updates focal queue to add more elements satisfying lower bound threshold.
    /// @param lower_bound_threshold is the absolute value, not a suboptimality factor, queue
    /// should then only pop() elements that satisfy this bound.
    virtual void updateWithBound(double lower_bound_threshold) override;

    /// @brief Returns the lower bound of all elements in the Queue.
    /// @return 
    /// @note Supported now as we internally have a SimpleQueue which keeps track of the lower bound.
    virtual double getLowerBound() const override;
};


/// @brief Wrapper class that enables multiple focal queues to be added to a single open list.
/// @tparam T element type which must support SearchStateLowerBoundMixin.
/// @tparam CompareMain used for anchor queue. Focal queues are added manually by the user.
template <class T, class CompareMain>
class MultiFocalAndAnchorQueueWrapper : public AbstractQueue<T> {
private:
    
    SimpleQueue<T, CompareMain> anchorQ_;
    std::vector<AbstractQueue<T>*> focalQs_;

public:
    template <class CompareFocal>
    void createNewFocalQueueFromComparator();
    inline int getNumFocalQueues() const {return focalQs_.size();}

    virtual T* min(int focalQ_index) const ;
    virtual void pop(int focalQ_index) ;
    virtual inline T* min() const {throw std::runtime_error("Not supported");}
    virtual inline void pop() {throw std::runtime_error("Not supported");}
    virtual void push(T* e) override;
    virtual void erase(T* e) override;
    virtual bool empty() const override;
    virtual void clear() override;
    virtual void update(T* e) override;
    virtual size_t size() const override;
    virtual bool contains(T* e) const override;

    /// @brief Updates focal queue to add more elements satisfying lower bound threshold.
    /// @param lower_bound_threshold is the absolute value, not a suboptimality factor, queue
    /// should then only pop() elements that satisfy this bound.
    virtual void updateWithBound(double lower_bound_threshold) override;

    /// @brief Returns the lower bound of all elements in the Queue.
    /// @return 
    /// @note Supported now as we internally have a SimpleQueue which keeps track of the lower bound.
    virtual double getLowerBound() const override;
};



} // Namespace ims


#include <search/common/queue_general.hpp>
