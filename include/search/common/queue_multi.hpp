#pragma once

#include <search/common/queue_general.h>
/// Objective with this class is to take in a vector of FocalQueues and handle popping/pushing across all of them.

namespace ims { 

template <class T>
class AbstractMultiQueueChooser {
public:
    std::vector<AbstractQueue<T>*> queues_;    

    virtual AbstractQueue<T>* getChosenQueue() const = 0;
    virtual void updateChosenQueue() = 0;
};

template <class T>
class RoundRobinQueueChooser : public AbstractMultiQueueChooser<T> {
private:
    std::vector<AbstractQueue<T>*> queues_;
    AbstractQueue<T>* anchor_queue_;
    size_t current_queue_index_;
public:
    RoundRobinQueueChooser(const MultiQueueWrapper<T>& mqwrapper) : queues_(mqwrapper->getQueues()), 
                            anchor_queue_(mqwrapper->getAnchorQueue()), current_queue_index_(0){};

    virtual AbstractQueue<T>* getChosenQueue() const override {
        AbstractQueue<T>* chosen_queue = queues_[current_queue_index_];
        if (chosen_queue->empty()) {
            return multiqueue_->getAnchorQueue();
        }
        return chosen_queue;
    }

    virtual void updateChosenQueue() override {
        current_queue_index_ = (current_queue_index_ + 1) % queues_.size();
    }
};


template <class T>
class MultiQueueWrapper : public AbstractQueue<T> {
private:
    std::vector<AbstractQueue<T>*> queues_;
    AbstractQueue<T>* anchor_queue_;
    AbstractMultiQueueChooser<T>* m_chooser;

public:

    /// @brief Constructor for MultiQueueWrrapper.
    /// @param queues 
    MultiQueueWrapper(std::vector<AbstractQueue<T>*> queues, AbstractQueue<T>* anchor_queue, AbstractMultiQueueChooser<T>* chooser) : 
                        queues_(queues), anchor_queue_(anchor_queue), m_chooser(chooser) {
    }

    std::vector<AbstractQueue<T>*> getQueues() const {
        return queues_;
    }

    AbstractQueue<T>* getAnchorQueue() const {
        return anchor_queue_;
    }

    virtual T* min() const override {
        return m_chooser->getChosenQueue()->min();
    };

    virtual void pop() override {
        m_chooser->getChosenQueue()->pop();
    }

    virtual void push(T* e) override {
        for (const AbstractQueue<T>* q : queues_) {
            q->push(e);
        }
    };

    virtual void erase(T* e) override {
        for (const AbstractQueue<T>* q : queues_) {
            q->erase(e);
        }
    };

    virtual bool empty() const override {
        return anchor_queue_->empty();
    };

    virtual void clear() override {
        for (const AbstractQueue<T>* q : queues_) {
            q->clear();
        }
        anchor_queue_->clear();
    };

    virtual void update(T* e) override {
        for (const AbstractQueue<T>* q : queues_) {
            if (q->contains(e)) {
                q->update(e);
            }
        }
        if (anchor_queue_->contains(e)) {
            anchor_queue_->update(e);
        }
    }

    /// @brief Returns size of anchor queue.
    /// @return 
    virtual size_t size() const override {
        return anchor_queue_->size();
    }

    /// @brief Checks if element is in any of the queues.
    /// @param e 
    /// @return 
    virtual bool contains(T* e) const override {
        for (const AbstractQueue<T>* q : queues_) {
            if (q->contains(e)) {
                return true;
            }
        }
        return anchor_queue_->contains(e);
    }

    /// @brief Updates queue to add more elements to focal list which satisfy lower bound threshold.
    /// @param lower_bound_threshold is the absolute value, not a suboptimality factor, queue
    /// should then only pop() elements that satisfy this bound.
    virtual void updateWithBound(double lower_bound_threshold) override;

    /// @brief Not supported as FocalQueue does not keep track of this (as the focal queue is sorted by
    /// other values).
    /// @return 
    virtual double getLowerBound() const override;
};

} // Namespace ims