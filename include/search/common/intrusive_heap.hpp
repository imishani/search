////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Andrew Dornbush

#ifndef SMPL_INTRUSIVE_HEAP_HPP
#define SMPL_INTRUSIVE_HEAP_HPP

#include <search/common/intrusive_heap.h>
#include <cassert>
#include <cstdio>

namespace smpl {

    template <class T, class Compare>
    IntrusiveHeap<T, Compare>::IntrusiveHeap(const compare& comp) :
            data_(1, nullptr),
            comp_(comp)
    {
    }

    template <class T, class Compare>
    template <class InputIt>
    IntrusiveHeap<T, Compare>::IntrusiveHeap(
            const compare& comp,
            InputIt first,
            InputIt last)
            :
            data_(),
            comp_(comp)
    {
        make_heap(first, last);
    }

    template <class T, class Compare>
    template <class InputIt>
    IntrusiveHeap<T, Compare>::IntrusiveHeap(InputIt first, InputIt last) :
            data_(),
            comp_()
    {
        make_heap(first, last);
    }

    template <class T, class Compare>
    IntrusiveHeap<T, Compare>::IntrusiveHeap(IntrusiveHeap&& o) :
            data_(std::move(o.data_)),
            comp_(std::move(o.comp_))
    {
    }

    template <class T, class Compare>
    IntrusiveHeap<T, Compare>&
    IntrusiveHeap<T, Compare>::operator=(IntrusiveHeap&& rhs)
    {
        if (this != &rhs) {
            data_ = std::move(rhs.data_);
            comp_ = std::move(rhs.comp_);
        }
        return *this;
    }

    template <class T, class Compare>
    T* IntrusiveHeap<T, Compare>::min() const
    {
        assert(data_.size() > 1);
        return data_[1];
    }

    template <class T, class Compare>
    typename IntrusiveHeap<T, Compare>::const_iterator
    IntrusiveHeap<T, Compare>::begin() const
    {
        return data_.begin() + 1;
    }

    template <class T, class Compare>
    typename IntrusiveHeap<T, Compare>::const_iterator
    IntrusiveHeap<T, Compare>::end() const
    {
        return data_.end();
    }

    template <class T, class Compare>
    bool IntrusiveHeap<T, Compare>::empty() const
    {
        return data_.size() == 1;
    }

    template <class T, class Compare>
    typename IntrusiveHeap<T, Compare>::size_type
    IntrusiveHeap<T, Compare>::size() const
    {
        return data_.size() - 1;
    }

    template <class T, class Compare>
    typename IntrusiveHeap<T, Compare>::size_type
    IntrusiveHeap<T, Compare>::max_size() const
    {
        return data_.max_size() - 1;
    }

    template <class T, class Compare>
    void IntrusiveHeap<T, Compare>::reserve(size_type new_cap)
    {
        data_.reserve(new_cap + 1);
    }

    template <class T, class Compare>
    void IntrusiveHeap<T, Compare>::clear()
    {
        for (size_t i = 1; i < data_.size(); ++i) {
            data_[i]->heap_index_ = 0;
        }
        data_.resize(1);
    }

    template <class T, class Compare>
    void IntrusiveHeap<T, Compare>::push(T* e)
    {
        assert(e);
        e->heap_index_ = data_.size();
        data_.push_back(e);
        percolate_up(data_.size() - 1);
    }

    template <class T, class Compare>
    void IntrusiveHeap<T, Compare>::pop()
    {
        assert(!empty());
        data_[1]->heap_index_ = 0;
        data_[1] = data_.back();
        // NOTE: no need to set heap index here, as it is not required in
        // percolate_down(); also produces incorrect behavior when popping from a
        // heap of size 1 without the required identity check here
//    data_[1]->heap_index_ = 1;
        data_.pop_back();
        percolate_down(1);
    }

    template <class T, class Compare>
    bool IntrusiveHeap<T, Compare>::contains(T* e)
    {
        assert(e);
        return e->heap_index_ != 0;
    }

    template <class T, class Compare>
    void IntrusiveHeap<T, Compare>::update(T* e)
    {
        assert(e && contains(e));
        percolate_up(e->heap_index_);
        percolate_down(e->heap_index_);
    }

    template <typename T, class Compare>
    void IntrusiveHeap<T, Compare>::increase(T* e)
    {
        assert(e && contains(e));
        percolate_down(e->heap_index_);
    }

    template <class T, class Compare>
    void IntrusiveHeap<T, Compare>::decrease(T* e)
    {
        assert(e && contains(e));
        percolate_up(e->heap_index_);
    }

    template <class T, class Compare>
    void IntrusiveHeap<T, Compare>::erase(T* e)
    {
        assert(e && contains(e));
        size_type pos = e->heap_index_;
        data_[pos] = data_.back();
        data_[pos]->heap_index_ = pos;
        e->heap_index_ = 0;
        data_.pop_back();
        if (pos < data_.size()) {
            update(data_[pos]);
        }
    }

    template <class T, class Compare>
    void IntrusiveHeap<T, Compare>::make()
    {
        for (auto i = (data_.size() - 1) >> 1; i >= 1; --i) {
            percolate_down(i);
        }
    }

    template <class T, class Compare>
    void IntrusiveHeap<T, Compare>::swap(IntrusiveHeap& o)
    {
        if (this != &o) {
            using std::swap;
            swap(data_, o.data_);
            swap(comp_, o.comp_);
        }
    }

    template <class T, class Compare>
    inline
    typename IntrusiveHeap<T, Compare>::size_type
    IntrusiveHeap<T, Compare>::ipow2(size_type exp)
    {
        if (exp == 0) {
            return 1;
        }

        size_type res = ipow2(exp >> 1) * ipow2(exp >> 1);
        if (exp % 2) {
            res *= 2;
        }
        return res;
    }

    template <class T, class Compare>
    inline
    typename IntrusiveHeap<T, Compare>::size_type
    IntrusiveHeap<T, Compare>::ilog2(size_type i)
    {
        std::size_t r = 0;
        while (i >>= 1) {
            ++r;
        }
        return r;
    }

    template <class T, class Compare>
    inline
    bool IntrusiveHeap<T, Compare>::ispow2(size_type val)
    {
        // does not check for val == 0
        return !(val & (val - 1));
    }

    template <class T, class Compare>
    template <class InputIt>
    void IntrusiveHeap<T, Compare>::make_heap(InputIt first, InputIt last)
    {
        auto n = std::distance(first, last);

        data_.clear();
        data_.reserve(n + 1);
        data_.push_back(nullptr);
        data_.insert(data_.end(), first, last);

        for (size_type i = 1; i < data_.size(); ++i) {
            data_[i]->heap_index_ = i;
        }

        for (auto i = n >> 1; i >= 1; --i) {
            percolate_down(i);
        }
    }

    template <class T, class Compare>
    template <class InputIt>
    void IntrusiveHeap<T, Compare>::make_heap(
            InputIt first,
            InputIt last,
            size_type root)
    {
        const auto n = std::distance(first, last);
        printf("make heap from %d elements from %zu\n", n, root);
        print();

        if (n <= 0) {
            return;
        }

        printf(" -> data[%zu] = %p\n", root, *first);
        data_[root] = *first;
        data_[root]->heap_index_ = root;

        if (n == 1) {
            return;
        }

        const size_type left = left_child(root);
        const size_type right = right_child(root);

        auto f = ilog2(n) - 1;
        size_type f2 = ipow2(f);
        size_type l = f2 - 1 + std::min(n - 2 * f2 + 1, f2);
        size_type r = n - 1 - l;

        InputIt new_start = std::next(first);
        InputIt mid = std::next(new_start, l);

        make_heap(new_start, mid, left);
        make_heap(mid, last, right);
        percolate_down(root);
    }

    template <class T, class Compare>
    inline
    typename IntrusiveHeap<T, Compare>::size_type
    IntrusiveHeap<T, Compare>::parent(size_type index) const
    {
        return index >> 1;
    }

    template <class T, class Compare>
    inline
    typename IntrusiveHeap<T, Compare>::size_type
    IntrusiveHeap<T, Compare>::right_child(size_type index) const
    {
        return (index << 1) + 1;
    }

    template <class T, class Compare>
    inline
    typename IntrusiveHeap<T, Compare>::size_type
    IntrusiveHeap<T, Compare>::left_child(size_type index) const
    {
        return index << 1;
    }

    template <class T, class Compare>
    inline
    void IntrusiveHeap<T, Compare>::percolate_down(size_type pivot)
    {
        if (is_external(pivot)) {
            return;
        }

        size_type left = left_child(pivot);
        size_type right = right_child(pivot);

        T* tmp = data_[pivot];
        while (is_internal(left)) {
            size_type s = right;
            if (is_external(right) || comp_(*data_[left], *data_[right])) {
                s = left;
            }

            if (comp_(*data_[s], *tmp)) {
                data_[pivot] = data_[s];
                data_[pivot]->heap_index_ = pivot;
                pivot = s;
            } else {
                break;
            }

            left = left_child(pivot);
            right = right_child(pivot);
        }
        data_[pivot] = tmp;
        data_[pivot]->heap_index_ = pivot;
    }

    template <class T, class Compare>
    inline
    void IntrusiveHeap<T, Compare>::percolate_up(size_type pivot)
    {
        T* tmp = data_[pivot];
        while (pivot != 1) {
            size_type p = parent(pivot);
            if (comp_(*data_[p], *tmp)) {
                break;
            }
            data_[pivot] = data_[p];
            data_[pivot]->heap_index_ = pivot;
            pivot = p;
        }
        data_[pivot] = tmp;
        data_[pivot]->heap_index_ = pivot;
    }

    template <class T, class Compare>
    inline
    bool IntrusiveHeap<T, Compare>::is_internal(size_type index) const
    {
        return index < data_.size();
    }

    template <class T, class Compare>
    inline
    bool IntrusiveHeap<T, Compare>::is_external(size_type index) const
    {
        return index >= data_.size();
    }

    template <class T, class Compare>
    void IntrusiveHeap<T, Compare>::print() const
    {
        printf("[ null, ");
        for (int i = 1; i < data_.size(); ++i) {
            printf(" (%d, %p)", data_[i]->heap_index_, data_[i]);
            if (i == data_.size() - 1) {
                printf(" ");
            } else {
                printf(", ");
            }
        }
        printf("]\n");
    }

    template <class T, class Compare>
    bool IntrusiveHeap<T, Compare>::check_heap(size_type index) const
    {
        auto right = right_child(index);
        if (is_internal(right)) {
            // check the ordering of the parent and the right child
            if (!comp_(*data_[index], *data_[right])) {
                return false;
            }

            // check the tree rooted at the right child
            if (!check_heap(right)) {
                return false;
            }
        }

        auto left = left_child(index);
        if (is_internal(left)) {
            // check the ordering of the parent and the left child
            if (!comp_(*data_[index], *data_[left])) {
                return false;
            }

            // check the tree rooted at the left child
            if (!check_heap(left)) {
                return false;
            }
        }

        return true;
    }

    template <class T, class Compare>
    bool IntrusiveHeap<T, Compare>::check_heap() const
    {
        if (empty()) {
            return true;
        }

        return check_heap(1);
    }

    template <class T, class Compare>
    void swap(IntrusiveHeap<T, Compare>& lhs, IntrusiveHeap<T, Compare>& rhs)
    {
        lhs.swap(rhs);
    }

} // namespace smpl

#endif