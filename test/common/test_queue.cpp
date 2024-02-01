/*
 * Copyright (C) 2024, Yorai Shaoul
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Carnegie Mellon University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/*!
 * \file   test_queue.hpp
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   2024-02-01
*/
// General imports.
#include <cassert>
#include <iostream>
#include <limits>
#include <stdexcept>

#include <search/common/queue_general.h>
#include <search/common/queue_general.hpp>
#include <search/planners/planner.hpp>
#include <search/planners/best_first_search.hpp>

// Create a simple SearchState object and a few comparator objects.
/// @brief The search state.
struct SearchState : public ims::SearchState, public ims::SearchStateLowerBoundMixin {

    /// @brief The parent state
    int parent_id = UNSET;
    double g = INF_DOUBLE;
    double f = INF_DOUBLE;
    double a = INF_DOUBLE;
    double b = INF_DOUBLE;
    double c = INF_DOUBLE;
    double d = INF_DOUBLE;
    double getLowerBound() const override {
        assert(f > 0.0);
        return f;
    }

    void print() override {
        std::cout << "SearchState: " << std::endl;
        std::cout << "  parent_id: " << parent_id << std::endl;
        std::cout << "  g: " << g << std::endl;
        std::cout << "  f: " << f << std::endl;
        std::cout << "  a: " << a << std::endl;
        std::cout << "  b: " << b << std::endl;
        std::cout << "  c: " << c << std::endl;
        std::cout << "  d: " << d << std::endl;
    }

};
struct SearchStateCompareF{
    bool operator()(const SearchState& s1, const SearchState& s2) const{
        return s1.f < s2.f;
    }
};

struct SearchStateCompareG{
    bool operator()(const SearchState& s1, const SearchState& s2) const{
        return s1.g < s2.g;
    }
};

struct SearchStateCompareA{
    bool operator()(const SearchState& s1, const SearchState& s2) const{
        return s1.a < s2.a;
    }
};

struct SearchStateCompareB{
    bool operator()(const SearchState& s1, const SearchState& s2) const{
        return s1.b < s2.b;
    }
};

struct SearchStateCompareC{
    bool operator()(const SearchState& s1, const SearchState& s2) const{
        return s1.c < s2.c;
    }
};


int main() {
    // Create a simple queue object.
    std::cout << GREEN << "Creating a simple queue object." << RESET << std::endl;
    ims::MultiFocalAndAnchorQueueWrapper<SearchState, SearchStateCompareF> queue;
    std::cout << CYAN << "Queue created. Let's add some focal comparators." << RESET << std::endl;

    // Add a few focal comparators.
    queue.createNewFocalQueueFromComparator<SearchStateCompareG>();
    queue.createNewFocalQueueFromComparator<SearchStateCompareA>();
    queue.createNewFocalQueueFromComparator<SearchStateCompareB>();
    queue.createNewFocalQueueFromComparator<SearchStateCompareC>();

    // Create a few search states. Let them all have some f value within a bound. Say they all get 8 and the bound is 10.
    // One with a small a value.
    SearchState* s_a = new SearchState(); 
    s_a->a = 1.0;
    s_a->f = 8.0;
    s_a->state_id = 1;

    // One with a small b value.
    SearchState* s_b = new SearchState(); 
    s_b->b = 1.0;
    s_b->f = 8.0;
    s_b->state_id = 2;
    
    // One with a small g value.
    SearchState* s_g = new SearchState(); 
    s_g->g = 1.0;
    s_g->f = 8.0;
    s_g->state_id = 3;
    
    // One with a small f value.
    SearchState* s_f = new SearchState(); 
    s_f->f = 1.0;
    s_f->state_id = 4;

    // One with a small f and also a small c.
    SearchState* s_ac = new SearchState();
    s_ac->a = 0.0;
    s_ac->c = 2.0;
    s_ac->f = 1.0;
    s_ac->state_id = 5;



    // Push the states into the queue.
    queue.push(s_a);
    queue.push(s_b);
    queue.push(s_g);
    queue.push(s_f);
    queue.push(s_ac);

    // Update the focal queues with a bound of 10.
    queue.updateWithBound(10.0);

    // Get the minimum element from each focal queue.
    std::cout << "Minimum element from focal queue 0 (should be 3): " << queue.min(0)->state_id << std::endl;
    assert(queue.min(0)->state_id == 3);
    std::cout << "Minimum element from focal queue 1 (should be 5): " << queue.min(1)->state_id << std::endl;
    assert(queue.min(1)->state_id == 5);
    std::cout << "Minimum element from focal queue 2 (should be 2): " << queue.min(2)->state_id << std::endl;
    assert(queue.min(2)->state_id == 2);
    std::cout << "Minimum element from focal queue 3 (should be 5): " << queue.min(3)->state_id << std::endl;
    assert(queue.min(3)->state_id == 5);

    // Pop the minimum element from queue 3 which should also change the minimum element in queue 1.
    std::cout << CYAN << "Popping the minimum element from queue 3." << RESET << std::endl;
    queue.pop(3);
    std::cout << "Minimum element from focal queue 1 (should be 1): " << queue.min(1)->state_id << std::endl;
    assert(queue.min(1)->state_id == 1);

    // Create a new search state with an even lower a value.
    std::cout << CYAN << "Adding a new search state with an even lower a value, currently outside of focal bounds." << RESET << std::endl;
    SearchState* s_a2 = new SearchState();
    s_a2->a = 0.5;
    s_a2->f = 12.0;
    s_a2->state_id = 6;

    // Push the new state into the queue.
    queue.push(s_a2);

    // Update the focal queues with a bound of 10.
    queue.updateWithBound(10.0);

    // Get the minimum element from each focal queue.
    std::cout << "Minimum element from focal queue 1 (should be 1): " << queue.min(1)->state_id << std::endl;
    assert(queue.min(1)->state_id == 1);

    std::cout << CYAN << "Increase focal bound." << RESET << std::endl;
    queue.updateWithBound(15.0);

    // Get the minimum element from each focal queue.
    std::cout << "Minimum element from focal queue 1 (should be 6): " << queue.min(1)->state_id << std::endl;
    assert(queue.min(1)->state_id == 6);

    std::cout << GREEN << "OK." << RESET << std::endl;
    return 0;
}