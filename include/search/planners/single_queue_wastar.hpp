#pragma once

#include <unordered_set>
#include "single_queue_planner.hpp"

namespace ims {

using GenericSearchState = SingleQueuePlanner::GenericSearchState;

class WAStarSettings : public SingleQueuePlanner::SingleQueueSettings {
public:

    struct WAStarSearchState : public GenericSearchState {
        double f;
        double h;

        WAStarSearchState(int robot_state_id, int parent_id, double g,
                        double f, double h) :
            GenericSearchState(robot_state_id, parent_id, g),
            f(f), h(h) {}
        ~WAStarSearchState() override = default;

        // double getLowerBound() const override { 
        //     return g + h; 
        // }
    };

    struct CompareWAStarSearchState {
        bool operator()(const GenericSearchState& s1, const GenericSearchState& s2) const {
            double f1 = dynamic_cast<const WAStarSearchState&>(s1).f;
            double f2 = dynamic_cast<const WAStarSearchState&>(s2).f;
            return f1 < f2;
        }
    };
    
    explicit WAStarSettings(double heuristic_weight, BaseHeuristic* heuristic,
                            const std::shared_ptr<ActionSpace>& action_space_ptr) : 
                heuristic_weight_(heuristic_weight), heuristic_(heuristic),
                action_space_ptr_(action_space_ptr) {
        main_queue_ = new SimpleQueue<GenericSearchState, CompareWAStarSearchState>();
    }
    ~WAStarSettings() {
        main_queue_->clear();
        delete main_queue_;
    }

    bool skipAsAlreadyExpanded(GenericSearchState* state) override {
        return expanded_robot_states_.find(state->robot_state_id) != expanded_robot_states_.end();
    }

    void addToExpanded(GenericSearchState* state) override {
        expanded_robot_states_.insert(state->robot_state_id);
    }

    AbstractQueue<GenericSearchState>* getQueue() override {
        return main_queue_;
    }

    void updateQueue() override {
        // Do nothing as main_queue is always up to date.
    }

    GenericSearchState* createNewSearchState(int robot_state_id,
                                        GenericSearchState* parent, double cost) override {
        WAStarSearchState* succ;
        if (parent == nullptr) {
            assert(cost == 0);
            succ = new WAStarSearchState(/*robot_state_id*/ robot_state_id, 
                                    /*parent_id=*/ PARENT_TYPE(UNSET), 
                                    /*g=*/ 0, /*f=*/ 0, /*h=*/ 0);
        } else {
            double next_g_val = parent->g + cost;
            double h_val = 0;
            heuristic_->getHeuristic(action_space_ptr_->getRobotState(robot_state_id)->state, h_val);
            double f_val = parent->g + cost + heuristic_weight_ * h_val;
            succ = new WAStarSearchState(/*robot_state_id*/ robot_state_id, 
                                    /*parent_id=*/ parent->search_id, 
                                    /*g=*/ next_g_val, /*f=*/ f_val, /*h=*/ h_val);
        }
        return succ;
    }

    double heuristic_weight_;
    std::unordered_set<int> expanded_robot_states_;
    BaseHeuristic* heuristic_;
    std::shared_ptr<ActionSpace> action_space_ptr_;
    SimpleQueue<GenericSearchState, CompareWAStarSearchState>* main_queue_;
};

} // namespace ims