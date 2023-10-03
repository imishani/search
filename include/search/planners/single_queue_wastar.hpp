#pragma once

#include <unordered_set>
#include "single_queue_planner.hpp"

namespace ims {

class SQWAStar : public SingleQueuePlanner {
public:
    struct SQWAStarParams {
        double heuristic_weight;
    };


protected:
    struct WAStarSearchState : public GenericSearchState {
        double f;
        double h;
    };

    struct CompareWAStarSearchState {
        bool operator()(const WAStarSearchState* s1, const WAStarSearchState* s2) const {
            return s1->f < s2->f;
        }
    };

    void populateSearchState(GenericSearchState* state, 
                        GenericSearchState* parent, double cost) override {
        WAStarSearchState* ss = dynamic_cast<WAStarSearchState*>(state);
        StateType robot_state = action_space_ptr_->getRobotState(state->robot_state_id)->state;
        heuristic_->getHeuristic(robot_state, ss->h); // Populates ss->h with heuristic value.
        ss->f = ss->g + params_->heuristic_weight * ss->h;
    }

    bool skipAsAlreadyExpanded(GenericSearchState* state) override {
        return expanded_robot_states_.find(state->robot_state_id) != expanded_robot_states_.end();
    }

    void addToExpanded(GenericSearchState* state) override {
        expanded_robot_states_.insert(state->robot_state_id);
    }

    void createQueue() override {
        main_queue_ = new SimpleQueue<GenericSearchState, CompareWAStarSearchState>();
    }

    SQWAStarParams* params_;
    std::unordered_set<int, GenericSearchState*> expanded_robot_states_;
    BaseHeuristic* heuristic_;
};




} // namespace ims