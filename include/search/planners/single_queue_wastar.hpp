#pragma once

#include <unordered_set>
#include "single_queue_planner.hpp"

namespace ims {

class SQWAStar : public SingleQueuePlanner {
public:
    struct SQWAStarParams {
        double heuristic_weight;
        BaseHeuristic* heuristic;

        // SQWAStarParams(double heuristic_weight) : heuristic_weight(heuristic_weight) {}
        SQWAStarParams(double heuristic_weight, BaseHeuristic* heuristic) : 
                    heuristic_weight(heuristic_weight), heuristic(heuristic) {}
    };

    explicit SQWAStar(SQWAStarParams* params) : 
        m_params_(params), heuristic_(params->heuristic) {}


protected:
    struct WAStarSearchState : public GenericSearchState {
        double f;
        double h;

        WAStarSearchState(int robot_state_id, int search_id, int parent_id, double g,
                        double f, double h) :
            GenericSearchState(robot_state_id, search_id, parent_id, g),
            f(f), h(h) {}
    };

    struct CompareWAStarSearchState {
        bool operator()(const GenericSearchState& s1, const GenericSearchState& s2) const {
            double f1 = dynamic_cast<const WAStarSearchState&>(s1).f;
            double f2 = dynamic_cast<const WAStarSearchState&>(s2).f;
            return f1 < f2;
        }
    };

    // void populateSearchState(GenericSearchState* state, 
    //                     GenericSearchState* parent, double cost) override {
    //     WAStarSearchState* ss = dynamic_cast<WAStarSearchState*>(state);
    //     if (ss == nullptr) {
    //         throw std::runtime_error("GenericSearchState to WAStarSearchState cast failed");
    //     }
    //     StateType robot_state = action_space_ptr_->getRobotState(state->robot_state_id)->state;
    //     heuristic_->getHeuristic(robot_state, ss->h); // Populates ss->h with heuristic value.
    //     ss->f = ss->g + m_params_->heuristic_weight * ss->h;
    // }

    GenericSearchState* createNewSearchState(int robot_state_id,
                                        GenericSearchState* parent, double cost) override {
        WAStarSearchState* succ;
        if (parent == nullptr) {
            succ = new WAStarSearchState(/*robot_state_id*/ robot_state_id, 
                                    /*search_id=*/ states_.size(), /*parent_id=*/ PARENT_TYPE(UNSET), 
                                    /*g=*/-1, /*f=*/ -1, /*h=*/ -1);
        } else {
            double next_g_val = parent->g + cost;
            double h_val;
            heuristic_->getHeuristic(action_space_ptr_->getRobotState(robot_state_id)->state, h_val);
            double f_val = parent->g + cost + m_params_->heuristic_weight * h_val;
            succ = new WAStarSearchState(/*robot_state_id*/ robot_state_id, 
                                    /*search_id=*/ states_.size(), /*parent_id=*/ parent->search_id, 
                                    /*g=*/ next_g_val, /*f=*/ f_val, /*h=*/ h_val);
        }
        // Populate here, specific per application. Requires casting here.
        states_.push_back(succ);
        return succ;
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

    SQWAStarParams* m_params_;
    std::unordered_set<int> expanded_robot_states_;
    BaseHeuristic* heuristic_;
};




} // namespace ims