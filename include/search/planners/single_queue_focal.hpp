#pragma once

#include "single_queue_planner.hpp"
#include "unordered_set"

namespace ims {

using GenericSearchState = SingleQueuePlanner::GenericSearchState;

class FocalSearchSettings : public SingleQueuePlanner::SingleQueueSettings {
public:
    
    struct FocalSearchState : public GenericSearchState {
        double f;
        double h;
        double c; // Num collisions

        FocalSearchState(int robot_state_id, int parent_id, double g,
                        double f, double h, double c) :
            GenericSearchState(robot_state_id, parent_id, g),
            f(f), h(h), c(c) {}
    };

    struct CompareFocalSearchState {
        bool operator()(const GenericSearchState& s1, const GenericSearchState& s2) const {
            const FocalSearchState& fs1 = dynamic_cast<const FocalSearchState&>(s1);
            const FocalSearchState& fs2 = dynamic_cast<const FocalSearchState&>(s2);
            // C++ tuples automatically compare in lexicographic order.
            return std::tie(fs1.c, fs1.f) < std::tie(fs2.c, fs2.f);
        }
    };

    struct CompareMainQueue {
        bool operator()(const GenericSearchState& s1, const GenericSearchState& s2) const {
            const FocalSearchState& fs1 = dynamic_cast<const FocalSearchState&>(s1);
            const FocalSearchState& fs2 = dynamic_cast<const FocalSearchState&>(s2);
            return fs1.f < fs2.f;
        }
    };
    
    explicit FocalSearchSettings(double heuristic_weight, BaseHeuristic* heuristic,
                                const std::shared_ptr<ActionSpace>& action_space_ptr) : 
                heuristic_weight_(heuristic_weight), heuristic_(heuristic),
                action_space_ptr_(action_space_ptr) {}

    bool skipAsAlreadyExpanded(GenericSearchState* state) override {
        return false;
    }

    void addToExpanded(GenericSearchState* state) override {
        // Do nothing.
    }

    AbstractQueue<GenericSearchState>* createQueue() override {
        return new FocalAndAnchorQueueWrapper<GenericSearchState, CompareMainQueue, CompareFocalSearchState>();
    }

    GenericSearchState* createNewSearchState(int robot_state_id,
                                        GenericSearchState* parent, double cost) override {
        FocalSearchState* succ;
        if (parent == nullptr) {
            succ = new FocalSearchState(/*robot_state_id*/ robot_state_id, 
                                    /*parent_id=*/ PARENT_TYPE(UNSET), 
                                    /*g=*/-1, /*f=*/ -1, /*h=*/ -1, /*c=*/ -1);
        } else {
            FocalSearchState* real_parent = dynamic_cast<FocalSearchState*>(parent);
            double next_g_val = real_parent->g + cost;
            double h_val;
            heuristic_->getHeuristic(action_space_ptr_->getRobotState(robot_state_id)->state, h_val);
            double f_val = real_parent->g + cost + heuristic_weight_ * h_val;
            double c_val = real_parent->c + 1; // TODO - get collision count
            succ = new FocalSearchState(/*robot_state_id*/ robot_state_id, 
                                    /*parent_id=*/ parent->search_id, 
                                    /*g=*/ next_g_val, /*f=*/ f_val, /*h=*/ h_val, /*c=*/ c_val);
        }
    }

    double focal_suboptimality_;
    double heuristic_weight_;
    std::unordered_set<int> expanded_robot_states_;
    BaseHeuristic* heuristic_;
    std::shared_ptr<ActionSpace> action_space_ptr_;
};

// class SQFocalSearch : public SingleQueuePlanner {
// public:
//     struct SQFocalParams {
//         double heuristic_weight;
//         BaseHeuristic* heuristic;
//         double focal_suboptimality;

//         SQFocalParams(double heuristic_weight, BaseHeuristic* heuristic, double focal_suboptimality) : 
//                     heuristic_weight(heuristic_weight), heuristic(heuristic), 
//                     focal_suboptimality(focal_suboptimality) {}
//     };

//     explicit SQFocalSearch(SQFocalParams* params) : 
//             m_params_(params), heuristic_(params->heuristic) {
//         createQueue();
//     }

// protected:
//     struct FocalSearchState : public GenericSearchState {
//         double f;
//         double h;
//         double c; // Num collisions

//         FocalSearchState(int robot_state_id, int search_id, int parent_id, double g,
//                         double f, double h, double c) :
//             GenericSearchState(robot_state_id, search_id, parent_id, g),
//             f(f), h(h), c(c) {}
//     };

//     struct CompareMainQueue {
//         bool operator()(const GenericSearchState& s1, const GenericSearchState& s2) const {
//             const FocalSearchState& fs1 = dynamic_cast<const FocalSearchState&>(s1);
//             const FocalSearchState& fs2 = dynamic_cast<const FocalSearchState&>(s2);
//             // C++ tuples automatically compare in lexicographic order.
//             return fs1.f < fs2.f;
//         }
//     };

//     struct CompareFocalSearchState {
//         bool operator()(const GenericSearchState& s1, const GenericSearchState& s2) const {
//             const FocalSearchState& fs1 = dynamic_cast<const FocalSearchState&>(s1);
//             const FocalSearchState& fs2 = dynamic_cast<const FocalSearchState&>(s2);
//             // C++ tuples automatically compare in lexicographic order.
//             return std::tie(fs1.c, fs1.f) < std::tie(fs2.c, fs2.f);
//         }
//     };

//     GenericSearchState* createNewSearchState(int robot_state_id,
//                                         GenericSearchState* parent, double cost) override {
//         FocalSearchState* succ;
//         if (parent == nullptr) {
//             succ = new FocalSearchState(/*robot_state_id*/ robot_state_id, 
//                                     /*search_id=*/ states_.size(), /*parent_id=*/ PARENT_TYPE(UNSET), 
//                                     /*g=*/-1, /*f=*/ -1, /*h=*/ -1, /*c=*/ -1);
//         } else {
//             FocalSearchState* real_parent = dynamic_cast<FocalSearchState*>(parent);
//             double next_g_val = real_parent->g + cost;
//             double h_val;
//             heuristic_->getHeuristic(action_space_ptr_->getRobotState(robot_state_id)->state, h_val);
//             double f_val = real_parent->g + cost + m_params_->heuristic_weight * h_val;
//             double c_val = real_parent->c + 1; // TODO - get collision count
//             succ = new FocalSearchState(/*robot_state_id*/ robot_state_id, 
//                                     /*search_id=*/ states_.size(), /*parent_id=*/ parent->search_id, 
//                                     /*g=*/ next_g_val, /*f=*/ f_val, /*h=*/ h_val, /*c=*/ c_val);
//         }
//         // Populate here, specific per application. Requires casting here.
//         states_.push_back(succ);
//         return succ;
//     }

    
//     bool skipAsAlreadyExpanded(GenericSearchState* state) override {
//         return false;
//     }

//     void addToExpanded(GenericSearchState* state) override {
//         // Do nothing.
//     }

//     void createQueue() override {
//         main_queue_ = new FocalAndAnchorQueueWrapper<GenericSearchState, CompareMainQueue, CompareFocalSearchState>();
//     }

//     SQFocalParams* m_params_;
//     BaseHeuristic* heuristic_;
// };

} // namespace ims