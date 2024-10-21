//
// Created by spencer on 10/21/24.
//

#ifndef ACTION_SPACE_RRT_MIXIN_HPP
#define ACTION_SPACE_RRT_MIXIN_HPP


#include "search/common/types.hpp"

namespace ims {

    class ActionSpaceRRTMixin {
    public:

        /// @brief Constructor
        ActionSpaceRRTMixin() = default;

        /// @brief Destructor
        virtual ~ActionSpaceRRTMixin() = default;

        /// @brief Get a random robot state config, will NOT create RobotState for it.
        virtual void sampleState(StateType& state) = 0;

        virtual void getSuccessorInDirection(int cur_state_id, const StateType& target_state, StateType& succ_state, double step_size) = 0;

        virtual bool isSameState(const StateType& state_1, const StateType& state_2) = 0;
    };
}




#endif //ACTION_SPACE_RRT_MIXIN_HPP
