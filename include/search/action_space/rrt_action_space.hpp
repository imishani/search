//
// Created by spencer on 10/21/24.
//

#ifndef RRT_ACTION_SPACE_HPP
#define RRT_ACTION_SPACE_HPP

#include "action_space.hpp"
#include "mixin/action_space_rrt_mixin.hpp"


namespace ims {

    class RRTActionSpace : public ActionSpace,
                           public ActionSpaceRRTMixin {

    };
}


#endif //RRT_ACTION_SPACE_HPP
