//
// Created by spencer on 10/10/24.
//

#ifndef MTRRT_ACTION_SPACE_HPP
#define MTRRT_ACTION_SPACE_HPP

#include "action_space.hpp"
#include "mixin/action_space_mtrrt_mixin.hpp"

namespace ims {

class MTRRTActionSpace : public ActionSpace,
                         public ActionSpaceMTRRTMixin {

};
}


#endif //MTRRT_ACTION_SPACE_HPP
