//
// Created by itamar on 7/6/24.
//

#pragma once

#include <vector>
#include "search/common/types.hpp"
#include "search/action_space/action_space.hpp"


namespace ims {

using ControllerFn = std::vector<ActionSequence>(*)(void* user, const std::shared_ptr<ActionSpace>& action_space_ptr);

enum class ControllerType {
    INVALID,
    GENERATOR,
    CONNECTOR
};

/// @class Controller
/// @brief A general struct for controller. It can be anything, from a simple analytical function to a neural network.
/// @note 1) It is important to define the controller function and the user data before using the controller.
/// @note 2) The controller function should return a vector of action sequences.
/// @example The controller function should be defined as follows:
/// @code inline std::vector<ActionSequence> invalidController(void* user, const std::shared_ptr<ActionSpace>& action_space_ptr) {
/// throw std::runtime_error("Invalid controller function.");
/// }
struct Controller {
    ControllerType type {ControllerType::INVALID}; // The type of the controller
    ControllerFn solver_fn {nullptr}; // The function to solve the state
    void* user_data {nullptr}; // User data for the function
//    TODO: Add constraints.
    std::shared_ptr<ActionSpace> as_ptr {nullptr};
    bool two_p_bvp {false}; // Whether it is a two point boundary value problem that requires the user to provide a start and a goal.
    // It is applicable only in connector controllers at the moment. For generator, default is false.
    [[nodiscard]] std::vector<ActionSequence> solve() const  {
        if (solver_fn == nullptr){
            throw std::runtime_error("Solver function is not set.");
        } else if (as_ptr == nullptr) {
            throw std::runtime_error("Action space is not set.");
        }
        return solver_fn(user_data, as_ptr);
    }

};

}
