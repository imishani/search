//
// Created by owner on 28/03/2023.
//

#ifndef SEARCH_ACTION_HPP
#define SEARCH_ACTION_HPP

// standard includes
//#include <utility>

// project includes
#include "common/state.hpp"

namespace ims{
    /// @brief Action type abstract struct
    struct actionType{
        /// @brief Constructor
        actionType() = default;

        /// @brief Destructor
        virtual ~actionType() = default;
    };

    template <typename Environment>
    /// @class Action class for the search problem
    class action{
    public:
        /// @brief Constructor
        /// @param action The action
        explicit action(const Environment& env, const actionType& action=actionType()){
            m_action = action;
            m_env = env;
        }

        /// @brief Destructor
        ~action() = default;

        /// @brief Get Successor
        /// @param curr_state The current state
        /// @param successor The successor state
        /// @return Success bool
        virtual bool getSuccessors(const state& curr_state, state& successor) = 0;

        /// @brief check if the state is valid
        /// @param state_val The values if the state
        /// @return Validity bool
        virtual bool isValid(const stateType& state_val) = 0;

    protected:
        actionType m_action;
        Environment m_env;

    };
}
#endif //SEARCH_ACTION_HPP
