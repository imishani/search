//
// Created by owner on 28/03/2023.
//

#ifndef SEARCH_ACTIONSPACE_HPP
#define SEARCH_ACTIONSPACE_HPP

// standard includes
#include <memory>

// project includes
#include <common/state.hpp>
#include <common/SceneInterface.hpp>

//auto std::hash<ims::state>::operator()(
//        const argument_type& s) const -> result_type
//{
//    size_t seed = 0;
//    boost::hash_combine(seed, boost::hash_range(s.getState().begin(), s.getState().end()));
//    return seed;
//}

namespace ims{
    /// @brief Action type abstract struct. This struct should be inherited by the action type
    /// Make sure to implement the getActions() function
    struct actionType{
        /// @brief Constructor
        actionType() = default;

        /// @brief Destructor
        virtual ~actionType() = default;

        /// @brief Get the possible actions
        /// @return The possible actions
        virtual std::vector<action> getActions() = 0;

        /// @brief Get the resolution of the state space (for discretization)
        /// @param state_des The state discretization
        virtual void Discretization(stateType& state_des) = 0;

        stateType mStateDiscretization;

    };

    /// @class Action class for the search problem
    class actionSpace{
    public:
        /// @brief Constructor
        /// @param env The environment
        /// @param actions_ptr The action definitions
        explicit actionSpace(const std::shared_ptr<SceneInterface>& env,
                             const std::shared_ptr<actionType>& actions_ptr){
            m_actions_ptr = actions_ptr;
            m_env = env;
        }

        /// @brief Destructor
        ~actionSpace() = default;

        /// @brief Get a state by id
        /// @param id The id of the state
        /// @return The state
        /// @note The id is assumed to be valid - meaning that the state exists in m_states
        state* getState(size_t id){
            assert(id < m_states.size() && id >= 0);
            return m_states[id];
        }

        /// @brief Get or create state by state value
        /// @param state_val The state value
        /// @return The state id
        int getOrCreateState(const stateType& state_val){
            // check if the state exists
            auto* curr_state = new state(state_val);
            auto it = m_state_to_id.find(curr_state);
            if(it != m_state_to_id.end()){
                delete curr_state;
                return it->second;
            }
            m_states.push_back(curr_state);
            m_state_to_id[curr_state] = curr_state->getStateId();
            return curr_state->getStateId();
        }


        /// @brief Get Successor
        /// @param curr_state The current state
        /// @param successors The successor state
        /// @return Success bool
        /// @note Beware the you should make sure that the state is discretized! (see actionType::Discretization)
        /// If you are using an implicit graph where the state space is not discrete the define the discretization
        /// based on the tolerance for comparison between states.
        /// @attention You should use getOrCreateState() and getState() when generating the successors!
        virtual bool getSuccessors(state& curr_state,
                                   std::vector<state*>& successors,
                                   std::vector<double>& costs) = 0;

        /// @brief check if the state is valid
        /// @param state_val The values if the state
        /// @return Validity bool
        virtual bool isStateValid(const stateType& state_val) = 0;

        virtual bool isPathValid(const pathType& path) = 0;

        std::vector<state*> m_states;
        using StateKey = state;
        using StateHash = PointerValueHash<StateKey>;
        using StateEqual = PointerValueEqual<StateKey>;
        hash_map<StateKey*, int, StateHash, StateEqual> m_state_to_id;

    protected:

        std::shared_ptr<actionType> m_actions_ptr;
        std::shared_ptr<SceneInterface> m_env;

    };
}
#endif //SEARCH_ACTIONSPACE_HPP
