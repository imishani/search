# Search: Multi-Agent Planners
We implement a few multi-agent planners in this package.

## CBS-Based Planners
Some of our planners are implemented in a CBS framework. This means that they are two-level algorithms, with the high level searching over potential multi-agent solutions (a path for each agent) and the low level planner solving single-agent planning instances. The CBS-based planners interface with an action space in order to convert any potential collisions between agents to conflicts. The multi-agent planner resolves the conflicts by converting them to constraints that must be satisfied by its low level planner.

### Architecture
We'll go step by step and illustrate the way our CBS-based planners operate. We'll be looking at `run_2d_cbs.cpp` as an example.

In a driver code, we'll

1. Choose a start and goal configuration for each agent,
    * We set the time (be convention, the last element of the state vector) to 0 for all agents' starts, and to -1 for all agents' goals.
    ```c++
    std::vector<ims::StateType> starts, goals;
    ...
        starts[i].emplace_back(0);
        goals[i].emplace_back(-1);
    ```
2. Create an instance of our planner, in this case a `CBS` planner, and pass it as input a params object,
    ```c++
    ims::CBS planner(params);
    ```
3. And finally, before asking the planner to plan, we initialize it with the action spaces for all agents. Note that the order of the starts, goals, and action spaces should match between all objects such that the `i`th element of each vector corresponds to the `i`th agent.
    ```c++
    planner.initializePlanner(action_spaces, starts, goals);
    MultiAgentPaths paths;
    planner.plan(paths);
    ``` 
Nice. We said nothing about how this works though. So let's look into what is happening within the planner. Following the CBS algorithm, upon each iteration of the algorithm it will choose the least cost high-level (conflict tree (CT)) node from it priority queue and expand it. Some notes here:
1. The ordering of the CT can be done in multiple ways. It could be based on the sum of costs of the solutions (this would be done in CBS), based on the number of conflicts within those solutions (this would be G-CBS), based on a combination with a FOCAL queue (similar to ECBS), etc. 

Once a CT node is requested for expansion, the planner will do the following:
1. Before even expanding it, the planner will first check whether conflicts exist in this node. The `SearchState` object, which holds a CT node, already keeps this information within it, so the check is fast.
    ```c++
    agent_action_space_ptrs_[0]->getPathsConflicts(std::make_shared<MultiAgentPaths>(state->paths), 
                                                    state->unresolved_conflicts, 
                                                    getConflictTypes()   ,
                                                    1, 
                                                    agent_names_);

    if (state->unresolved_conflicts.empty()) {
        ...
        return true;
    }
    ```
    Note that the `unresolved_conflicts` field is not exhaustive. It will often have only a subset of all the conflicts that exist in the solution stored in the CT node. However, if there are _any_ conflicts in the CT node, then the `unresolved_conflicts` field will have at least one conflict in it.

    The planner requests specific types of conflicts to be found in a proposed multi-agent solution by the action space (one of a few, all can do this), which iterates through a proposed solution and looks for illegal multi-agent configurations. If it finds any, it will create corresponding conflicts and store them in the `unresolved_conflicts` field of the CT node.

2. If there are conflicts in the CT solution then things are interesting. The search will now convert the conflicts into constraints. This is handled by the planner. Each planner converts conflicts to constraints in a way that may be unique to it. For example, CBS on 2D grids converts edge conflicts to edge constraints, but CBS for large agents converts point-collision conflicts to a set of vertex constraints.
    ```c++
    // Get a collection of constraint sets. Each constraint set holds the constraints that were created for a single agent from a single conflict. An agent may appear many times, once per conflict.
    std::vector<std::pair<int, std::vector<std::shared_ptr<Constraint>>>> constraints = conflictsToConstraints(state->unresolved_conflicts);
    ```
    Note: the returned object is used to create new CT nodes. Each CT node is created from a single `pair` in the vector. The `int` in the pair is the agent that the constraints are for, and the `vector` holds the constraints collection for that agent. The planner will create a new CT node for each pair in the vector. Additionally, notice that the planner asks the action space for specific conflict types. For example, if the planner is only interested in edge conflicts, then the action space will only return edge conflicts.

3. Given the constraints that were created for the conflicts that were found in the proposed multi-agent solution, the planner creates a new CT node for each of those constraints. In the new node, the constraints from its ancestors are copied.
    ```c++
    SearchState new_state = getOrCreateSearchState(new_state_id);
    ...
    new_state->constraints_collectives = state->constraints_collectives;
    ```
    * The new constraint that prompted the creation of this new CT node is added to the new CT node.
    ```c++
    new_state->constraints_collectives[agent_id].addConstraints(constraint_ptr);
    ```
    * Very importantly too, the constraints are also set in the action space of the newly constrained agent.
    ```c++
    agent_action_space_ptrs_[agent_id]->setConstraintsCollective(std::make_shared<ConstraintsCollective>(new_state->constraints_collectives[agent_id]));
    ```
    * To complete the creation of the new CT node, the planner asks the corresponding action space of the affected agent to replan a path that respects all of its constraints, including the new one.

And that's it. So in order to create a new planner, you need to do the following:
1. In the action space, implement the `getPathsConflicts` method.
2. Create a new planner class that inherits from `CBS`.
3. Implement the `conflictsToConstraints` method.
4. Implement the `getConflictTypes` method.

