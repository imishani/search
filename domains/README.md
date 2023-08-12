# Domain-Specific Examples
Hello and good morning!

To demonstrate the operation of Search Lib, we have prepared a number of examples for you.

## Example 1: 2D Search
These set of examples demonstrate the use of the 2D search algorithms. To run them, you will need to build the project. From the root directory, run the following commands (replace the planner-name by the specific planner you want to run):

```
./build/run_2d_rob_nav_<planner-name> <map_ix> <num_runs> <scale>
```

where:
- `map_ix` is the index of the map to use (0-4). Each map is a different layout. (see `domains/data`.
- `num_runs` is the number of runs to perform. Each with a different random start and goal state.
- `scale` is a scaling factor applied to the map -- making each cell larger or smaller. (1 < integer.)


## Example 2: 2D Conflict-Based Search (CBS)
From your `build` directory, run the following command:
```
./run_2d_mapf_cbs 6 4 1 && python3 ../src/scripts/visualize_2d_time_paths.py --paths-yaml paths.yaml --fps 2 && open paths.gif
```
OR
```
./run_2d_mapf_cbs 6 4 1 && python3 ../src/scripts/visualize_2d_time_paths.py --paths-yaml paths.yaml --fps 2 && xdg-open paths.gif
```
