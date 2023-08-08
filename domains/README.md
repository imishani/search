# Domain-Specific Examples
Hello and good morning!

To demonstrate the operation of Search Lib, we have prepared a number of examples for you.

## Example 1: 2D A* Search
This example demonstrates the use of the A* search algorithm in a 2D grid. The example is located in the `domains/run_2d.cpp` file. The executable is located in the `build` directory, and should be run like this:

```
./build/run_2d <map_ix> <num_runs> <scale>

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
