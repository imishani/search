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
