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
From your `build` directory, run the following command to run the algorithm. To use a custom map, ( specified using integer identifiers) for specific custom maps:
```
./run_2d_mapf_cbs_or_ecbs -m 6 -a ignore -n 4 -h 1.1
```
Or to run an mapf benchmark map, first download the maps and random scenarios from [Moving AI's MAPF benchmark](https://movingai.com/benchmarks/mapf.html). Create a `datasets` folder in 2d_mapf and unzip these files into `datasets/mapf-map/` and `datasets/scen-random/`. We provide an example already there. Then you should be able to run commands like:
```
./run_2d_mapf_cbs_or_ecbs --map_file_path=domains/2d_mapf/datasets/mapf-map/Boston_0_256.map -a domains/2d_mapf/datasets/scen-random/Boston_0_256.map.scen -n 10 --high_level_suboptimality=1.5
```

You can visualize it by then running
```
python3 ../src/scripts/visualize_2d_time_paths.py --paths-yaml paths.yaml --fps 2
```
and viewing `path.gif` in your build directory.