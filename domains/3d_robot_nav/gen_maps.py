import numpy as np
import os
import argparse

def generate_3d_map(width, height, depth, obstacle_density, blocks=False):
    """Generate a 3D binary map with obstacles or blocks."""
    map_3d = np.ones((depth, height, width), dtype=int)

    if blocks:
        place_random_blocks(map_3d)
    else:
        map_3d = np.random.random((depth, height, width)) > obstacle_density
        map_3d = map_3d.astype(int)

    return map_3d

def place_random_blocks(map_3d):
    """Randomly place 7 blocks amounting to 20% of each side length."""
    depth, height, width = map_3d.shape
    block_size_w = int(width * 0.3)
    block_size_h = int(height * 0.3)
    block_size_d = int(depth * 0.3)

    num_blocks = 7
    for _ in range(num_blocks):
        start_w = np.random.randint(0, width - block_size_w)
        start_h = np.random.randint(0, height - block_size_h)
        start_d = np.random.randint(0, depth - block_size_d)

        map_3d[start_d:start_d+block_size_d, start_h:start_h+block_size_h, start_w:start_w+block_size_w] = 0

def save_map_file(map_3d, filename):
    """Save the 3D map to a file."""
    depth, height, width = map_3d.shape
    with open(filename, 'w') as f:
        f.write(f"type octile\n")
        f.write(f"height {height}\n")
        f.write(f"width {width}\n")
        f.write(f"depth {depth}\n")
        f.write("map\n")
        for z in range(depth):
            for y in range(height):
                for x in range(width):
                    f.write('.' if map_3d[z, y, x] else '@')
                f.write('\n')
            f.write('\n') 

def generate_start_goal(map_3d, num_pairs):
    """Generate random start and goal positions."""
    depth, height, width = map_3d.shape
    starts = []
    goals = []
    for _ in range(num_pairs):
        while True:
            start = np.random.randint(0, [width, height, depth])
            if map_3d[start[2], start[1], start[0]] == 1:
                break
        while True:
            goal = np.random.randint(0, [width, height, depth])
            if map_3d[goal[2], goal[1], goal[0]] == 1 and not np.array_equal(start, goal):
                break
        starts.append(start)
        goals.append(goal)
    return starts, goals

def save_start_goal_file(starts, goals, filename):
    """Save start and goal positions to a file."""
    with open(filename, 'w') as f:
        for start, goal in zip(starts, goals):
            f.write(f"{start[0]} {start[1]} {start[2]} {goal[0]} {goal[1]} {goal[2]}\n")

def main(args):
    os.makedirs(args.output_dir, exist_ok=True)

    for i in range(args.num_maps):
        map_3d = generate_3d_map(args.width, args.height, args.depth, args.obstacle_density, args.blocks)
        map_filename = os.path.join(args.output_dir, f"map_{i+1}.map")
        save_map_file(map_3d, map_filename)
        print(f"Generated map: {map_filename}")

        starts, goals = generate_start_goal(map_3d, args.num_start_goal_pairs)
        sg_filename = os.path.join(args.output_dir, f"start_goal_{i+1}.txt")
        save_start_goal_file(starts, goals, sg_filename)
        print(f"Generated start/goal file: {sg_filename}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate 3D maps for pathfinding")
    parser.add_argument("--width", type=int, default=50, help="Width of the map")
    parser.add_argument("--height", type=int, default=50, help="Height of the map")
    parser.add_argument("--depth", type=int, default=50, help="Depth of the map")
    parser.add_argument("--obstacle_density", type=float, default=0.3, help="Density of obstacles")
    parser.add_argument("--num_maps", type=int, default=5, help="Number of maps to generate")
    parser.add_argument("--num_start_goal_pairs", type=int, default=10, help="Number of start/goal pairs per map")
    parser.add_argument("--output_dir", type=str, default="data", help="Output directory for generated maps")
    parser.add_argument("--blocks", action='store_true', help="Randomly place 7 blocks amounting to 20% of each side length instead of randomly scattering obstacles")
    
    args = parser.parse_args()
    main(args)
