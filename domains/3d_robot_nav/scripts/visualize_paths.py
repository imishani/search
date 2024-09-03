import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse
import os

def load_map(file_path, scaler):
    """
    Reads a 3D octile map from a file and returns the map data.
    """
    map_data = []
    map_type = ""
    width = height = depth = 0

    try:
        with open(file_path, 'r') as file:
            lines = file.readlines()
            if lines[0].strip() == "type octile":
                height = int(lines[1].split()[1])
                width = int(lines[2].split()[1])
                depth = int(lines[3].split()[1])
                map_type = "octile"

                map_data = [[[0 for _ in range(width)] for _ in range(height)] for _ in range(depth)]

                for z in range(depth):
                    for y in range(height):
                        line = lines[5 + z * (height + 1) + y].strip()
                        for x in range(width):
                            c = line[x]
                            map_data[z][y][x] = 0 if c in ['.', 'G', 'S', 'T'] else 100

        # Scale the map
        map_data = np.array(map_data)
        scaled_depth = int(depth * scaler)
        scaled_height = int(height * scaler)
        scaled_width = int(width * scaler)
        map_data = map_data.repeat(scaler, axis=0).repeat(scaler, axis=1).repeat(scaler, axis=2)
        map_data = map_data[:scaled_depth, :scaled_height, :scaled_width]
        depth, height, width = scaled_depth, scaled_height, scaled_width

    except FileNotFoundError:
        print(f"Error: File '{file_path}' not found.")
        return None

    return map_data, map_type, width, height, depth

def load_data(file_path):
    """
    Loads the path data from the file.
    """
    with open(file_path, "r") as f:
        header = f.readline().strip().split(",")
        if header[0] != "Problem" or header[1] != "Scale" or header[2] != "PathsNumber":
            raise ValueError("Invalid header")
        
        map_index = int(header[3])
        scaler = float(header[4])
        num_paths = int(header[5])
        
        paths = {}
        for _ in range(num_paths):
            header = f.readline().strip().split(",")
            path_index = int(header[0])
            num_states = int(header[1])
            
            states = []
            for _ in range(num_states):
                state = f.readline().strip().split(",")
                states.append([float(state[0]), float(state[1]), float(state[2])])
            
            paths[path_index] = np.array(states)
    
    return map_index, scaler, paths

def visualize_3d(map_ind, scaler, paths_dict, path_ids_to_visualize, map_file, output_file):
    """
    Visualizes the 3D paths with rotated map, cube obstacles, and uniform path color.
    Saves the visualization to a file instead of displaying it.
    """
    map_data, _, width, height, depth = load_map(map_file, scaler)

    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Ensure equal aspect ratio
    ax.set_box_aspect([1, 1, 1])  # Set equal scaling for x, y, and z axes

    # Visualize the map
    x, y, z = np.where(map_data == 100)
    ax.scatter(x, y, z, c='black', alpha=0.3, s=3)

    # Visualize the paths
    if path_ids_to_visualize is not None:
        paths_to_visualize = {pid: paths_dict[pid] for pid in path_ids_to_visualize if pid in paths_dict}
    else:
        paths_to_visualize = paths_dict

    for path_id, path in paths_to_visualize.items():
        x, y, z = path[:, 0], path[:, 1], path[:, 2]
        ax.plot(x, y, z, c='darkblue', linewidth=2)
        ax.plot([x[0]], [y[0]], [z[0]], 'ro', markersize=6)  # Start point
        ax.plot([x[-1]], [y[-1]], [z[-1]], 'go', markersize=6)  # End point

    # Set the viewing angle to reduce distortion
    ax.view_init(elev=30, azim=30)

    ax.set_xlim(0, width)
    ax.set_ylim(0, height)
    ax.set_zlim(0, depth)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.tight_layout()
    
    # Save the figure
    plt.savefig(output_file)
    print(f"Visualization saved to {output_file}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--filepath", help="Path to the file containing the path solutions", required=True)
    parser.add_argument("--mapfile", help="Path to the 3D map file", required=True)
    parser.add_argument("--path_ids", help="The path IDs to visualize (if multiple IDs, separate with commas)", default=None)
    parser.add_argument("--imagepath", help="Path to save the generated plot image", default="3d_map_visualization.png")
    args = parser.parse_args()

    if args.path_ids is not None:
        args.path_ids = [int(x) for x in args.path_ids.split(",")]

    # Load the data
    map_index, scale, paths = load_data(args.filepath)

    # Visualize the paths and save to file
    visualize_3d(map_index, scale, paths, args.path_ids, args.mapfile, args.imagepath)
