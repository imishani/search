import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def load_map(file_path, scaler):
    """
    Reads an octile map from a file and returns the map, image, type, width, height.

    Args:
    - file_path (str): The path to the file containing the octile map.

    Returns:
    - tuple: A tuple containing map, image, type, width, height.
    """
    map_data = []
    map_type = ""
    width = 0
    height = 0

    try:
        with open(file_path, 'r') as file:
            lines = file.readlines()
            if lines[0].strip() == "type octile":
                height = int(lines[1].split()[1])
                width = int(lines[2].split()[1])
                map_type = "octile"

                map_data = [[0 for _ in range(width)] for _ in range(height)]

                for y in range(height):
                    line = lines[4 + y].strip()
                    for x in range(width):
                        c = line[x]
                        map_data[y][x] = 0 if c in ['.', 'G', 'S', 'T'] else 100

        # scale the map
        map_data = np.array(map_data)
        scaled_height = int(height * scaler)
        scaled_width = int(width * scaler)
        map_data = map_data.repeat(scaler, axis=0).repeat(scaler, axis=1)
        map_data = map_data[:scaled_height, :scaled_width]
        height = scaled_height
        width = scaled_width

    except FileNotFoundError:
        print(f"Error: File '{file_path}' not found.")
        return None

    return map_data, map_type, width, height


def load_data(file_path: str):
    """
    Loads the data from the file
    :param file_path:
    :return:
    """
    with open(file_path, "r") as f:
        header = f.readline().strip().split(",")
        # check if the header is correct
        if header[0] != "Problem" or header[1] != "Scale" or header[2] != "PathsNumber":
            raise ValueError("Invalid header")
        # get the map index
        map_index_ = int(header[3])
        # scaler
        scaler = float(header[4])
        # get the number of paths
        num_paths = int(header[5])
        # get the paths
        paths_ = {}
        # get the paths
        for i in range(num_paths):
            # get the header
            header = f.readline().strip().split(",")
            # get the path index
            path_index = int(header[0])
            # get the number of states
            num_states = int(header[1])
            # get the states
            states = []
            for j in range(num_states):
                # get the state
                state = f.readline().strip().split(",")
                # add the state
                states.append([float(state[0]), float(state[1])])
            # add the path
            paths_[path_index] = np.array(states)
    # return the data
    return map_index_, scaler, paths_


def visualize(map_ind, scaler, paths_dict, path_ids_to_visualize, mapfile):
    """
    Visualizes the paths
    :param scaler:
    :param map_ind: the index of the map to use
    :param paths_dict: A dictionary
    :param path_ids_to_visualize: A list of path ids to visualize
    :return:
    """
    # load the map (octile map)
    if mapfile == None:
        map_file = MAPS[map_ind]
    else:
        map_file = mapfile
    map_data, map_type, width, height = load_map(map_file, scaler)

    # create the figure
    fig = plt.figure()
    # create the axis
    ax = plt.axes(xlim=(0, width), ylim=(0, height))
    ax.invert_yaxis()
    # create the image
    plt.imshow(map_data, cmap='Greys', vmin=0, vmax=100)
    # remove the xaixs and yaxis
    ax.axes.get_xaxis().set_visible(False)
    ax.axes.get_yaxis().set_visible(False)
    # if a path id is provided, plot only that path
    if path_ids_to_visualize is not None:
        paths_to_visualize = {}
        for path_id in path_ids_to_visualize:
            paths_to_visualize[path_id] = paths_dict[path_id]
    else:
        paths_to_visualize = paths_dict
    # plot the paths
    for path in paths_to_visualize.values():
        # get the x and y coordinates
        x = path[:, 0]
        y = path[:, 1]
        ax.plot(x, y, 'blue', markersize=1)
        # plot a red circle (without fill) at the start
        ax.plot(x[0], y[0], 'ro', fillstyle='none', markersize=5)
        # plot a green circle (without fill) at the goal
        ax.plot(x[-1], y[-1], 'go', fillstyle='none', markersize=5)

    if args.imagepath is not None:
        plt.savefig(args.imagepath)

    # show the plot
    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    # get the path argument
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--filepath", help="path to the file containing the path solutions")
    parser.add_argument("--path_ids", help="the path ids to visualize (if multiple ids, separate with commas)",
                        default=None)
    parser.add_argument("--mapfile", help="the file of the partial map",
                        default=None)
    parser.add_argument("--imagepath", help="path to the image file the generated plot will be saved to", 
                        default=None)
    args = parser.parse_args()
    # if no argument is provided, raise an error
    if args.filepath is None:
        raise ValueError("No path provided")
    if args.path_ids is not None:
        args.path_ids = [int(x) for x in args.path_ids.split(",")]

    import os
    path_to_this_file = os.path.dirname(os.path.abspath(__file__))

    MAPS = [path_to_this_file + "/../data/hrt201n/hrt201n.map",
            path_to_this_file + "/../data/den501d/den501d.map",
            path_to_this_file + "/../data/den520d/den520d.map",
            path_to_this_file + "/../data/ht_chantry/ht_chantry.map",
            path_to_this_file + "/../data/brc203d/brc203d.map"]

    # load the data
    map_index, scale, paths = load_data(args.filepath)
    # visualize the paths
    visualize(map_index, scale, paths, args.path_ids, args.mapfile)
