import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

MAPS = ["../data/brc202d/brc202d.map",
        "../data/den501d/den501d.map",
        "../data/den520d/den520d.map",
        "../data/ht_chantry/ht_chantry.map",
        "../data/brc203d/brc203d.map"]


def load_map(file_path):
    """
    Reads an octile map from a file and returns the map, image, type, width, height.

    Args:
    - file_path (str): The path to the file containing the octile map.

    Returns:
    - tuple: A tuple containing map, image, type, width, height.
    """
    map_data = []
    img = None
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

    except FileNotFoundError:
        print(f"Error: File '{file_path}' not found.")
        return None

    return map_data, img, map_type, width, height


def visualize(map_index, trajectories, debug):
    """

    :param map_index:  the index of the map to use
    :param trajectories
    :return:
    """
    # load the map (octile map)
    map_file = MAPS[map_index]
    map_data, img, map_type, width, height = load_map(map_file)

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
    fig.tight_layout()
    if debug:
        plt.show(block=False)
    # plot the trajectories
    k = ['r', 'g', 'b', 'c', 'm', 'y', 'k']
    k_id = 0
    for traj in trajectories:
        x, y = traj.reshape(-1, 3)[:, 1], traj.reshape(-1, 3)[:, 2]
        ms = 3
        if len(x) == 1:
            ms = 10
        ax.plot(x, y, k[k_id] + 'o', markersize=ms, alpha=0.2)
        k_id += 1
        if k_id == len(k):
            k_id = 0
    plt.show()

    return


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--map_number", help="The index of the map to use", type=int, default=1)
    parser.add_argument("--debug_mode", help="Debug mode", type=bool, default=True)
    args = parser.parse_args()

    # load the trajs_path
    trajs_path = np.loadtxt("../../../cmake-build-release/trajs_path_mosaic.txt", dtype=int, delimiter=',')
    trajectories = []
    for i in trajs_path:
        # load the trajectory
        trajectory = np.loadtxt(f"../../../cmake-build-release/trajectories/trajectory_{i}.txt", dtype=float, delimiter=',')
        trajectories.append(trajectory)
    # visualize the states
    visualize(args.map_number, trajectories, args.debug_mode)
