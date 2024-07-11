import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

MAPS = ["../data/brc202d/brc202d.map",
        "../data/den501d/den501d.map",
        "../data/den520d/den520d.map",
        "../data/ht_chantry/ht_chantry.map",
        "../data/brc203d/brc203d.map"]


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


def visualize(map_index, states):
    """

    :param map_index:  the index of the map to use
    :param states: a numpy array of states, where each row is a state containing its index and x,y coordinates
    :return:
    """
    # load the map (octile map)
    map_file = MAPS[map_index]
    map_data, map_type, width, height = load_map(map_file, 1)

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

    # plot the states
    sc = ax.scatter(states[:, 1], states[:, 2], c='r', s=2)
    # show the plot
    fig.tight_layout()
    plt.show()

    return


if __name__ == '__main__':
    # load the states
    states = np.loadtxt("../../../cmake-build-release/states_mosaic.txt", dtype=float, delimiter=',').astype(np.int64)

    # visualize the states
    visualize(1, states)
