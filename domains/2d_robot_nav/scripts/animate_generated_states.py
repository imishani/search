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


def visualize(map_index, states):
    """

    :param map_index:  the index of the map to use
    :param states: a numpy array of states, where each row is a state containing its index and x,y coordinates
    :return:
    """
    # load the map (octile map)
    map_file = MAPS[map_index]
    map_data, img, map_type, width, height = load_map(map_file)

    def update(frame_number):
        # if frame_number % 100 == 0:
        #     print(f"Generating frame {frame_number} of {states.shape[0]}")
        print(f"Generating frame {20*frame_number} of {states.shape[0]}")
        state = states[20 * frame_number: 20 * frame_number + 20, :]
        x = state[:, 1]
        y = state[:, 2]
        im.set_array(map_data)
        plt.plot(x, y, 'go', markersize=0.1)
        return im,

    fig = plt.figure(figsize=(width / 100, height / 100))

    im = plt.imshow(map_data, cmap='Greys', vmin=0, vmax=100, animated=True)
    im.axes.get_xaxis().set_visible(False)
    im.axes.get_yaxis().set_visible(False)

    ani = animation.FuncAnimation(fig, update, frames=states.shape[0] // 20, interval=5, blit=True, repeat_delay=1000)
    ani.save("animation_mgs.mp4",
             writer='ffmpeg', dpi=300)
    # save as gif
    # ani.save("animation_mgs.gif", writer='imagemagick', fps=350)
    return


if __name__ == '__main__':
    # load the states
    states = np.loadtxt("../../../cmake-build-release/states_mosaic.txt", dtype=float, delimiter=',').astype(np.int64)

    # visualize the states
    visualize(1, states)
