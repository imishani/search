import numpy as np
import matplotlib.pyplot as plt

MAPS = ["../data/hrt201n/hrt201n.map",
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


def read_map(map_file):
    with open(map_file, 'r') as f:
        lines = f.readlines()
    lines = [l.strip() for l in lines]
    lines = [l for l in lines if l]
    lines = [l.split() for l in lines]
    lines = [[int(x) for x in l] for l in lines]
    return np.array(lines)


def plot_map(map_file):
    walls = read_map(map_file)
    plt.imshow(walls, cmap='gray')
    plt.show()


def plot_maps(map_files):
    fig, axs = plt.subplots(1, len(map_files), figsize=(15, 5))
    for i, map_file in enumerate(map_files):
        if 'wall' in map_file:
            walls = read_map(map_file)
            axs[i].imshow(walls, cmap='gray')
        else:
            map_data, _, _, _ = load_map(map_file, 1)
            axs[i].imshow(map_data.T, cmap='Greys', vmin=0, vmax=100)
    plt.show()


plot_maps(['wall_map.txt', MAPS[1]])
