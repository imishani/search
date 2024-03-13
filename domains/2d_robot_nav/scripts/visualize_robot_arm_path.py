import numpy as np
import matplotlib.pyplot as plt
import matplotlib.transforms as transforms
import imageio.v2 as imageio
import os
import shutil

def load_data(file_path: str):
    """
    Loads the data from the file
    :param file_path:
    :return:
    """

    with open(file_path, "r") as f:
        header = f.readline().strip().split(",")
        # check if the header is correct
        if header[0] != "PathsNumber":
            raise ValueError("Invalid header")

        # get the number of paths
        num_paths = int(header[1])
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
                states.append([int(angle) for angle in state])
            # add the path
            paths_[path_index] = np.array(states)
    # return the data
    return paths_

def make_frame(state, frame_number, link_lengths):
    fig, ax = plt.subplots()
    lim = sum(link_lengths)*1.1
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)

    # hard coded obstacles
    rect = plt.Rectangle((5, 10), 10.0, 10.0, edgecolor='r', facecolor='none')
    ax.add_patch(rect)
    rect = plt.Rectangle((-15, -3), 30.0, 2.0, edgecolor='r', facecolor='none')
    ax.add_patch(rect)

    def rotate_link(rect, pos, deg):
        center_of_rotation = (pos[0], pos[1]+0.5)
        translation_to_origin = transforms.Affine2D().translate(-center_of_rotation[0], -center_of_rotation[1])
        rotation_transform = transforms.Affine2D().rotate_deg(deg)
        combined_transform = translation_to_origin + rotation_transform + translation_to_origin.inverted()
        rect.set_transform(combined_transform + ax.transData)

    pos = [0, -0.5]
    total_angle = 0.0
    for i, angle in enumerate(state):
        link_len = link_lengths[i]

        rect = plt.Rectangle((pos[0], pos[1]), link_len, 1.0, edgecolor='black', facecolor='none')
        total_angle += angle
        rotate_link(rect, pos, total_angle)
        pos[0] += np.cos(total_angle/180*np.pi)*link_len
        pos[1] += np.sin(total_angle/180*np.pi)*link_len
        ax.add_patch(rect)
    ax.set_aspect('equal', adjustable='box')

    filename = f'frames/frame{frame_number}.png'
    plt.savefig(filename)
    plt.close()
    return imageio.imread(filename)

def visualize(paths_dict, path_ids_to_visualize, link_lengths):
    """
    Visualizes the paths
    :param paths_dict: A dictionary
    :param path_ids_to_visualize: A list of path ids to visualize
    :return:
    """
    if path_ids_to_visualize is not None:
        paths_to_visualize = {}
        for path_id in path_ids_to_visualize:
            paths_to_visualize[path_id] = paths_dict[path_id]
    else:
        paths_to_visualize = paths_dict
    # plot the paths
    for i, path in enumerate(paths_to_visualize.values()):
        images = []

        if os.path.exists('frames'):
            # raise ValueError("Frames directory already exists")
            shutil.rmtree('frames')

        os.mkdir('frames')
        for j, state in enumerate(path):
            images.append(make_frame(state, j, link_lengths))

        imageio.mimsave(f'path{i}.gif', images, format='GIF', duration=0.5)
        shutil.rmtree('frames')


if __name__ == "__main__":
    # get the path argument
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--filepath", help="path to the file containing the path solutions")
    parser.add_argument("--path_ids", help="the path ids to visualize (if multiple ids, separate with commas)",
                        default=None)
    args = parser.parse_args()
    # if no argument is provided, raise an error
    if args.filepath is None:
        raise ValueError("No path provided")
    if args.path_ids is not None:
        args.path_ids = [int(x) for x in args.path_ids.split(",")]

    import os
    path_to_this_file = os.path.dirname(os.path.abspath(__file__))

    # (hard coded input)
    link_lengths = [10.0, 10.0, 10.0]

    # load the data
    paths = load_data(args.filepath)
    visualize(paths, args.path_ids, link_lengths)
