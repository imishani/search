import sys

import numpy as np

import matplotlib.pyplot as plt
import argparse

sys.path.append('.')
from obs_generator import *
from dataset import *
from grid import *
sys.path.remove('.')


def make_maps(map_size, n_envs, max_obs, max_obs_size):
    images = []
    for dom in range(n_envs):
        obs = obstacles([map_size[0], map_size[1]], size_max=max_obs_size)
        n_obs = obs.add_n_rand_obs(max_obs)
        border_res = obs.add_border()
        if n_obs == 0 or not border_res:
            continue
        im = obs.get_final()
        images.append(1 - im)
    return images


def plot_map(images):
    for image in images:
        plt.imshow(image, cmap='Greys', vmin=0, vmax=1)
        plt.show()

def save_map(images, map_size):
    # make sure the extension is .map
    # check for number of maps in maps directory, and save as map_{n_maps}.map

    # check in maps
    import os
    # if there is not a directory for the map size, create it
    if not os.path.exists(f'maps/{map_size[0]}x{map_size[1]}'):
        os.makedirs(f'maps/{map_size[0]}x{map_size[1]}')

    n_maps = len(os.listdir(f'maps/{map_size[0]}x{map_size[1]}'))
    for i in range(len(images)):
        filename = f'maps/{map_size[0]}x{map_size[1]}/map_{n_maps + i}.map'
        np.savetxt(filename, images[i], fmt='%d')

def load_map(filename):
    # make sure the extension is .map
    assert filename[-4:] == '.map', "Filename must have .map extension"
    return np.loadtxt(filename)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Make maps for gridworld navigation.')
    parser.add_argument('--map_size', type=int, nargs=2, default=[28, 28], help='Size of the map.')
    parser.add_argument('--n_envs', type=int, default=1, help='Number of environments to make.')
    parser.add_argument('--max_obs', type=int, default=50, help='Maximum number of obstacles to add.')
    parser.add_argument('--max_obs_size', type=int, default=4, help='Maximum size of obstacles to add.')
    parser.add_argument('--plot', action='store_true', help='Plot the map.')
    parser.add_argument('--save', action='store_true', help='Save the map to a file.')
    parser.add_argument('--load', type=str, help='Load the map from a file.')
    args = parser.parse_args()

    if args.load:
        images = [load_map(args.load)]
    else:
        images = make_maps(args.map_size, args.n_envs, args.max_obs, args.max_obs_size)

    if args.plot:
        plot_map(images)

    if args.save:
        save_map(images, args.map_size)