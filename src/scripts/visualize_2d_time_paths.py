""""/*
 * Copyright (C) 2023, Yorai Shaoul
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Carnegie Mellon University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */"""
"""/*!
 * \file   visualize_paths.py
 * \author Yorai Shaoul (yorai@cmu.edu)
 * \date   13/07/2023
*/"""

"""
A script to visualize a set of paths on a map.
The inputs are a json file with the paths (format x y time), and a map image.
Run with:
python3 visualize_paths.py --paths <path_to_json> --map <path_to_map_image>
"""

import sys
import json
import cv2
import imageio
import numpy as np
import argparse

import yaml


def get_map_np(map_path):
    # The map is a .map file. The first line is "type octile", the second line is "height <height>", the third line
    # is "width <width>", the fourth line is "map", and the rest of the lines are the map. The map data looks like
    # this: @@@@@@@@@@@@TTTT@@@@@, with T being traversable and @ being not traversable.

    # Parse the map file.
    map_file = open(map_path, "r")
    map_lines = map_file.readlines()
    map_file.close()
    map_height = int(map_lines[1].split(" ")[1])
    map_width = int(map_lines[2].split(" ")[1])
    # Create a numpy array with the map data.
    map_np = np.zeros((map_height, map_width), dtype=np.uint8)

    map_data = map_lines[4:]
    for i in range(map_height):
        for j in range(map_width):
            if map_data[i][j] == "@":
                map_np[i, j] = 0
            else:
                map_np[i, j] = 255

    return map_np


def get_paths_np(paths):
    # Get the paths as a numpy array. The output shape is (num_paths, num_steps, 3), where the last dimension is (x, y, time).
    # Load paths
    # This object is a list of paths. Each path is a list of states. Each state is a list of (x, y, time).
    # [
    #     [
    #         [x, y, time],
    #         [x, y, time],
    #         ...
    #     ],
    #     [
    #         [x, y, time],
    #         [x, y, time],
    #         ...
    #     ],
    #     ...
    # ]

    # If the paths are of different lengths, then the shorter paths are padded with the last state.
    # Find the longest path.
    max_path_length = 0
    for path in paths:
        if len(path) > max_path_length:
            max_path_length = len(path)

    # Pad the paths.
    for path in paths:
        if len(path) < max_path_length:
            last_state = path[-1]
            path += [last_state] * (max_path_length - len(path))

    # TODO(yoraish): handle the case where paths start at different times.

    # Convert to numpy array.
    paths_np = np.array(paths)

    return paths_np


def create_xy_time_animation(map_np, paths_np, duration=(100 * 1/20), inflate=0):
    # Create an animation of the paths on the map.
    # Create a color for each robot. This is a color and a diluted color.
    colors = [
        [0, 0, 255],
        [0, 255, 0],
        [255, 0, 0],
        [255, 0, 255],
        [0, 255, 255],
        [255, 255, 0],
        [255, 255, 255]]

    tail_colors = [
        [200, 200, 255],
        [200, 255, 200],
        [255, 200, 200],
        [255, 200, 255],
        [200, 255, 255],
        [255, 255, 200],
        [255, 255, 255]]

    # Get the map height and width.
    map_height = map_np.shape[0]
    map_width = map_np.shape[1]

    # Get the number of paths and the number
    num_paths = paths_np.shape[0]
    num_steps = paths_np.shape[1]

    # The gif frames.
    frames = []

    # The current frame, this is a 3-channel image of the map.
    frame_current = np.stack([map_np, map_np, map_np], axis=2)

    # Iterate through the steps.
    for frame_ix in range(num_steps):
        # Add a dot for each path.
        for path_ix in range(num_paths):
            path_color = colors[path_ix % len(colors)]

            x = paths_np[path_ix, frame_ix, 0]
            y = paths_np[path_ix, frame_ix, 1]
            frame_current[y, x] = path_color

            # Mark around the dot.
            for i in range(-inflate, inflate + 1):
                for j in range(-inflate, inflate + 1):
                    if x + i >= 0 and x + i < map_width and y + j >= 0 and y + j < map_height:
                        if np.all(frame_current[y + j, x + i] != np.array([0, 0, 0])):
                            frame_current[y + j, x + i] = path_color

        # Add the frame to the list of frames.
        frames.append(frame_current.copy())

        # Modify the frame to have the current step shown in a different color.
        for path_ix in range(num_paths):
            path_tail_color = tail_colors[path_ix % len(tail_colors)]
            x = paths_np[path_ix, frame_ix, 0]
            y = paths_np[path_ix, frame_ix, 1]
            frame_current[y, x] = path_tail_color

            # Mark around the dot.
            for i in range(-inflate, inflate + 1):
                for j in range(-inflate, inflate + 1):
                    if 0 <= x + i < map_width and 0 <= y + j < map_height:
                        # if frame_current[y + j, x + i] == np.array(path_color):
                        if np.all(frame_current[y + j, x + i] == np.array(path_color)):
                            frame_current[y + j, x + i] = [255, 255, 255]

    # Resize all frames.
    for i in range(len(frames)):
        new_height = 200
        new_width = int(new_height * map_width / map_height)
        frames[i] = cv2.resize(frames[i], (new_width, new_height), interpolation=cv2.INTER_NEAREST)

    # Save the frames as a video using imageio.
    imageio.mimsave("paths.gif", frames, duration=duration)


def main():
    """
    Main function
    """
    parser = argparse.ArgumentParser(description='Visualize paths on a map')
    parser.add_argument('--paths-yaml', type=str, help='path to yaml file with paths')
    parser.add_argument('--fps', type=int, default=20, help='frames per second')
    args = parser.parse_args()

    # Convert the yaml to a dictionary.
    with open(args.paths_yaml, "r") as f:
        paths_dict = yaml.load(f, Loader=yaml.FullLoader)

    # Load paths
    paths = get_paths_np(paths_dict['paths'])

    # Load map
    map_np = get_map_np(paths_dict['map_path'])

    # Draw paths on map.
    print("Creating animation with FPS={}".format(args.fps))
    create_xy_time_animation(map_np, paths, duration=(500/args.fps), inflate=0)


if __name__ == '__main__':
    main()
