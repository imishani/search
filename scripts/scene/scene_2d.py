from matplotlib import pyplot as plt
import torch
from .scene_interface_base import SceneInterfaceBase
from scripts.utils_torch.torch_utils import DEFAULT_TENSOR_ARGS, to_numpy


class Scene2D(SceneInterfaceBase):
    def __init__(self, file_path, scaler=1,
                 tensor_args=DEFAULT_TENSOR_ARGS,
                 **kwargs):
        super().__init__(**kwargs)
        self.file_path = file_path
        self.scaler = scaler
        self.tensor_args = tensor_args
        self.map, self.map_type, self.width, self.height = self.load_scene()

    def load_scene(self):
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
            with open(self.file_path, 'r') as file:
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
            map_data = torch.tensor(map_data, **self.tensor_args)
            scaled_height = int(height * self.scaler)
            scaled_width = int(width * self.scaler)
            map_data = map_data.repeat(self.scaler, self.scaler)
            map_data = map_data[:scaled_height, :scaled_width]
            height = scaled_height
            width = scaled_width

        except FileNotFoundError:
            print(f"Error: File '{self.file_path}' not found.")
            return None

        return map_data, map_type, width, height

    def is_collision(self, state, *args, **kwargs):
        x, y = state
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return True
        return self.map[y, x] == 100

    def is_trajectory_collision(self,
                                trajectory: torch.Tensor,
                                *args,
                                **kwargs):
        """
        Check if a trajectory is in collision.
        :param trajectory:
        :param args:
        :param kwargs:
        :return:
        """
        trajectory = trajectory.int()
        if (trajectory < 0).any() or (trajectory[:, 0] >= self.width).any() or (trajectory[:, 1] >= self.height).any():
            return True

        return (self.map[trajectory[:, 1], trajectory[:, 0]] >= 100).any()

    def render(self,
               trajectories: torch.Tensor = None,
               *args, **kwargs):
        """
        Render the scene with the trajectories.
        :param trajectories: The trajectories to render.
        :param args:
        :param kwargs:
        :return:
        """
        # create the figure
        fig = plt.figure()
        # create the axis
        ax = plt.axes(xlim=(0, self.width), ylim=(0, self.height))
        ax.invert_yaxis()
        # create the image
        plt.imshow(to_numpy(self.map), cmap='Greys', vmin=0, vmax=100)
        # remove the xaixs and yaxis
        ax.axes.get_xaxis().set_visible(False)
        ax.axes.get_yaxis().set_visible(False)

        if trajectories is not None:
            for row in range(trajectories.shape[0]):
                traj = to_numpy(trajectories[row])
                # plot the trajectory
                x = traj[:, 0]
                y = traj[:, 1]
                ax.plot(x, y, 'b', markersize=2)
                # plot a red circle (without fill) at the start
                ax.plot(x[0], y[0], 'ro', fillstyle='none', markersize=5)
                # plot a green circle (without fill) at the goal
                ax.plot(x[-1], y[-1], 'go', fillstyle='none', markersize=5)

        # show the plot
        fig.tight_layout()
        return fig
