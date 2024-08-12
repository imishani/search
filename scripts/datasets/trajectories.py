import abc
import h5py
from torch.utils.data import Dataset

# project imports
from scripts.datasets.normalizers import DatasetNormalizer
from scripts.utils_torch import *


class TrajDatasetBase(Dataset, abc.ABC):
    """
    Base class for trajectory datasets.
    """

    def __init__(self,
                 data_directory: str,
                 normalizer: str = 'LimitsNormalizer',
                 tensor_args: dict = None,
                 **kwargs):
        """
        Constructor for the TrajDatasetBase class.
        :param data_directory: The directory containing the dataset.
        :param normalizer: Normalization method to use. Default is 'LimitsNormalizer'. Options are 'LimitsNormalizer',
        'Identity', 'GaussianNormalizer', 'SafeLimitsNormalizer', 'FixedLimitsNormalizer'.
        :param tensor_args: Dictionary containing tensor arguments.
        :param kwargs:
        """
        self.data_dir = data_directory
        if tensor_args is None:
            self.tensor_args = {'device': torch.device(get_device()), 'dtype': torch.float32}
        else:
            self.tensor_args = tensor_args
        self.data_fields = {}
        self.metadata = None
        self.load_data()  # TODO: fix the domain name
        self.normalizer = DatasetNormalizer(self.data_fields, normalizer)
        self.normalize_all_data(*self.data_fields.keys())

    def load_data(self):
        """
        Loads the data from the file. We use the h5py library to load the data, so make sure the data is in the HDF5
        :return:
        """
        # assert the data is in the HDF5 format
        assert self.data_dir.endswith('.h5') or self.data_dir.endswith('.hdf5'), \
            f"Data file must be in HDF5 format (ends with .h5 or .hdf5). Got {self.data_dir}"
        with h5py.File(self.data_dir, 'r') as f:
            for key in f.keys():
                if key == 'metadata':
                    self.metadata = {}
                    assert 'size' in f[key].keys(), ("Metadata must contain 'size' key, "
                                                     "which state the size of the data.")
                    for k in f[key].keys():
                        if k == 'domain':
                            self.metadata[k] = {kk: to_torch(np.array(f[key][k][kk]).astype(float),
                                                             **self.tensor_args) if kk != 'name' else f[key][k][kk]
                                                for kk in f[key][k].keys()}
                        else:
                            self.metadata[k] = to_torch(np.array(f[key][k]).astype(float), **self.tensor_args)
                else:
                    self.data_fields[key] = to_torch(np.array(f[key]).astype(float), **self.tensor_args)

    def normalize_all_data(self, *keys):
        for key in keys:
            self.data_fields[f'{key}_normalized'] = self.normalizer(self.data_fields[f'{key}'], key)

    def __len__(self):
        return int(self.metadata['size'])

    def __repr__(self):
        return f'[ TrajDatasetBase ] {self.metadata}'

    def __getitem__(self, idx):
        # take only th normalized fields
        return {k: v[idx] for k, v in self.data_fields.items() if k.endswith('_normalized')}

    def normalize(self, x, key):
        return self.normalizer(x, key)  # TODO: if it doesnt work, try self.normalizer.normalize(x, key)

    def denormalize(self, x, key):
        return self.normalizer.denormalize(x, key)


class TrajectoryDataset(TrajDatasetBase):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.field_key_traj = 'trajs'
        assert self.field_key_traj in self.data_fields.keys(), f"Trajectory key '{self.field_key_traj}' not found."
        assert 'horizon' in self.metadata.keys(), "Metadata must contain 'horizon' key."
        # number of support points as int
        self.n_support_points = int(self.metadata['horizon'].item())
        assert 'state_dim' in self.metadata.keys(), "Metadata must contain 'state_dim' key."
        self.state_dim = int(self.metadata['state_dim'].item())

    def get_hard_conditions(self,
                            traj,
                            horizon=None,
                            normalize=False):
        # start and goal positions
        start_state = traj[0]
        goal_state = traj[-1]

        # if self.include_velocity:
        #     # If velocities are part of the state, then set them to zero at the beggining and end of a trajectory
        #     start_state = torch.cat((start_state_pos, torch.zeros_like(start_state_pos)), dim=-1)
        #     goal_state = torch.cat((goal_state_pos, torch.zeros_like(goal_state_pos)), dim=-1)
        # else:
        #     start_state = start_state_pos
        #     goal_state = goal_state_pos

        if normalize:
            start_state = self.normalizer.normalize(start_state, key=self.field_key_traj)
            goal_state = self.normalizer.normalize(goal_state, key=self.field_key_traj)

        if horizon is None:
            horizon = self.n_support_points
        hard_conds = {
            0: start_state,
            horizon - 1: goal_state
        }
        return hard_conds

    def get_batch_hard_conditions(self,
                                  trajs,
                                  horizon=None,
                                  normalize=False):
        # start and goal positions
        start_states = trajs[:, 0]
        goal_states = trajs[:, -1]
        if normalize:
            start_states = self.normalizer.normalize(start_states, key=self.field_key_traj)
            goal_states = self.normalizer.normalize(goal_states, key=self.field_key_traj)

        if horizon is None:
            horizon = self.n_support_points
        hard_conds = {
            0: start_states,
            horizon - 1: goal_states
        }
        return hard_conds

    def get_single_pt_hard_conditions(self, traj, idx, normalize=False):
        # state_pos = self.robot.get_position(traj[idx])
        state = traj[idx]
        # if self.include_velocity:
        #     state = torch.cat((state_pos, torch.zeros_like(state_pos)), dim=-1)
        # else:
        #     state = state_pos
        if normalize:
            state = self.normalizer.normalize(state, key=self.field_key_traj)
        return {idx: state}


# if __name__ == "__main__":
#     data_dir = "/home/itamar/work/code/algorithms/search/domains/2d_robot_nav/data/den501d/data/paths.h5"
#     dataset = TrajectoryDataset(data_directory=data_dir)
#     data_normalized = dataset[[0, 4]]
#     data_unnorm = dataset.denormalize(data_normalized, dataset.field_key_traj)
#
#     i = 0
