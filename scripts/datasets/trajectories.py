from scripts.datasets.dataset_base import *


class TrajectoryDataset(DatasetBase):

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
