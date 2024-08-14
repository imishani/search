from scripts.datasets.dataset_base import *


class FQIReplayBuffer(DatasetBase):
    """
    Class for the FQI replay buffer.
    """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.field_key_data = 'data'
        assert self.field_key_data in self.data_fields.keys(), f"Policy data key '{self.field_key_data}' not found."
        assert self.metadata is not None, "Metadata key not found."
        assert 'state_dim' in self.metadata.keys(), "Metadata must contain 'state_dim' key."
        self.state_dim = int(self.metadata['state_dim'].item())
        assert 'action_dim' in self.metadata.keys(), "Metadata must contain 'action_dim' key."
        self.action_dim = int(self.metadata['action_dim'].item())
        # self.balance_data(column_dim=2*self.state_dim)

    def balance_data(self, column_dim):
        """
        Balance the data by the column dimension.
        """
        data = self.data_fields[self.field_key_data]
        unique, counts = torch.unique(data[:, column_dim], return_counts=True)
        min_count = torch.min(counts)
        min_ind = torch.argmin(counts)
        data_balanced = torch.empty((0, data.shape[1]), **self.tensor_args)
        for col in unique:
            if col == unique[min_ind]:
                data_balanced = torch.cat((data_balanced, data[data[:, column_dim] == col]), dim=0)
                continue
            indices = torch.where(data[:, column_dim] == col)[0]
            indices = indices[torch.randperm(len(indices))[:min_count]]
            data_balanced = torch.cat((data_balanced, data[indices]), dim=0)
        self.data_fields[self.field_key_data] = data_balanced
        self.metadata['size'] = torch.tensor(len(data_balanced), **self.tensor_args)
        self.normalize_all_data(self.field_key_data)


if __name__ == '__main__':
    data_path = '/home/itamar/work/code/algorithms/search/domains/2d_robot_nav/data/den501d/data/fqi_replay_buffer.h5'
    fqi_replay_buffer = FQIReplayBuffer(data_directory=data_path)
    print(fqi_replay_buffer)
