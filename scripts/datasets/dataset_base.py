import abc
import h5py
from torch.utils.data import Dataset

# project imports
from scripts.datasets.normalizers import DatasetNormalizer
from scripts.utils_torch import *


class DatasetBase(Dataset, abc.ABC):
    """
    Base class for trajectory datasets.
    """

    def __init__(self,
                 data_directory: str,
                 normalizer: str = 'LimitsNormalizer',
                 tensor_args: dict = None,
                 **kwargs):
        """
        Constructor for the DatasetBase class.
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
        return f'[ DatasetBase ] {self.metadata}'

    def __getitem__(self, idx):
        # take only th normalized fields
        return {k: v[idx] for k, v in self.data_fields.items() if k.endswith('_normalized')}

    def normalize(self, x, key):
        return self.normalizer(x, key)  # TODO: if it doesnt work, try self.normalizer.normalize(x, key)

    def denormalize(self, x, key):
        return self.normalizer.denormalize(x, key)
