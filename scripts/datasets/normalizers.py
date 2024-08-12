import einops
import torch
from abc import ABC, abstractmethod


# ----------------------------------------------------------------------------- #
# --------------------------- multi-field normalizer -------------------------- #
# ----------------------------------------------------------------------------- #
class DatasetNormalizer:

    def __init__(self, dataset, normalizer):
        dataset = flatten(dataset)

        if isinstance(normalizer, str):
            normalizer = eval(normalizer)

        self.normalizers = {}
        for key, val in dataset.items():
            try:
                self.normalizers[key] = normalizer(val)
            except KeyError or ValueError:
                print(f'[ utils/normalization ] Skipping {key} | {normalizer}')

    def __repr__(self):
        string = ''
        for key, normalizer in self.normalizers.items():
            string += f'{key}: {normalizer}]\n'
        return string

    def __call__(self, *args, **kwargs):
        return self.normalize(*args, **kwargs)

    def normalize(self, x, key):
        return self.normalizers[key].normalize(x)

    def denormalize(self, x, key):
        return self.normalizers[key].denormalize(x)

    def get_field_normalizers(self):
        return self.normalizers


def flatten(dataset):
    """
        flattens dataset of { key: [ ... x dim ] }
            to { key : [ (...) x dim ]}
    """
    flattened = {}
    for key, xs in dataset.items():
        xs_new = xs
        if xs.ndim == 2:
            # environments (e d)
            pass
        elif xs.ndim == 3:
            # trajectories in fixed environments
            xs_new = einops.rearrange(xs, 'b h d -> (b h) d')
        elif xs.ndim == 4:
            # trajectories in variable environments
            xs_new = einops.rearrange(xs, 'e b h d -> (e b h) d')
        else:
            raise NotImplementedError
        flattened[key] = xs_new
    return flattened


# --------------------------------------------------------------------------- #
# ------------------------ Single-field normalizers ------------------------- #
# --------------------------------------------------------------------------- #
class Normalizer(ABC):
    """
        parent class, subclass by defining the `normalize` and `denormalize` methods
    """

    #
    # def __init__(self, X):
    #     self.X = X
    #     self.mins = X.min(dim=0).values
    #     self.maxs = X.max(dim=0).values

    @abstractmethod
    def __repr__(self):
        # return (
        #     f'''[ Normalizer ] dim: {self.mins.size}\n    -: '''
        #     f'''{torch.round(self.mins, decimals=2)}\n    +: {torch.round(self.maxs, decimals=2)}\n'''
        # )
        raise NotImplementedError()

    def __call__(self, x):
        return self.normalize(x)

    @abstractmethod
    def normalize(self, *args, **kwargs):
        raise NotImplementedError()

    @abstractmethod
    def denormalize(self, *args, **kwargs):
        raise NotImplementedError()


class Identity(Normalizer):

    def __repr__(self):
        return f'[ Normalizer ] Identity'

    def normalize(self, x):
        return x

    def denormalize(self, x):
        return x


class GaussianNormalizer(Normalizer):
    """
        normalizes to zero mean and unit variance
    """

    def __init__(self, data):
        self.data = data
        self.means = self.data.mean(dim=0)
        self.stds = self.data.std(dim=0)
        self.z = 1

    def __repr__(self):
        return (
            f'[ Normalizer ] Gaussian\n'
            f'    -: {torch.round(self.means, decimals=2)}\n'
            f'    +: {torch.round(self.stds, decimals=2)}\n'
        )

    def normalize(self, x):
        return (x - self.means) / self.stds

    def denormalize(self, x):
        return x * self.stds + self.means


class LimitsNormalizer(Normalizer):
    """
        maps [ xmin, xmax ] to [ -1, 1 ]
    """

    def __init__(self, data, *args, **kwargs):
        self.data = data
        self.mins = self.data.min(dim=0).values
        self.maxs = self.data.max(dim=0).values

    def __repr__(self):
        return (
            f'[ Normalizer ] Limits\n'
            f'    -: {torch.round(self.mins, decimals=2)}\n'
            f'    +: {torch.round(self.maxs, decimals=2)}\n'
        )

    def normalize(self, x):
        # [ 0, 1 ]
        x = (x - self.mins) / (self.maxs - self.mins)
        # [ -1, 1 ]
        x = 2 * x - 1
        return x

    def denormalize(self, x, eps=1e-4):
        """
            x : [ -1, 1 ]
        """
        if x.max() > 1 + eps or x.min() < -1 - eps:
            # print(f'[ datasets/mujoco ] Warning: sample out of range | ({x.min():.4f}, {x.max():.4f})')
            x = torch.clip(x, -1, 1)

        # [ -1, 1 ] --> [ 0, 1 ]
        x = (x + 1) / 2.

        return x * (self.maxs - self.mins) + self.mins


class SafeLimitsNormalizer(LimitsNormalizer):
    """
        functions like LimitsNormalizer, but can handle data for which a dimension is constant
    """

    def __init__(self, *args, eps=1, **kwargs):
        super().__init__(*args, **kwargs)
        for i in range(len(self.mins)):
            if self.mins[i] == self.maxs[i]:
                print(f'''
                    [ utils/normalization ] Constant data in dimension {i} | '''
                      f'''max = min = {self.maxs[i]}'''
                      )
                self.mins -= eps
                self.maxs += eps


class FixedLimitsNormalizer(LimitsNormalizer):
    """
        functions like LimitsNormalizer, but with fixed limits not derived from the data
    """

    def __init__(self, *args, minimum=-1, maximum=1, **kwargs):
        super().__init__(*args, **kwargs)
        self.mins = torch.ones_like(self.mins) * minimum
        self.maxs = torch.ones_like(self.maxs) * maximum
