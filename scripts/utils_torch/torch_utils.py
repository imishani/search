import collections
import numpy as np
import torch


def get_device() -> str:
    """
    Get device.
    """
    return 'cuda' if torch.cuda.is_available() else 'cpu'


def get_torch_device(device='cuda'):
    if 'cuda' in device and torch.cuda.is_available():
        device = 'cuda'
    elif 'mps' in device:
        device = 'mps'
    else:
        device = 'cpu'
    return torch.device(device)


DEFAULT_TENSOR_ARGS = {'device': get_torch_device('cuda'), 'dtype': torch.float32}


def to_torch(x,
             device: str = 'cpu',
             dtype: torch.dtype = torch.float32,
             requires_grad: bool = False,
             clone: bool = False) -> torch.Tensor:
    """
    Convert numpy array to torch tensor.
    x: numpy array or torch tensor
    device: 'cpu' or 'cuda'
    dtype: torch dtype
    requires_grad: whether to track gradients
    clone: whether to clone the tensor
    """
    if isinstance(x, torch.Tensor):
        if clone:
            return x.clone().to(device=device, dtype=dtype)
        else:
            return x.to(device=device, dtype=dtype)
    else:
        return torch.tensor(x, device=device, dtype=dtype, requires_grad=requires_grad)


def to_numpy(x,
             dtype: np.dtype = np.float32,
             clone: bool = False) -> np.ndarray:
    """
    Convert torch tensor to numpy array.
    x: numpy array or torch tensor
    dtype: numpy dtype
    clone: whether to clone the tensor
    """
    if isinstance(x, np.ndarray):
        if clone:
            return x.copy().astype(dtype)
        else:
            return x.astype(dtype)
    else:
        return x.detach().cpu().numpy().astype(dtype)


def to_device(x, device: str = 'cpu') -> torch.Tensor:
    """
    Move tensor to device.
    x: torch tensor
    device: 'cpu' or 'cuda'
    """
    return x.to(device=device)


def dict_to_device(d, device: str = 'cpu') -> dict or torch.Tensor:
    """
    Move dictionary to device.
    d: dictionary
    device: 'cpu' or 'cuda'
    """
    if isinstance(d, collections.abc.Mapping):
        return {k: dict_to_device(v, device=device) for k, v in d.items()}
    else:
        assert isinstance(d, torch.Tensor)
        return d.to(device)


def to_dtype(x, dtype: torch.dtype = torch.float32) -> torch.Tensor:
    """
    Convert tensor to dtype.
    x: torch tensor
    dtype: torch dtype
    """
    return x.to(dtype=dtype)


def to_requires_grad(x, requires_grad: bool = False) -> torch.Tensor:
    """
    Convert tensor to requires_grad.
    x: torch tensor
    requires_grad: whether to track gradients
    """
    return x.requires_grad_(requires_grad)


def freeze_torch_model(model):
    """
    Freeze torch model.
    model: torch model
    """
    assert isinstance(model, torch.nn.Module)
    for param in model.parameters():
        param.requires_grad = False
    model.is_frozen = True


def batched_weighted_dot_prod(x: torch.Tensor, M: torch.Tensor, y: torch.Tensor, with_einsum: bool = False):
    """
    Computes batched version of weighted dot product (distance) x.T @ M @ x
    """
    assert x.ndim >= 2
    if with_einsum:
        My = M.unsqueeze(0) @ y
        r = torch.einsum('...ij,...ij->...j', x, My)
    else:
        r = x.transpose(-2, -1) @ M.unsqueeze(0) @ x
        r = r.diagonal(dim1=-2, dim2=-1)
    return r
