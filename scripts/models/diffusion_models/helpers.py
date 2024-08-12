import numpy as np
import torch
from torch import nn
import torch.nn.functional as F
# package imports
from scripts.utils_torch import to_torch
from scripts.losses import WeightedL1, WeightedL2


# ----------------------------------------------------------------------------- #
Losses = {
    'l1': WeightedL1,
    'l2': WeightedL2,
}


# ----------------------------------------------------------------------------- #
# ---------------------------- variance schedules ----------------------------- #
# ----------------------------------------------------------------------------- #
def linear_beta_schedule(n_diffusion_steps, beta_start=0.0001, beta_end=0.02):
    return torch.linspace(beta_start, beta_end, n_diffusion_steps)


def quadratic_beta_schedule(n_diffusion_steps, beta_start=0.0001, beta_end=0.02):
    return torch.linspace(beta_start ** 0.5, beta_end ** 0.5, n_diffusion_steps) ** 2


def sigmoid_beta_schedule(n_diffusion_steps, beta_start=0.0001, beta_end=0.02):
    betas = torch.linspace(-6, 6, n_diffusion_steps)
    return torch.sigmoid(betas) * (beta_end - beta_start) + beta_start


def cosine_beta_schedule(n_diffusion_steps, s=0.008, a_min=0, a_max=0.999, dtype=torch.float32):
    """
    cosine schedule
    as proposed in https://openreview.net/forum?id=-NEXDKk8gZ
    """
    steps = n_diffusion_steps + 1
    x = np.linspace(0, steps, steps)
    alphas_cumprod = np.cos(((x / steps) + s) / (1 + s) * np.pi * 0.5) ** 2
    alphas_cumprod = alphas_cumprod / alphas_cumprod[0]
    betas = 1 - (alphas_cumprod[1:] / alphas_cumprod[:-1])
    betas_clipped = np.clip(betas, a_min=a_min, a_max=a_max)
    return to_torch(betas_clipped, dtype=dtype)


def exponential_beta_schedule(n_diffusion_steps, beta_start=1e-4, beta_end=1.0):
    # exponential increasing noise from t=0 to t=T
    x = torch.linspace(0, n_diffusion_steps, n_diffusion_steps)
    beta_start = to_torch(beta_start)
    beta_end = to_torch(beta_end)
    a = 1 / n_diffusion_steps * torch.log(beta_end / beta_start)
    return beta_start * torch.exp(a * x)


def constant_fraction_beta_schedule(n_diffusion_steps):
    # exponential increasing noise from t=0 to t=T
    x = torch.linspace(0, n_diffusion_steps, n_diffusion_steps)
    return 1 / (n_diffusion_steps - x + 1)


def variance_preserving_beta_schedule(n_diffusion_steps, beta_start=1e-4, beta_end=1.0):
    # Works only with a small number of diffusion steps
    # https://arxiv.org/abs/2112.07804
    # https://openreview.net/pdf?id=AHvFDPi-FA
    x = torch.linspace(0, n_diffusion_steps, n_diffusion_steps)
    alphas = torch.exp(
        -beta_start * (1 / n_diffusion_steps) - 0.5 * (beta_end - beta_start) * (2 * x - 1) / (n_diffusion_steps ** 2))
    betas = 1 - alphas
    return betas



