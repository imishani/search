import torch
from torch import nn
import torch.nn.functional as F


# ----------------------------------------------------------------------------- #
# ---------------------------------- losses ----------------------------------- #
# ----------------------------------------------------------------------------- #
class WeightedLoss(nn.Module):

    def __init__(self, weights=None):
        super().__init__()
        self.register_buffer('weights', weights)

    def forward(self, pred, targ):
        """
            pred, targ : tensor
                [ batch_size x horizon x transition_dim ]
        """
        loss = self._loss(pred, targ)
        if self.weights is not None:
            weighted_loss = (loss * self.weights).mean()
        else:
            weighted_loss = loss.mean()
        return weighted_loss, {}


class WeightedL1(WeightedLoss):

    def _loss(self, pred, targ):
        return torch.abs(pred - targ)


class WeightedL2(WeightedLoss):

    def _loss(self, pred, targ):
        return F.mse_loss(pred, targ, reduction='none')
