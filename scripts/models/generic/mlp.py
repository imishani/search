from torch import nn

from scripts.models.layers import MLP


class MLPModel(nn.Module):
    def __init__(self,
                 in_dim: int = 16,
                 out_dim: int = 16,
                 input_field: str = 'x',
                 output_field: str = 'y',
                 pretrained_dir=None,
                 **kwargs):
        super().__init__()

        self.input_field = input_field
        self.output_field = output_field

        self.in_dim = in_dim
        self.out_dim = out_dim

        self._net = MLP(in_dim, out_dim, **kwargs)

    def forward(self, input):
        x = input[self.input_field]
        out = self._net(x)
        return {self.output_field: out}
