import abc
import itertools
from torch import nn
from torch import optim

from torch import distributions

from scripts.utils_torch import *
from scripts.models.rl_policies.base_policy import BasePolicy
from scripts.models.layers import MLP
from scripts.losses import *


class MLPPolicy(BasePolicy, nn.Module, metaclass=abc.ABCMeta):

    def __init__(self,
                 ac_dim,
                 ob_dim,
                 n_layers,
                 size,
                 activation: str = 'relu',
                 discrete: bool = True,
                 learning_rate: float = 1e-4,
                 loss_fn: WeightedLoss = WeightedL2(),
                 training: bool = True,
                 nn_baseline: bool = False,
                 **kwargs
                 ):
        super().__init__(**kwargs)

        # init vars
        self.ac_dim = ac_dim
        self.ob_dim = ob_dim
        self.n_layers = n_layers
        self.discrete = discrete
        self.size = size
        self.learning_rate = learning_rate
        self.loss_fn = loss_fn
        self.training = training
        self.nn_baseline = nn_baseline

        if self.discrete:
            # self.logits_na = ptu.build_mlp(input_size=self.ob_dim,
            #                                output_size=self.ac_dim,
            #                                n_layers=self.n_layers,
            #                                size=self.size)
            self.logits_na = MLP(
                in_dim=self.ob_dim,
                out_dim=self.ac_dim,
                hidden_dim=self.size,
                n_layers=self.n_layers,
                act=activation
            )

            self.mean_net = None
            self.logstd = None
            self.optimizer = optim.Adam(self.logits_na.parameters(),
                                        self.learning_rate)
        else:
            self.logits_na = None
            self.mean_net = MLP(
                in_dim=self.ob_dim,
                out_dim=self.ac_dim,
                hidden_dim=self.size,
                n_layers=self.n_layers,
                act=activation
            )
            self.logstd = nn.Parameter(
                torch.zeros(self.ac_dim, dtype=torch.float32)
            )
            self.optimizer = optim.Adam(
                itertools.chain([self.logstd], self.mean_net.parameters()),
                self.learning_rate
            )

        if nn_baseline:
            self.baseline = MLP(
                in_dim=self.ob_dim,
                out_dim=1,
                hidden_dim=self.size,
                n_layers=self.n_layers,
                act=activation
            )
            self.baseline_optimizer = optim.Adam(
                self.baseline.parameters(),
                self.learning_rate,
            )
        else:
            self.baseline = None

    ##################################

    def save(self, filepath):
        torch.save(self.state_dict(), filepath)

    ##################################
    def get_action(self, obs: np.ndarray, **kwargs) -> np.ndarray:
        # TODO: get this from hw1 or hw2
        if len(obs.shape) > 1:
            observation = obs
        else:
            observation = obs[None]

        observation = to_torch(observation, device=self.logits_na.device if self.discrete else self.mean_net.device)
        action_distribution = self.forward(observation)
        action = action_distribution.sample()
        return to_numpy(action)

    # update/train this policy
    def update(self, observations, actions, **kwargs):
        raise NotImplementedError

    # This function defines the forward pass of the network.
    # You can return anything you want, but you should be able to differentiate
    # through it. For example, you can return a torch.FloatTensor. You can also
    # return more flexible objects, such as a
    # `torch.distributions.Distribution` object. It's up to you!
    def forward(self, observation: torch.Tensor):
        if self.discrete:
            logits = self.logits_na(observation)
            action_distribution = distributions.Categorical(logits=logits)
            return action_distribution
        else:
            batch_mean = self.mean_net(observation)
            scale_tril = torch.diag(torch.exp(self.logstd))
            batch_dim = batch_mean.shape[0]
            batch_scale_tril = scale_tril.repeat(batch_dim, 1, 1)
            action_distribution = distributions.MultivariateNormal(
                batch_mean,
                scale_tril=batch_scale_tril,
            )
            return action_distribution

    def loss(self, observations, actions, adv_n=None):
        forward = self.forward(observations)
        log_prob = forward.log_prob(actions)
        if adv_n is not None:
            loss = -torch.mean(log_prob * adv_n)
        else:
            loss = -torch.mean(log_prob)
        return loss

#####################################################
#####################################################


# class MLPPolicyAC(MLPPolicy):
#     def update(self, observations, actions, adv_n=None):
#         # TODO: update the policy and return the loss
#         forward = self.forward(ptu.from_numpy(observations))
#         log_prob = forward.log_prob(ptu.from_numpy(actions))
#         if adv_n is not None:
#             loss = -torch.mean(log_prob * ptu.from_numpy(adv_n))
#         else:
#             loss = -torch.mean(log_prob)
#         self.optimizer.zero_grad()
#         loss.backward()
#         self.optimizer.step()
#         return loss.item()
