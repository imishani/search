import abc
import itertools

import torch
from torch import nn
from torch import optim

from torch import distributions

from scripts.utils_torch import *
from scripts.models.rl_policies.base_policy import BasePolicy
from scripts.models.layers import MLP
from scripts.losses import *


class ArgMaxPolicy(BasePolicy, nn.Module, metaclass=abc.ABCMeta):

    def __init__(self,
                 ac_dim,
                 ob_dim,
                 n_layers,
                 size,
                 activation: str = 'relu',
                 discrete: bool = True,
                 learning_rate: float = 1e-4,
                 loss_fn: WeightedLoss = SmoothL1Loss(),
                 training: bool = True,
                 double_q: bool = False,
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
        self.double_q = double_q

        if self.discrete:
            self.q_net = MLP(
                in_dim=self.ob_dim,
                out_dim=self.ac_dim * 2,
                hidden_dim=self.size,
                n_layers=self.n_layers,
                act=activation,
                batch_norm=False
            )

            self.mean_net = None
            self.logstd = None
            # self.optimizer = optim.Adam(self.q_net.parameters(),
            #                             self.learning_rate)
        else:
            self.q_net = MLP(
                in_dim=self.ob_dim,
                out_dim=self.ac_dim * 2,
                hidden_dim=self.size,
                n_layers=self.n_layers,
                act=activation,
                batch_norm=False
            )
            self.logstd = nn.Parameter(
                torch.zeros(self.ac_dim, dtype=torch.float32)
            )
            # self.optimizer = optim.Adam(
            #     itertools.chain([self.logstd], self.q_net.parameters()),
            #     self.learning_rate
            # )

    ##################################

    def save(self, filepath):
        torch.save(self.state_dict(), filepath)

    ##################################
    def get_action(self, obs: torch.Tensor, **kwargs) -> torch.Tensor:
        if len(obs.shape) > 1:
            observation = obs
        else:
            observation = obs[None]

        # # observation = to_torch(observation, device=self.logits_na.device if self.discrete else self.mean_net.device)
        # action_distribution = self.forward(observation)
        # action = action_distribution.sample()
        # action = self.q_net(observation).argmax(axis=-1)

        q_cost_vals = self.q_net(observation)
        q_vals, cost_vals = torch.chunk(q_cost_vals, 2, dim=1)
        # take the sum
        v_vals = q_vals + cost_vals
        action = q_vals.argmin(axis=-1)

        return action.squeeze()
        # return action

    # update/train this policy
    def update(self, ob_no, ac_na, next_ob_no, reward_n, q_next, **kwargs):
        """
            Update the parameters of the critic.
            let sum_of_path_lengths be the sum of the lengths of the paths sampled from
                Agent.sample_trajectories
            let num_paths be the number of paths sampled from Agent.sample_trajectories
            arguments:
                ob_no: shape: (sum_of_path_lengths, ob_dim)
                ac_na: length: sum_of_path_lengths. The action taken at the current step.
                next_ob_no: shape: (sum_of_path_lengths, ob_dim). The observation after taking one step forward
                reward_n: length: sum_of_path_lengths. Each element in reward_n is a scalar containing
                    the reward for each timestep
                q_next: length: sum_of_path_lengths. Each element in q_next is a scalar containing
            returns:
                nothing
        """
        # ob_no = to_torch(ob_no, device=self.q_net.device)
        # ac_na = to_torch(ac_na, device=self.q_net.device)
        # next_ob_no = to_torch(next_ob_no, device=self.q_net.device)
        # reward_n = to_torch(reward_n, device=self.q_net.device)
        # q_next = to_torch(q_next, device=self.q_net.device)

        qa_t_values = self.q_net(ob_no)
        # q_t_values = torch.gather(qa_t_values, 1, ac_na.unsqueeze(1)).squeeze(1)
        q_out, cost_out = torch.chunk(qa_t_values, 2, dim=1)
        q_out = torch.gather(q_out, 1, ac_na.unsqueeze(1)).squeeze(1)
        cost_out = torch.gather(cost_out, 1, ac_na.unsqueeze(1)).squeeze(1)

        # q_values_next = self.q_net(next_ob_no)

        terminal_n = torch.zeros_like(reward_n)
        # target_q_values = reward_n + (1 - terminal_n) * 0.99 * q_values_next.max(dim=1)[0]
        # target_q_values = reward_n + (1 - terminal_n) * 0.99 * q_next
        target_q_values = q_next

        # target_q_values = torch.ones_like(qa_t_values, dtype=torch.float32, device=qa_t_values.device)
        # at index ac_na, set the target_q_values to q_next
        # target_q_values.scatter_(1, ac_na.unsqueeze(1), q_next.unsqueeze(1))

        loss1, info1 = self.loss_fn(q_out, target_q_values)
        loss2, info2 = self.loss_fn(cost_out, reward_n.unsqueeze(1))
        # loss, info = self.loss_fn(qa_t_values, target_q_values)
        return loss1 + loss2, info1

        # # Use self.forward to compute the action distribution and loss
        # action_distribution = self.forward(ob_no)
        # loss = -action_distribution.log_prob(ac_na) * q_next
        # loss = loss.mean()
        # return loss, {}

    # This function defines the forward pass of the network.
    # You can return anything you want, but you should be able to differentiate
    # through it. For example, you can return a torch.FloatTensor. You can also
    # return more flexible objects, such as a
    # `torch.distributions.Distribution` object. It's up to you!
    def forward(self, observation: torch.Tensor):
        if self.discrete:
            logits = self.q_net(observation)
            action_distribution = distributions.Categorical(logits=logits)
            return action_distribution
        else:
            batch_mean = self.q_net(observation)
            scale_tril = torch.diag(torch.exp(self.logstd))
            batch_dim = batch_mean.shape[0]
            batch_scale_tril = scale_tril.repeat(batch_dim, 1, 1)
            action_distribution = distributions.MultivariateNormal(
                batch_mean,
                scale_tril=batch_scale_tril,
            )
            return action_distribution

    # def loss(self,

#####################################################
#####################################################
