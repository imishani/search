import abc
import numpy as np


class BasePolicy(object, metaclass=abc.ABCMeta):
    """
    Base class for RL policies.
    """
    @abc.abstractmethod
    def get_action(self, obs: np.ndarray, **kwargs) -> np.ndarray:
        """
        Get an action from the policy.
        :param obs: The state of the environment.
        :param kwargs:
        :return: The action to take.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def update(self, *args, **kwargs):
        """
        Update the policy.
        :param args:
        :param kwargs:
        :return:
        """
        raise NotImplementedError
