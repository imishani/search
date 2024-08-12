import abc
import torch


class SceneInterfaceBase(abc.ABC):
    """
    Base class for scene interfaces.
    """
    def __init__(self, **kwargs):
        pass

    @abc.abstractmethod
    def is_collision(self, *args, **kwargs):
        pass

    def is_trajectory_collision(self,
                                trajectory: torch.Tensor,
                                *args,
                                **kwargs):
        """
        Check if a trajectory is in collision.
        :param trajectory: The trajectory to check.
        :param args: Additional arguments.
        :param kwargs: Additional arguments.
        :return: True if the trajectory is in collision, False otherwise.
        """
        for i in range(trajectory.shape[1]):
            if self.is_collision(trajectory[i, :], *args, **kwargs):
                return True
        return False

    @abc.abstractmethod
    def render(self, *args, **kwargs):
        pass
