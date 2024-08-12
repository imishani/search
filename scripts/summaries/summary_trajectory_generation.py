import einops
import matplotlib.pyplot as plt
import numpy as np
import torch
import wandb

from scripts.models import build_context
from scripts.scene import SceneInterfaceBase
from scripts.summaries.summary_base import SummaryBase

# Fix if QtWidgets and QtWidgets.QApplication.instance():
# AttributeError: module 'PySide2.QtWidgets' has no attribute 'QApplication'
import matplotlib
matplotlib.use('Agg')


class SummaryTrajectoryGeneration(SummaryBase):

    def __init__(self,
                 scene_interface: SceneInterfaceBase,
                 project_name: str = "mosaic",
                 **kwargs):
        super().__init__(**kwargs)
        self.scene_interface = scene_interface
        wandb.init(project=project_name)

    def compute_fraction_free_trajs(self, trajs):
        n_free = 0
        for traj in trajs:
            if not self.scene_interface.is_trajectory_collision(traj):
                n_free += 1
        return n_free / len(trajs)

    def summary_fn(self, train_step=None, model=None, datasubset=None, prefix='', debug=False, **kwargs):

        dataset = datasubset.dataset

        # ------------------------------------------------------------------------------------
        n_samples = 25
        trajectory_id = np.random.choice(datasubset.indices, size=n_samples, replace=False)

        data_normalized = dataset[trajectory_id]
        context = build_context(model, dataset, data_normalized)

        # ------------------------------------------------------------------------------------
        # Sample trajectories with the diffusion/cvae model
        horizon = dataset.n_support_points
        hard_conds = dataset.get_batch_hard_conditions(data_normalized[f"{dataset.field_key_traj}_normalized"])

        trajs_normalized = model.run_inference(
            context, hard_conds,
            n_samples=n_samples, horizon=horizon,
            deterministic_steps=0
        )

        # unnormalize trajectory samples from the diffusion model
        trajs = dataset.denormalize(trajs_normalized, dataset.field_key_traj)
        trajs_ground_truth = dataset.denormalize(data_normalized['trajs_normalized'], dataset.field_key_traj)

        # ------------------------------------------------------------------------------------
        # STATISTICS
        wandb.log({f'{prefix}percentage free trajs': self.compute_fraction_free_trajs(trajs)}, step=train_step)
        # wandb.log({f'{prefix}percentage collision intensity': dataset.task.compute_collision_intensity_trajs(trajs)},
        #           step=train_step)
        # wandb.log({f'{prefix}success': dataset.task.compute_success_free_trajs(trajs)}, step=train_step)

        # ------------------------------------------------------------------------------------
        # Render

        # # dataset trajectory
        fig_joint_trajs_dataset, fig_robot_trajs_dataset = None, None
        fig_robot_trajs_dataset = self.scene_interface.render(trajectories=trajs_ground_truth)

        # fig_joint_trajs_dataset, _, fig_robot_trajs_dataset, _ = dataset.render(
        #     task_id=task_id,
        #     render_joint_trajectories=True
        # )
        #
        # # diffusion trajectory
        # pos_trajs = dataset.robot.get_position(trajs)
        # start_state_pos = pos_trajs[0][0]
        # goal_state_pos = pos_trajs[0][-1]
        #
        fig_joint_trajs_diffusion = None
        # fig_joint_trajs_diffusion, _ = dataset.planner_visualizer.plot_joint_space_state_trajectories(
        #     trajs=pos_trajs,
        #     pos_start_state=start_state_pos, pos_goal_state=goal_state_pos,
        #     vel_start_state=torch.zeros_like(start_state_pos), vel_goal_state=torch.zeros_like(goal_state_pos),
        #     linestyle='dashed'
        # )
        #
        fig_robot_trajs_diffusion = self.scene_interface.render(trajectories=trajs)
        # fig_robot_trajs_diffusion, _ = dataset.planner_visualizer.render_robot_trajectories(
        #     trajs=pos_trajs, start_state=start_state_pos, goal_state=goal_state_pos,
        #     linestyle='dashed'
        # )

        if fig_joint_trajs_dataset is not None:
            wandb.log({f"{prefix}joint trajectories DATASET": wandb.Image(fig_joint_trajs_dataset)}, step=train_step)
        if fig_robot_trajs_dataset is not None:
            wandb.log({f"{prefix}robot trajectories DATASET": wandb.Image(fig_robot_trajs_dataset)}, step=train_step)

        if fig_joint_trajs_diffusion is not None:
            wandb.log({f"{prefix}joint trajectories DIFFUSION": wandb.Image(fig_joint_trajs_diffusion)},
                      step=train_step)
        if fig_robot_trajs_diffusion is not None:
            wandb.log({f"{prefix}robot trajectories DIFFUSION": wandb.Image(fig_robot_trajs_diffusion)},
                      step=train_step)

        if debug:
            plt.show()

        if fig_joint_trajs_dataset is not None:
            plt.close(fig_joint_trajs_dataset)
        if fig_robot_trajs_dataset is not None:
            plt.close(fig_robot_trajs_dataset)
        if fig_joint_trajs_diffusion is not None:
            plt.close(fig_joint_trajs_diffusion)
        if fig_robot_trajs_diffusion is not None:
            plt.close(fig_robot_trajs_diffusion)
