import os
import torch
import trainer
from models import UNET_DIM_MULTS, TemporalUnet
from utils_torch import get_torch_device, fix_random_seed
from scene import scene_2d


def experiment(
        ########################################################################
        # Dataset
        # dataset_subdir: str = 'EnvTableFree3D-RobotPanda',
        dataset_path: str = '/home/itamar/work/code/algorithms/search/domains/2d_robot_nav/data/den501d/data/paths.h5',

        ########################################################################
        # Diffusion Model
        diffusion_model_class: str = 'GaussianDiffusionModel',
        variance_schedule: str = 'exponential',  # cosine
        n_diffusion_steps: int = 25,
        predict_epsilon: bool = True,

        # Unet
        unet_input_dim: int = 32,
        unet_dim_mults_option: int = 1,

        ########################################################################
        # Loss
        loss_class: str = 'GaussianDiffusionLoss',

        # Training parameters
        batch_size: int = 512,
        lr: float = 1e-4,
        num_train_steps: int = 150_000,  # 500000

        use_ema: bool = True,
        use_amp: bool = False,

        # Summary parameters
        steps_til_summary: int = 500,
        summary_class: str = 'SummaryTrajectoryGeneration',

        steps_til_ckpt: int = 2000,  # 50000,

        ########################################################################
        device: str = 'cuda',

        debug: bool = False,

        ########################################################################
        # MANDATORY
        seed: int = 1,
        results_dir: str = 'logs',

        ########################################################################
        # WandB
        wandb_mode: str = 'disabled',  # "online", "offline" or "disabled"
        wandb_entity: str = 'scoreplan',
        wandb_project: str = 'test_train',
        **kwargs
):
    fix_random_seed(seed)

    device = get_torch_device(device=device)
    tensor_args = {'device': device, 'dtype': torch.float32}

    # Dataset
    train_subset, train_dataloader, val_subset, val_dataloader = trainer.get_dataset(
        dataset_class='TrajectoryDataset',
        dataset_path=dataset_path,
        batch_size=batch_size,
        results_dir=results_dir,
        save_indices=True,
        tensor_args=tensor_args
    )

    dataset = train_subset.dataset

    # Model
    diffusion_configs = dict(
        variance_schedule=variance_schedule,
        n_diffusion_steps=n_diffusion_steps,
        predict_epsilon=predict_epsilon,
    )

    unet_configs = dict(
        state_dim=dataset.state_dim,
        n_support_points=dataset.n_support_points,
        unet_input_dim=unet_input_dim,
        dim_mults=UNET_DIM_MULTS[unet_dim_mults_option],
    )

    model = trainer.get_model(
        model_class=diffusion_model_class,
        model=TemporalUnet(**unet_configs),
        tensor_args=tensor_args,
        **diffusion_configs,
        **unet_configs
    )

    # Loss
    loss_fn = val_loss_fn = trainer.get_loss(
        loss_class=loss_class
    )

    # Summary
    scene2d = scene_2d.Scene2D('/home/itamar/work/code/algorithms/search/domains/2d_robot_nav/data/den501d/den501d.map')
    summary_fn = trainer.get_summary(
        summary_class=summary_class,
        scene_interface=scene2d
    )

    # print the number of parameters
    print(f"Number of parameters: {sum(p.numel() for p in model.parameters())}")

    # Train
    trainer.train(
        model=model,
        train_dataloader=train_dataloader,
        train_subset=train_subset,
        val_dataloader=val_dataloader,
        val_subset=train_subset,
        epochs=trainer.get_num_epochs(num_train_steps, batch_size, len(dataset)),
        model_dir=results_dir,
        summary_fn=summary_fn,
        lr=lr,
        loss_fn=loss_fn,
        val_loss_fn=val_loss_fn,
        steps_til_summary=steps_til_summary,
        steps_til_checkpoint=steps_til_ckpt,
        clip_grad=True,
        use_ema=use_ema,
        use_amp=use_amp,
        debug=debug,
        tensor_args=tensor_args
    )


if __name__ == '__main__':
    # Leave unchanged
    experiment()
