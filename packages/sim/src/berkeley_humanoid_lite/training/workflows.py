from __future__ import annotations

from datetime import datetime
from pathlib import Path

from .arguments import parse_rsl_rl_cfg, to_rsl_rl_runner_cfg, update_rsl_rl_cfg
from .artifacts import dump_training_configuration, save_policy_deployment_configuration
from .checkpoints import resolve_checkpoint_path
from .export import build_policy_deployment_configuration, export_policy_models
from .paths import get_policy_export_config_path, get_rsl_rl_logs_dir


def _configure_torch_runtime() -> None:
    import torch

    torch.backends.cuda.matmul.allow_tf32 = True
    torch.backends.cudnn.allow_tf32 = True
    torch.backends.cudnn.deterministic = False
    torch.backends.cudnn.benchmark = False


def _create_training_log_directories(*, experiment_name: str, run_name: str | None) -> tuple[Path, Path, Path]:
    log_root_path = get_rsl_rl_logs_dir() / experiment_name
    isaaclab_log_directory = log_root_path / "isaaclab"
    isaaclab_log_directory.mkdir(parents=True, exist_ok=True)

    run_directory_name = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    if run_name:
        run_directory_name += f"_{run_name}"
    log_directory = log_root_path / run_directory_name
    return log_root_path, log_directory, isaaclab_log_directory


def _maybe_wrap_video(
    environment: object,
    *,
    enabled: bool,
    video_folder: Path,
    video_length: int,
    step_trigger: object,
    log_message: str,
) -> object:
    if not enabled:
        return environment

    import gymnasium as gym
    from isaaclab.utils.dict import print_dict

    video_kwargs = {
        "video_folder": str(video_folder),
        "step_trigger": step_trigger,
        "video_length": video_length,
        "disable_logger": True,
    }
    print(log_message)
    print_dict(video_kwargs, nesting=4)
    return gym.wrappers.RecordVideo(environment, **video_kwargs)


def _wrap_environment_for_rsl_rl(environment: object) -> object:
    from isaaclab.envs import DirectMARLEnv, multi_agent_to_single_agent
    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper

    if isinstance(environment.unwrapped, DirectMARLEnv):
        environment = multi_agent_to_single_agent(environment)
    return RslRlVecEnvWrapper(environment)


def run_training(*, task_name: str, env_cfg: object, agent_cfg: object, arguments: object, caller_file: str) -> None:
    import gymnasium as gym

    from rsl_rl.runners import OnPolicyRunner

    _configure_torch_runtime()

    agent_cfg = update_rsl_rl_cfg(agent_cfg, arguments)
    env_cfg.scene.num_envs = arguments.num_envs if arguments.num_envs is not None else env_cfg.scene.num_envs
    agent_cfg.max_iterations = (
        arguments.max_iterations if arguments.max_iterations is not None else agent_cfg.max_iterations
    )
    env_cfg.seed = agent_cfg.seed
    env_cfg.sim.device = arguments.device if arguments.device is not None else env_cfg.sim.device

    log_root_path, log_directory, isaaclab_log_directory = _create_training_log_directories(
        experiment_name=agent_cfg.experiment_name,
        run_name=agent_cfg.run_name,
    )
    print(f"[INFO] Logging experiment in directory: {log_root_path}")

    if hasattr(env_cfg.sim, "log_dir"):
        env_cfg.sim.log_dir = str(isaaclab_log_directory)

    environment = gym.make(task_name, cfg=env_cfg, render_mode="rgb_array" if arguments.video else None)
    environment = _maybe_wrap_video(
        environment,
        enabled=arguments.video,
        video_folder=log_directory / "videos" / "train",
        video_length=arguments.video_length,
        step_trigger=lambda step: step % arguments.video_interval == 0,
        log_message="[INFO] Recording videos during training.",
    )
    environment = _wrap_environment_for_rsl_rl(environment)

    runner = OnPolicyRunner(
        environment,
        to_rsl_rl_runner_cfg(agent_cfg),
        log_dir=str(log_directory),
        device=agent_cfg.device,
    )
    runner.add_git_repo_to_log(caller_file)
    if agent_cfg.resume:
        resume_path = resolve_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)
        print(f"[INFO]: Loading model checkpoint from: {resume_path}")
        runner.load(str(resume_path))

    dump_training_configuration(log_directory, env_cfg, agent_cfg)
    runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)
    environment.close()


def run_policy_playback(*, arguments: object, simulation_app: object) -> None:
    import gymnasium as gym
    import torch

    from rsl_rl.runners import OnPolicyRunner

    from isaaclab_tasks.utils import parse_env_cfg

    env_cfg = parse_env_cfg(
        arguments.task,
        device=arguments.device,
        num_envs=arguments.num_envs,
        use_fabric=not arguments.disable_fabric,
    )
    agent_cfg = parse_rsl_rl_cfg(arguments.task, arguments)

    log_root_path = get_rsl_rl_logs_dir() / agent_cfg.experiment_name
    print(f"[INFO] Loading experiment from directory: {log_root_path}")
    resume_path = resolve_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)
    log_directory = resume_path.parent

    environment = gym.make(arguments.task, cfg=env_cfg, render_mode="rgb_array" if arguments.video else None)
    environment = _maybe_wrap_video(
        environment,
        enabled=arguments.video,
        video_folder=log_directory / "videos" / "play",
        video_length=arguments.video_length,
        step_trigger=lambda step: step == 0,
        log_message="[INFO] Recording videos during playback.",
    )
    environment = _wrap_environment_for_rsl_rl(environment)

    print(f"[INFO]: Loading model checkpoint from: {resume_path}")
    ppo_runner = OnPolicyRunner(
        environment,
        to_rsl_rl_runner_cfg(agent_cfg),
        log_dir=None,
        device=agent_cfg.device,
    )
    ppo_runner.load(str(resume_path))

    policy = ppo_runner.get_inference_policy(device=environment.unwrapped.device)
    export_directory = export_policy_models(ppo_runner, log_directory / "exported")
    deployment_configuration = build_policy_deployment_configuration(
        env_cfg=env_cfg,
        env=environment,
        export_directory=export_directory,
    )
    save_policy_deployment_configuration(deployment_configuration, get_policy_export_config_path())

    observations = environment.get_observations()
    timestep = 0
    while simulation_app.is_running():
        with torch.inference_mode():
            actions = policy(observations)
            observations, _, _, _ = environment.step(actions)
        if arguments.video:
            timestep += 1
            if timestep == arguments.video_length:
                break

    environment.close()
