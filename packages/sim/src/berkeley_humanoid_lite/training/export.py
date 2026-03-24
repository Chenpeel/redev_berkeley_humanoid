from __future__ import annotations

from pathlib import Path


def _resolve_actuator_scalar(
    actuator_name: str,
    actuator_group: object,
    *,
    primary_name: str,
    fallback_names: tuple[str, ...] = (),
) -> float:
    """读取 actuator 配置中的标量参数，并兼容新旧字段名。"""

    for attribute_name in (primary_name, *fallback_names):
        value = getattr(actuator_group, attribute_name, None)
        if value is not None:
            return value

    expected_names = ", ".join((primary_name, *fallback_names))
    raise ValueError(
        f"Actuator group {actuator_name!r} does not define any of: {expected_names}"
    )


def export_policy_models(ppo_runner: object, export_directory: str | Path) -> Path:
    """导出 JIT 与 ONNX 策略模型。"""
    from isaaclab_rl.rsl_rl import export_policy_as_jit, export_policy_as_onnx

    resolved_export_directory = Path(export_directory)
    resolved_export_directory.mkdir(parents=True, exist_ok=True)

    policy_obs_normalizer = getattr(ppo_runner, "obs_normalizer", None)
    if policy_obs_normalizer is None:
        policy_obs_normalizer = getattr(ppo_runner.alg.policy, "actor_obs_normalizer", None)

    export_policy_as_jit(
        ppo_runner.alg.policy,
        policy_obs_normalizer,
        path=str(resolved_export_directory),
        filename="policy.pt",
    )
    export_policy_as_onnx(
        ppo_runner.alg.policy,
        normalizer=policy_obs_normalizer,
        path=str(resolved_export_directory),
        filename="policy.onnx",
    )
    return resolved_export_directory


def build_policy_deployment_configuration(
    *,
    env_cfg: object,
    env: object,
    export_directory: str | Path,
) -> dict[str, object]:
    """从 Isaac Lab 配置与环境实例抽取部署配置。"""
    import isaaclab.utils.string as string_utils
    import torch

    joint_names = list(env_cfg.scene.robot.init_state.joint_pos.keys())
    initial_joint_positions = list(env_cfg.scene.robot.init_state.joint_pos.values())
    num_joints = len(joint_names)

    joint_kp = torch.zeros(num_joints, device=env.unwrapped.device)
    joint_kd = torch.zeros(num_joints, device=env.unwrapped.device)
    effort_limits = torch.zeros(num_joints, device=env.unwrapped.device)

    for actuator_name, actuator_group in env_cfg.scene.robot.actuators.items():
        match_expression_list = list(actuator_group.joint_names_expr)
        match_expression_dict = {expression: None for expression in match_expression_list}
        indices, _, _ = string_utils.resolve_matching_names_values(
            match_expression_dict,
            joint_names,
            preserve_order=True,
        )
        joint_kp[indices] = actuator_group.stiffness
        joint_kd[indices] = actuator_group.damping
        effort_limits[indices] = _resolve_actuator_scalar(
            actuator_name,
            actuator_group,
            primary_name="effort_limit",
            fallback_names=("effort_limit_sim",),
        )

    action_expression_dict = {expression: None for expression in env_cfg.actions.joint_pos.joint_names}
    action_indices, _, _ = string_utils.resolve_matching_names_values(
        action_expression_dict,
        joint_names,
        preserve_order=True,
    )

    resolved_export_directory = Path(export_directory)
    return {
        "policy_checkpoint_path": str(resolved_export_directory / "policy.onnx"),
        "ip_robot_addr": "127.0.0.1",
        "ip_policy_obs_port": 10000,
        "ip_host_addr": "127.0.0.1",
        "ip_policy_acs_port": 10001,
        "control_dt": 0.004,
        "policy_dt": env_cfg.sim.dt * env_cfg.decimation,
        "physics_dt": 0.0005,
        "cutoff_freq": 1000,
        "num_joints": num_joints,
        "joints": joint_names,
        "joint_kp": joint_kp.tolist(),
        "joint_kd": joint_kd.tolist(),
        "effort_limits": effort_limits.tolist(),
        "default_base_position": env_cfg.scene.robot.init_state.pos,
        "default_joint_positions": initial_joint_positions,
        "num_observations": env.observation_space["policy"].shape[-1],
        "history_length": env_cfg.observations.policy.actions.history_length,
        "command_velocity": env_cfg.observations.policy.velocity_commands.func(
            env.unwrapped,
            env_cfg.observations.policy.velocity_commands.params["command_name"],
        )[0].tolist(),
        "num_actions": env.action_space.shape[-1],
        "action_scale": env_cfg.actions.joint_pos.scale,
        "action_indices": action_indices,
        "action_limit_lower": -10000,
        "action_limit_upper": 10000,
    }
