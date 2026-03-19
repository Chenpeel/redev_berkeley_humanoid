from __future__ import annotations

import numpy as np

try:
    import torch
except ImportError:  # pragma: no cover - 无 torch 环境仍允许 numpy 分支被导入
    torch = None


def create_gravity_vector(
    *,
    reference: np.ndarray | torch.Tensor | None = None,
) -> np.ndarray | torch.Tensor:
    """根据参考数据类型创建标准重力向量。"""
    if torch is not None and isinstance(reference, torch.Tensor):
        dtype = reference.dtype if reference.dtype.is_floating_point else torch.float32
        return torch.tensor([0.0, 0.0, -1.0], dtype=dtype, device=reference.device)
    return np.array([0.0, 0.0, -1.0], dtype=np.float32)


def quat_rotate_inverse(
    quaternion: np.ndarray | torch.Tensor,
    vector: np.ndarray | torch.Tensor,
) -> np.ndarray | torch.Tensor:
    """按输入类型分派到 numpy 或 torch 的逆四元数旋转实现。"""
    if torch is not None and (isinstance(quaternion, torch.Tensor) or isinstance(vector, torch.Tensor)):
        return _quat_rotate_inverse_torch(quaternion, vector)
    return _quat_rotate_inverse_numpy(quaternion, vector)


def compute_projected_gravity(
    quaternion: np.ndarray | torch.Tensor,
    gravity_vector: np.ndarray | torch.Tensor | None = None,
) -> np.ndarray | torch.Tensor:
    """计算基坐标系下的重力投影。"""
    effective_gravity_vector = gravity_vector
    if effective_gravity_vector is None:
        effective_gravity_vector = create_gravity_vector(reference=quaternion)
    return quat_rotate_inverse(quaternion, effective_gravity_vector)


def _quat_rotate_inverse_numpy(
    quaternion: np.ndarray | torch.Tensor,
    vector: np.ndarray | torch.Tensor,
) -> np.ndarray:
    quaternion_array = np.asarray(quaternion, dtype=np.float32)
    vector_array = np.asarray(vector, dtype=np.float32)

    quaternion_w = quaternion_array[0]
    quaternion_vector = quaternion_array[1:4]
    a_term = vector_array * (2.0 * quaternion_w**2 - 1.0)
    b_term = np.cross(quaternion_vector, vector_array) * quaternion_w * 2.0
    c_term = quaternion_vector * (np.dot(quaternion_vector, vector_array)) * 2.0
    return a_term - b_term + c_term


def _quat_rotate_inverse_torch(
    quaternion: np.ndarray | torch.Tensor,
    vector: np.ndarray | torch.Tensor,
) -> torch.Tensor:
    if torch is None:  # pragma: no cover - 仅在异常环境下触发
        raise RuntimeError("当前环境未安装 torch，无法处理 torch.Tensor 输入。")

    device = None
    dtype = torch.float32
    if isinstance(quaternion, torch.Tensor):
        device = quaternion.device
        if quaternion.dtype.is_floating_point:
            dtype = quaternion.dtype
    elif isinstance(vector, torch.Tensor):
        device = vector.device
        if vector.dtype.is_floating_point:
            dtype = vector.dtype

    quaternion_tensor = torch.as_tensor(quaternion, dtype=dtype, device=device)
    vector_tensor = torch.as_tensor(vector, dtype=dtype, device=device)

    quaternion_w = quaternion_tensor[0]
    quaternion_vector = quaternion_tensor[1:4]
    a_term = vector_tensor * (2.0 * quaternion_w**2 - 1.0)
    b_term = torch.cross(quaternion_vector, vector_tensor, dim=-1) * quaternion_w * 2.0
    c_term = quaternion_vector * torch.dot(quaternion_vector, vector_tensor) * 2.0
    return a_term - b_term + c_term
