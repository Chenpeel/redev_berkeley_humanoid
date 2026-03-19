"""任务包根入口。"""

from .registry import TASK_REGISTRATIONS, TaskRegistration, register_tasks

__all__ = ["TASK_REGISTRATIONS", "TaskRegistration", "register_tasks"]
