"""Script to play a checkpoint if an RL agent from RSL-RL."""

"""Launch Isaac Sim Simulator first."""

import argparse

from isaaclab.app import AppLauncher
from berkeley_humanoid_lite.training import add_rsl_rl_args, run_policy_playback

# add argparse arguments
parser = argparse.ArgumentParser(description="Play a checkpoint with an RSL-RL policy.")
parser.add_argument("--video", action="store_true", default=False, help="Record videos during training.")
parser.add_argument("--video_length", type=int, default=200, help="Length of the recorded video (in steps).")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
add_rsl_rl_args(parser)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
# always enable cameras to record video
if args_cli.video:
    args_cli.enable_cameras = True

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

from berkeley_humanoid_lite import register_tasks

register_tasks()


def main():
    """Play with RSL-RL agent."""
    run_policy_playback(arguments=args_cli, simulation_app=simulation_app)


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
