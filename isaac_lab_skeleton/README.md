# Isaac Lab Skeleton

This folder is a **future-extension skeleton** for the primary robotics-research target platform: Isaac Lab.

The Unity code is the practical first implementation path. This Isaac Lab skeleton is included so the GitHub repository shows the intended direction toward Isaac Lab, reinforcement learning, and sim-to-real robotics workflows.

The file `spherical_drone_env_skeleton.py` is not expected to run without additional Isaac Lab setup, a drone asset, and task registration. It is a planning scaffold for how the Unity environment can later be translated into Isaac Lab.

Future Isaac Lab tasks:

1. Create or import a spherical drone USD asset.
2. Define six motor/thruster attachment points.
3. Define the state vector: position, velocity, orientation, angular velocity, target, motor health.
4. Define the action vector: six thrust commands and optional gimbal commands.
5. Implement reward terms for hover, command tracking, motor energy, and failure recovery.
6. Train with a supported RL backend.
