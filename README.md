# Digital Twin and Machine-Learning Control Allocation for a Six-Motor Spherical Drone

Starter repository for **ME 569 C: Data-Driven Control**.

This project develops a digital twin of a six-motor spherical drone and trains a machine-learning controller to allocate thrust among the motors for hover, movement, energy efficiency, and motor-failure recovery.

## Project direction

- **Primary long-term target:** Isaac Lab for robotics reinforcement learning and future sim-to-real work.
- **Current implementation path:** Unity first, using Unity ML-Agents, because it is faster to build, visualize, debug, and present for a class project.
- **Backup implementation path:** A complete Unity + ML-Agents demo if Isaac Lab setup takes too long.

The backup is **not** removing machine learning. The backup is keeping the ML controller in Unity instead of moving fully into Isaac Lab.

## Repository layout

```text
.
├── UnityStarter/
│   ├── Assets/Scripts/
│   │   ├── DroneMotor.cs
│   │   ├── SixMotorDroneAgent.cs
│   │   ├── DroneSceneBootstrapper.cs
│   │   ├── KeyboardTargetController.cs
│   │   └── MotorFailureScheduler.cs
│   └── configs/
│       └── spherical_drone_ppo.yaml
├── isaac_lab_skeleton/
│   ├── README.md
│   └── spherical_drone_env_skeleton.py
├── docs/
│   ├── CONTROL_DESIGN.md
│   └── UNITY_SETUP.md
├── .gitignore
└── README.md
```

## What this starter code does

The Unity starter code creates a simple six-motor spherical drone:

- A spherical drone body with a Rigidbody.
- Six motors mounted around the sphere like dice faces.
- A reinforcement-learning agent using Unity ML-Agents.
- Continuous actions for six thrust commands and optional pitch/yaw gimbal commands.
- Observations for position, velocity, orientation, angular velocity, target position, motor health, and previous motor commands.
- A starter reward function for hover stability, command tracking, energy efficiency, and survival inside the flight area.
- Motor failure utilities for testing damaged or weakened motors.

## Basic Unity setup

1. Create a new Unity 3D project.
2. Install the Unity ML-Agents package.
3. Copy the `UnityStarter/Assets/Scripts` folder into your Unity project's `Assets` folder.
4. Create an empty GameObject named `Bootstrapper`.
5. Add the `DroneSceneBootstrapper` component to it.
6. In the component menu, run **Build Spherical Drone Scene**. This creates a simple drone, target marker, camera, light, and ground plane in the scene.
7. On the generated `SphericalDroneAgent` GameObject, add:
   - `Behavior Parameters`
   - `Decision Requester`
8. Set Behavior Parameters:
   - Behavior Name: `SphericalDrone`
   - Vector Observation Space Size: `34`
   - Continuous Actions: `18`
   - Discrete Branches: `0`
9. Set Decision Requester:
   - Decision Period: `1`

See `docs/UNITY_SETUP.md` for more detail.

## Training command

From the folder containing `spherical_drone_ppo.yaml`, run:

```bash
mlagents-learn spherical_drone_ppo.yaml --run-id=spherical_drone_v0
```

Then press Play in Unity when the terminal says it is waiting for the Unity environment.

## Current limitations

This is starter code, not a finished flight controller. The physics model is simplified, and the motor/gimbal model is intentionally lightweight so that the project can get running quickly. The next steps are to tune the reward function, improve the motor model, add better failure cases, and compare learned control allocation against a baseline controller.
