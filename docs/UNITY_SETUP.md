# Unity Setup Guide

This starter is designed so you can quickly create a visible drone demo and connect it to Unity ML-Agents.

## 1. Create the Unity project

Create a new 3D Unity project. The starter scripts are not a full Unity project by themselves; they are scripts/config files to copy into a Unity project.

## 2. Install ML-Agents

In Unity:

1. Open **Window > Package Manager**.
2. Select **+ > Add package by name**.
3. Enter `com.unity.ml-agents`.
4. Add the package.

You will also need the Python-side ML-Agents package to run training.

## 3. Copy the scripts

Copy this folder into your Unity project:

```text
UnityStarter/Assets/Scripts
```

It should become:

```text
YourUnityProject/Assets/Scripts
```

## 4. Build the starter scene

1. Create a new empty scene.
2. Create an empty GameObject named `Bootstrapper`.
3. Add the `DroneSceneBootstrapper` script to `Bootstrapper`.
4. Open the three-dot/component menu on `DroneSceneBootstrapper` and run **Build Spherical Drone Scene**. You can also right-click the component title and choose the same context-menu command.

The script will create these objects in the scene:

- `SphericalDroneAgent`
- a spherical drone body
- six motor objects
- a target marker
- a ground plane
- a camera and light

## 5. Add ML-Agents components

Select the generated `SphericalDroneAgent` GameObject and add:

- `Behavior Parameters`
- `Decision Requester`

Set the Behavior Parameters as follows:

```text
Behavior Name: SphericalDrone
Vector Observation Space Size: 34
Continuous Actions: 18
Discrete Branches: 0
```

Set the Decision Requester as follows:

```text
Decision Period: 1
Take Actions Between Decisions: true
```

## 6. Train

From the `UnityStarter/configs` folder, run:

```bash
mlagents-learn spherical_drone_ppo.yaml --run-id=spherical_drone_v0
```

When the terminal says it is waiting for Unity, press Play in Unity.

## 7. Manual demo keys

The `Heuristic` method in `SixMotorDroneAgent` allows rough manual testing when the Behavior Type is set to Heuristic Only:

```text
Space       upward thrust from bottom motor
Left Shift  downward thrust from top motor
W/S         forward/backward motor thrust
A/D         left/right motor thrust
```

The `KeyboardTargetController` moves the target marker:

```text
I/K         target up/down
J/L         target left/right
U/O         target forward/backward
Q/E         target yaw left/right
```

The `MotorFailureScheduler` gives a quick failure demo:

```text
F           damage a random motor
R           repair all motors
```

## 8. Notes

This is not yet a perfect drone simulator. It is a starter digital twin with a simplified force model. The goal is to have code that can be committed to GitHub and then improved for the project.
