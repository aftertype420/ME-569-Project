# Finish Demo Guide

This patch turns the starter project into a class-project-ready demo:

1. A six-motor spherical drone digital twin in Unity.
2. A baseline control allocator for comparison.
3. A machine-learning action/observation/reward setup using Unity ML-Agents.
4. Motor-failure test cases.
5. CSV telemetry logging for report plots.

## Unity settings

On `SphericalDroneAgent`:

- Behavior Name: `SphericalDrone`
- Vector Observation Space Size: `34`
- Continuous Actions: `18`
- Discrete Branches: `0`
- Manual demo Behavior Type: `Heuristic Only`
- Training Behavior Type: `Default`

For a first demo, uncheck `Randomize Motor Failures`. Use the `F` key to manually fail a motor.

## Demo controls

- `I/K`: move target up/down
- `J/L`: move target left/right
- `U/O`: move target forward/backward
- `Q/E`: change target yaw
- `F`: damage a random motor
- `R`: repair all motors
- `H`: hide/show HUD

## Training command

From the repository root, run:

```powershell
mlagents-learn .\ml-agents-configs\spherical_drone_ppo.yaml --run-id=spherical_drone_v1
```

When the terminal says to start training, press Play in Unity.

## What to compare in the final report

Run three short tests and use screenshots/telemetry:

1. Baseline hover without failures.
2. Baseline hover with one failed motor.
3. ML-Agents training run, showing reward trend or describing partial training if the model is still in progress.

Telemetry CSV files are saved to Unity's `Application.persistentDataPath`. Select the `DroneTelemetryLogger` component and use `Print Log Folder` to see the exact folder.
