# Digital Twin and Machine-Learning Control Allocation for a Six-Motor Spherical Drone

ME 569 C: Data-Driven Control final project repository.

This project develops a Unity-based digital twin of a six-motor spherical drone and uses Unity ML-Agents to train and test a machine-learning controller for data-driven control allocation. The drone is modeled as a spherical rigid body with six independently controlled motors mounted around the outside of the sphere. The control objective is to allocate motor thrust for hover, target tracking, energy-aware motor usage, and partial motor-failure recovery.

## Current status

The project now includes a working Unity demonstration and preliminary machine-learning results.

Implemented:

- Unity digital twin scene with a spherical drone body and six external motors.
- Physics-based motor thrust and simplified gimbal command logic.
- Unity ML-Agents `Agent` implementation with:
  - Behavior name: `SphericalDrone`
  - Vector observation size: `34`
  - Continuous action size: `18`
  - Discrete actions: `0`
- Baseline heuristic control allocator for target tracking.
- Motor-failure testing with reduced motor-health values.
- Telemetry logger for position error, speed, angular speed, motor health, motor commands, thrust, and energy proxy.
- Demo HUD for visualizing controller performance during runtime.
- PPO training configuration for Unity ML-Agents.
- A preliminary trained ONNX policy from a 150,110-step ML-Agents training run.
- Baseline and ML telemetry CSV files for normal and motor-failure tests.
- Isaac Lab skeleton for future robotics-research extension.

## Project direction

- **Long-term robotics target:** Isaac Lab, because it is better suited for GPU-accelerated robot learning, reinforcement learning, imitation learning, and future sim-to-real workflows.
- **Current completed implementation:** Unity + ML-Agents, because Unity made it faster to build, visualize, debug, demonstrate, and train a class-project digital twin.
- **Machine learning requirement:** Machine learning is not a backup feature. The Unity ML-Agents policy was trained and tested, and future work would improve the learned controller rather than remove it.

## Repository layout

```text
.
├── UnityProject/
│   ├── Assets/
│   │   ├── Scenes/
│   │   │   └── SphericalDroneDemo.unity
│   │   ├── Scripts/
│   │   │   ├── DroneMotor.cs
│   │   │   ├── SixMotorDroneAgent.cs
│   │   │   ├── DroneSceneBootstrapper.cs
│   │   │   ├── KeyboardTargetController.cs
│   │   │   ├── MotorFailureScheduler.cs
│   │   │   ├── DroneTelemetryLogger.cs
│   │   │   └── DroneDemoHUD.cs
│   │   └── ML-Agents/
│   │       └── Models/
│   │           └── SphericalDrone_hover_v1.onnx
│   └── Packages/
├── UnityStarter/
├── isaac_lab_skeleton/
├── ml-agents-configs/
├── results/
├── docs/
├── .gitignore
└── README.md
```

## How to run the Unity demo

1. Open `UnityProject` in Unity 6.
2. Open the scene:

   ```text
   Assets/Scenes/SphericalDroneDemo.unity
   ```

3. Select `SphericalDroneAgent`.
4. For baseline/demo mode, set the ML-Agents `Behavior Parameters` component to:

   ```text
   Behavior Name: SphericalDrone
   Behavior Type: Heuristic Only
   Vector Observation Space Size: 34
   Continuous Actions: 18
   Discrete Branches: 0
   ```

5. Press Play.

Useful demo controls:

```text
I / K     move target up/down
J / L     move target left/right
U / O     move target forward/backward
Q / E     rotate target yaw
F         damage a random motor
R         repair all motors
H         hide/show HUD
```

## How to run the trained ML policy

1. Open the Unity scene.
2. Select `SphericalDroneAgent`.
3. In `Behavior Parameters`, set:

   ```text
   Behavior Type: Inference Only
   Model: SphericalDrone_hover_v1
   ```

4. Make sure `Drone Telemetry Logger > Log On Play` is checked if you want a new CSV file.
5. Press Play and let the policy run without manual keyboard control.

## Training command

The quick PPO configuration used for the preliminary policy is:

```text
ml-agents-configs/spherical_drone_ppo_quick.yaml
```

From the repository root, activate the ML-Agents Python environment and run:

```bash
mlagents-learn ./ml-agents-configs/spherical_drone_ppo_quick.yaml --run-id=spherical_drone_hover_v1 --force
```

Then press Play in Unity when the terminal says it is waiting for the Unity environment.

## Results summary

Telemetry files are stored in `results/`.

| Run | Description |
|---|---|
| `baseline_normal_telemetry.csv` | Baseline allocator extended/nominal test run |
| `baseline_fault_telemetry.csv` | Baseline allocator with motor-failure test |
| `ml_normal_telemetry.csv` | Trained ML policy nominal test |
| `ml_fault_telemetry.csv` | Trained ML policy with motor-failure test |

Preliminary results show that the baseline allocator currently tracks the target with lower average position error and lower mean thrust usage than the first trained ML policy. The ML policy completed the full training and inference pipeline, but it still needs better reward shaping, curriculum training, and constrained allocation before it can outperform the baseline.

## Current limitations

This is a preliminary research prototype, not a finished flight controller. Current limitations include:

- Simplified drone physics and simplified motor/gimbal model.
- No calibrated aerodynamic model for the spherical frame.
- No real battery, current, or propeller-efficiency model.
- Preliminary reward function that does not yet produce an efficient learned controller.
- Short training horizon for a difficult nonlinear flight-control problem.
- Fault handling is based on simplified motor-health scaling.
- Isaac Lab version is only a future skeleton, not a complete environment.

## Recommended next steps

- Add curriculum learning: hover first, then target tracking, then motor failures.
- Reduce the action space or constrain the neural network output through a model-based allocation layer.
- Improve reward weights for stability, low angular velocity, smooth commands, and energy use.
- Add more realistic actuator limits and rate limits.
- Compare additional training runs against the baseline telemetry.
- Extend the project to Isaac Lab for faster parallel robotics training and better sim-to-real workflows.

## Attribution

Initial starter code structure and debugging support were developed with ChatGPT assistance and then modified, tested, trained, and evaluated by the author. Literature, Unity ML-Agents documentation, NVIDIA Isaac Lab documentation, and course feedback informed the final project direction.
