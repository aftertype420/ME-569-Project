# Control Design Notes

## Project concept

The project is a digital twin of a six-motor spherical drone. The six motors are arranged around the spherical body like the faces of a dice:

```text
0 Top
1 Bottom
2 Left
3 Right
4 Front
5 Back
```

Each motor is independently commanded. In the starter Unity version, each motor has:

- a thrust command from 0 to 1
- a simplified pitch gimbal command
- a simplified yaw gimbal command
- a health value from 0 to 1

## Machine-learning objective

The controller should learn control allocation. Instead of using all six motors equally, it should learn which motors are useful for the current task:

- hover
- move up/down
- move left/right
- move forward/backward
- rotate
- recover from a weakened or failed motor

## Observation vector

The current starter observation size is 34:

| Observation group | Size |
|---|---:|
| Drone local position | 3 |
| Drone velocity | 3 |
| Drone up direction | 3 |
| Drone forward direction | 3 |
| Drone angular velocity | 3 |
| Target local position | 3 |
| Target yaw error | 1 |
| Motor health values | 6 |
| Previous thrust commands | 6 |
| Local target error | 3 |
| **Total** | **34** |

## Action vector

The current starter action size is 18:

| Action group | Size |
|---|---:|
| Motor thrust commands | 6 |
| Motor pitch gimbal commands | 6 |
| Motor yaw gimbal commands | 6 |
| **Total** | **18** |

The ML-Agents network outputs values in `[-1, 1]`. The starter code maps thrust actions to `[0, 1]` and maps pitch/yaw actions to `+/- maxGimbalAngleDeg`.

## Reward idea

The starter reward function includes:

- positive reward for staying close to the target
- small survival reward
- penalty for large velocity
- penalty for tilted orientation
- penalty for angular velocity
- penalty for high motor usage
- bonus for being very close to the target with low speed
- crash penalty
- penalty for leaving the allowed flight region

## Next improvements

Recommended next steps:

1. Tune motor thrust so the drone can hover more easily.
2. Tune the reward function so the agent first learns stable hover.
3. Train without random motor failures first.
4. Add one-motor failure after hover works.
5. Add two-motor failure after one-motor failure works.
6. Compare the learned controller against a basic hand-coded controller.
7. Track metrics such as position error, velocity error, motor usage, and energy proxy.
8. Move the concept toward Isaac Lab after the Unity prototype is working.
