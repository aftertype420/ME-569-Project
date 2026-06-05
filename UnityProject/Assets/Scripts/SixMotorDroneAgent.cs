using System;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

/// <summary>
/// Unity ML-Agents controller for a six-motor spherical drone.
///
/// Behavior Parameters setup:
/// - Behavior Name: SphericalDrone
/// - Vector Observation Space Size: 34
/// - Continuous Actions: 18
/// - Discrete Branches: 0
///
/// Continuous action layout:
/// - actions[0..5]   = six motor thrust commands, normalized from [-1, 1] to [0, 1]
/// - actions[6..11]  = six pitch gimbal commands, normalized to +/- maxGimbalAngleDeg
/// - actions[12..17] = six yaw gimbal commands, normalized to +/- maxGimbalAngleDeg
///
/// This is a starter controller. The reward function and motor model should be tuned.
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class SixMotorDroneAgent : Agent
{
    public const int MotorCount = 6;
    public const int ContinuousActionCount = 18;
    public const int ObservationCount = 34;

    [Header("Motor setup")]
    public DroneMotor[] motors = new DroneMotor[MotorCount];
    public float maxGimbalAngleDeg = 65f;

    [Header("Target")]
    public Transform targetMarker;
    public Vector3 targetPositionLocal = new Vector3(0f, 2f, 0f);
    public float targetYawDeg = 0f;
    public float hoverHeight = 2f;
    public bool randomizeTargetEachEpisode = true;
    public Vector2 randomTargetXZRange = new Vector2(-2f, 2f);
    public Vector2 randomTargetYRange = new Vector2(1.5f, 3f);

    [Header("Episode settings")]
    public float episodeDurationSeconds = 20f;
    public float maxDistanceFromOrigin = 8f;
    public float crashHeight = -0.5f;

    [Header("Observation normalization")]
    public float maxSpeed = 10f;
    public float maxAngularSpeed = 12f;

    [Header("Motor failure training")]
    public bool randomizeMotorFailures = true;
    [Range(0f, 1f)]
    public float motorFailureEpisodeProbability = 0.35f;
    public int maxFailedMotors = 2;
    public Vector2 damagedMotorHealthRange = new Vector2(0f, 0.45f);

    [Header("Reward weights")]
    public float positionRewardWeight = 0.04f;
    public float velocityPenaltyWeight = 0.004f;
    public float orientationPenaltyWeight = 0.004f;
    public float angularVelocityPenaltyWeight = 0.003f;
    public float energyPenaltyWeight = 0.002f;
    public float nearTargetBonus = 0.01f;
    public float survivalReward = 0.001f;

    [Header("Debug")]
    public bool verboseWarnings = true;

    private Rigidbody rb;
    private Vector3 startLocalPosition;
    private Quaternion startLocalRotation;
    private float episodeTimer;
    private readonly float[] lastThrustCommands = new float[MotorCount];
    private readonly float[] lastPitchCommandsDeg = new float[MotorCount];
    private readonly float[] lastYawCommandsDeg = new float[MotorCount];

    public Vector3 TargetWorldPosition
    {
        get
        {
            if (transform.parent != null)
            {
                return transform.parent.TransformPoint(targetPositionLocal);
            }

            return targetPositionLocal;
        }
    }

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
        rb.useGravity = true;
        rb.maxAngularVelocity = maxAngularSpeed;

        startLocalPosition = transform.localPosition;
        startLocalRotation = transform.localRotation;

        CacheAndSortMotors();
        ResetMotorStats();
        UpdateTargetMarker();
    }

    public override void OnEpisodeBegin()
    {
        if (rb == null)
        {
            rb = GetComponent<Rigidbody>();
        }

        episodeTimer = 0f;

        transform.localPosition = new Vector3(0f, hoverHeight, 0f);
        transform.localRotation = startLocalRotation;
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        if (randomizeTargetEachEpisode)
        {
            float targetX = UnityEngine.Random.Range(randomTargetXZRange.x, randomTargetXZRange.y);
            float targetZ = UnityEngine.Random.Range(randomTargetXZRange.x, randomTargetXZRange.y);
            float targetY = UnityEngine.Random.Range(randomTargetYRange.x, randomTargetYRange.y);
            targetPositionLocal = new Vector3(targetX, targetY, targetZ);
            targetYawDeg = UnityEngine.Random.Range(-180f, 180f);
        }

        ResetMotorStats();
        ResetMotorHealth();

        if (randomizeMotorFailures)
        {
            RandomizeMotorFailuresForEpisode();
        }

        UpdateTargetMarker();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // 1-3: normalized local position
        sensor.AddObservation(ClampVector(transform.localPosition / Mathf.Max(maxDistanceFromOrigin, 0.001f)));

        // 4-6: normalized world velocity
        sensor.AddObservation(ClampVector(rb.linearVelocity / Mathf.Max(maxSpeed, 0.001f)));

        // 7-9: current up direction
        sensor.AddObservation(transform.up);

        // 10-12: current forward direction
        sensor.AddObservation(transform.forward);

        // 13-15: normalized angular velocity
        sensor.AddObservation(ClampVector(rb.angularVelocity / Mathf.Max(maxAngularSpeed, 0.001f)));

        // 16-18: normalized target position
        sensor.AddObservation(ClampVector(targetPositionLocal / Mathf.Max(maxDistanceFromOrigin, 0.001f)));

        // 19: normalized yaw error
        float yawError = Mathf.DeltaAngle(transform.eulerAngles.y, targetYawDeg) / 180f;
        sensor.AddObservation(Mathf.Clamp(yawError, -1f, 1f));

        // 20-25: motor health values
        for (int i = 0; i < MotorCount; i++)
        {
            sensor.AddObservation(GetMotorHealth(i));
        }

        // 26-31: previous thrust commands
        for (int i = 0; i < MotorCount; i++)
        {
            sensor.AddObservation(Mathf.Clamp01(lastThrustCommands[i]));
        }

        // 32-34: local target error direction, normalized
        Vector3 localTargetError = transform.InverseTransformDirection(TargetWorldPosition - transform.position);
        sensor.AddObservation(ClampVector(localTargetError / Mathf.Max(maxDistanceFromOrigin, 0.001f)));
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        ActionSegment<float> continuousActions = actions.ContinuousActions;

        if (continuousActions.Length < MotorCount)
        {
            if (verboseWarnings)
            {
                Debug.LogWarning("SixMotorDroneAgent expected at least 6 continuous actions.");
            }
            AddReward(-0.01f);
            return;
        }

        episodeTimer += Time.fixedDeltaTime;

        ApplyContinuousActions(continuousActions);
        ApplyStepReward();
        CheckTerminationConditions();
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continuousActions = actionsOut.ContinuousActions;

        for (int i = 0; i < continuousActions.Length; i++)
        {
            continuousActions[i] = -1f;
        }

        // Manual starter controls for quick demo/testing.
        // These are rough because the learning agent is the main controller.
        // Motor convention from DroneSceneBootstrapper:
        // 0 Top, 1 Bottom, 2 Left, 3 Right, 4 Front, 5 Back
        float hoverCommand = Input.GetKey(KeyCode.Space) ? 0.7f : 0.35f;
        SetActionMotorCommand(continuousActions, 1, hoverCommand); // bottom motor gives upward force

        if (Input.GetKey(KeyCode.LeftShift))
        {
            SetActionMotorCommand(continuousActions, 0, 0.7f); // top motor gives downward force
        }

        if (Input.GetKey(KeyCode.D))
        {
            SetActionMotorCommand(continuousActions, 2, 0.7f); // left motor pushes right
        }

        if (Input.GetKey(KeyCode.A))
        {
            SetActionMotorCommand(continuousActions, 3, 0.7f); // right motor pushes left
        }

        if (Input.GetKey(KeyCode.S))
        {
            SetActionMotorCommand(continuousActions, 4, 0.7f); // front motor pushes backward
        }

        if (Input.GetKey(KeyCode.W))
        {
            SetActionMotorCommand(continuousActions, 5, 0.7f); // back motor pushes forward
        }

        // Leave gimbal commands at zero if the action space includes them.
        for (int i = MotorCount; i < continuousActions.Length; i++)
        {
            continuousActions[i] = 0f;
        }
    }

    public void MoveTargetLocal(Vector3 localDelta)
    {
        targetPositionLocal += localDelta;
        UpdateTargetMarker();
    }

    public void SetTargetLocal(Vector3 newTargetLocal)
    {
        targetPositionLocal = newTargetLocal;
        UpdateTargetMarker();
    }

    public void SetAllMotorHealth(float health)
    {
        if (motors == null)
        {
            return;
        }

        for (int i = 0; i < motors.Length; i++)
        {
            if (motors[i] != null)
            {
                motors[i].SetHealth(health);
            }
        }
    }

    public void FailMotor(int motorIndex, float failedHealth = 0f)
    {
        DroneMotor motor = GetMotor(motorIndex);
        if (motor != null)
        {
            motor.SetHealth(failedHealth);
        }
    }

    private void ApplyContinuousActions(ActionSegment<float> continuousActions)
    {
        float dt = Time.fixedDeltaTime;

        for (int i = 0; i < MotorCount; i++)
        {
            float thrustCommand01 = NormalizedMinusOneToOneToZeroToOne(continuousActions[i]);
            float pitchDeg = 0f;
            float yawDeg = 0f;

            if (continuousActions.Length >= ContinuousActionCount)
            {
                pitchDeg = Mathf.Clamp(continuousActions[MotorCount + i], -1f, 1f) * maxGimbalAngleDeg;
                yawDeg = Mathf.Clamp(continuousActions[MotorCount * 2 + i], -1f, 1f) * maxGimbalAngleDeg;
            }

            lastThrustCommands[i] = thrustCommand01;
            lastPitchCommandsDeg[i] = pitchDeg;
            lastYawCommandsDeg[i] = yawDeg;

            DroneMotor motor = GetMotor(i);
            if (motor != null)
            {
                motor.ApplyMotor(rb, thrustCommand01, pitchDeg, yawDeg, dt);
            }
        }
    }

    private void ApplyStepReward()
    {
        Vector3 targetWorld = TargetWorldPosition;
        float positionError = Vector3.Distance(transform.position, targetWorld);
        float positionScore = Mathf.Exp(-positionError);

        float velocityPenalty = Mathf.Clamp01(rb.linearVelocity.magnitude / Mathf.Max(maxSpeed, 0.001f));
        float orientationPenalty = Vector3.Angle(transform.up, Vector3.up) / 180f;
        float angularPenalty = Mathf.Clamp01(rb.angularVelocity.magnitude / Mathf.Max(maxAngularSpeed, 0.001f));
        float energyPenalty = AverageSquaredThrustCommand();

        AddReward(survivalReward);
        AddReward(positionRewardWeight * positionScore);
        AddReward(-velocityPenaltyWeight * velocityPenalty);
        AddReward(-orientationPenaltyWeight * orientationPenalty);
        AddReward(-angularVelocityPenaltyWeight * angularPenalty);
        AddReward(-energyPenaltyWeight * energyPenalty);

        if (positionError < 0.35f && rb.linearVelocity.magnitude < 0.5f)
        {
            AddReward(nearTargetBonus);
        }
    }

    private void CheckTerminationConditions()
    {
        if (transform.localPosition.y < crashHeight)
        {
            AddReward(-1f);
            EndEpisode();
            return;
        }

        if (transform.localPosition.magnitude > maxDistanceFromOrigin)
        {
            AddReward(-0.5f);
            EndEpisode();
            return;
        }

        if (episodeTimer >= episodeDurationSeconds)
        {
            EndEpisode();
        }
    }

    private void CacheAndSortMotors()
    {
        DroneMotor[] childMotors = GetComponentsInChildren<DroneMotor>();
        if (childMotors != null && childMotors.Length > 0)
        {
            motors = childMotors;
        }

        if (motors == null)
        {
            motors = new DroneMotor[MotorCount];
        }

        Array.Sort(motors, (a, b) =>
        {
            int ai = a == null ? int.MaxValue : a.motorIndex;
            int bi = b == null ? int.MaxValue : b.motorIndex;
            return ai.CompareTo(bi);
        });

        if (motors.Length != MotorCount && verboseWarnings)
        {
            Debug.LogWarning("Expected exactly 6 DroneMotor components. Found " + motors.Length + ".");
        }
    }

    private void ResetMotorStats()
    {
        for (int i = 0; i < MotorCount; i++)
        {
            lastThrustCommands[i] = 0f;
            lastPitchCommandsDeg[i] = 0f;
            lastYawCommandsDeg[i] = 0f;

            DroneMotor motor = GetMotor(i);
            if (motor != null)
            {
                motor.ResetEpisodeStats();
            }
        }
    }

    private void ResetMotorHealth()
    {
        for (int i = 0; i < MotorCount; i++)
        {
            DroneMotor motor = GetMotor(i);
            if (motor != null)
            {
                motor.SetHealth(1f);
            }
        }
    }

    private void RandomizeMotorFailuresForEpisode()
    {
        if (UnityEngine.Random.value > motorFailureEpisodeProbability)
        {
            return;
        }

        int failureCount = UnityEngine.Random.Range(1, Mathf.Max(1, maxFailedMotors) + 1);

        for (int j = 0; j < failureCount; j++)
        {
            int motorIndex = UnityEngine.Random.Range(0, MotorCount);
            float damagedHealth = UnityEngine.Random.Range(damagedMotorHealthRange.x, damagedMotorHealthRange.y);
            FailMotor(motorIndex, damagedHealth);
        }
    }

    private DroneMotor GetMotor(int motorIndex)
    {
        if (motors == null || motorIndex < 0 || motorIndex >= motors.Length)
        {
            return null;
        }

        return motors[motorIndex];
    }

    private float GetMotorHealth(int motorIndex)
    {
        DroneMotor motor = GetMotor(motorIndex);
        if (motor == null)
        {
            return 0f;
        }

        return Mathf.Clamp01(motor.health);
    }

    private float AverageSquaredThrustCommand()
    {
        float sum = 0f;
        for (int i = 0; i < MotorCount; i++)
        {
            sum += lastThrustCommands[i] * lastThrustCommands[i];
        }

        return sum / MotorCount;
    }

    private void UpdateTargetMarker()
    {
        if (targetMarker != null)
        {
            targetMarker.position = TargetWorldPosition;
        }
    }

    private static Vector3 ClampVector(Vector3 value)
    {
        return Vector3.ClampMagnitude(value, 1f);
    }

    private static float NormalizedMinusOneToOneToZeroToOne(float value)
    {
        return Mathf.Clamp01((Mathf.Clamp(value, -1f, 1f) + 1f) * 0.5f);
    }

    private static void SetActionMotorCommand(ActionSegment<float> continuousActions, int motorIndex, float command01)
    {
        if (motorIndex < 0 || motorIndex >= continuousActions.Length)
        {
            return;
        }

        continuousActions[motorIndex] = Mathf.Clamp(command01, 0f, 1f) * 2f - 1f;
    }

    private void OnDrawGizmos()
    {
        Gizmos.DrawWireSphere(TargetWorldPosition, 0.2f);
        Gizmos.DrawLine(transform.position, TargetWorldPosition);
    }
}
