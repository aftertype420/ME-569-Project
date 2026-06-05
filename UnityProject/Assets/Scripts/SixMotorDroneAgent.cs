using System;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

/// <summary>
/// Unity ML-Agents controller for a six-motor spherical drone.
///
/// Final-project version features:
/// - Six independently controlled motors.
/// - Continuous ML action space for thrust and simplified two-axis gimbal commands.
/// - Fault-tolerant observations: each motor health value is visible to the policy.
/// - Reward terms for position tracking, stability, energy efficiency, survival, and motor failures.
/// - A baseline heuristic allocator that can hover/track the target before a trained model exists.
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
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class SixMotorDroneAgent : Agent
{
    public enum HeuristicControlMode
    {
        ManualDirect,
        BaselineAllocator
    }

    public const int MotorCount = 6;
    public const int ContinuousActionCount = 18;
    public const int ObservationCount = 34;

    [Header("Motor setup")]
    public DroneMotor[] motors = new DroneMotor[MotorCount];
    public float maxGimbalAngleDeg = 65f;

    [Header("Heuristic / demo control")]
    [Tooltip("ManualDirect maps keyboard keys to motors. BaselineAllocator tracks the target using a simple force allocator.")]
    public HeuristicControlMode heuristicControlMode = HeuristicControlMode.BaselineAllocator;
    [Tooltip("Applies a small direct torque during baseline heuristic mode to approximate a local attitude/gimbal stabilizer. Turn off during pure physics tests.")]
    public bool baselineTorqueAssist = true;
    public float positionKp = 3.0f;
    public float velocityKd = 2.2f;
    public float maxDesiredAcceleration = 16f;
    public float attitudeKp = 10f;
    public float attitudeKd = 2.5f;
    public float yawKp = 1.5f;
    public float maxAssistTorque = 8f;

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
    public float failureRecoveryBonus = 0.004f;

    [Header("Debug")]
    public bool verboseWarnings = true;

    private Rigidbody rb;
    private Quaternion startLocalRotation;
    private float episodeTimer;
    private int episodeIndex;
    private bool pendingBaselineTorqueAssist;

    private readonly float[] lastThrustCommands = new float[MotorCount];
    private readonly float[] lastPitchCommandsDeg = new float[MotorCount];
    private readonly float[] lastYawCommandsDeg = new float[MotorCount];

    public int EpisodeIndex => episodeIndex;
    public float EpisodeTimer => episodeTimer;
    public float CurrentPositionError => Vector3.Distance(transform.position, TargetWorldPosition);
    public float CurrentSpeed => rb == null ? 0f : rb.linearVelocity.magnitude;
    public float CurrentAngularSpeed => rb == null ? 0f : rb.angularVelocity.magnitude;
    public float CurrentEnergyProxy => SumMotorEnergyProxy();
    public float AverageMotorHealth => ComputeAverageMotorHealth();
    public float AverageThrustCommand => ComputeAverageThrustCommand();

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

        episodeIndex++;
        episodeTimer = 0f;
        pendingBaselineTorqueAssist = false;

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

        if (pendingBaselineTorqueAssist && baselineTorqueAssist)
        {
            ApplyBaselineAttitudeTorqueAssist();
        }
        pendingBaselineTorqueAssist = false;

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

        if (heuristicControlMode == HeuristicControlMode.BaselineAllocator)
        {
            BaselineAllocatorHeuristic(continuousActions);
            pendingBaselineTorqueAssist = true;
            return;
        }

        ManualDirectHeuristic(continuousActions);
    }

    private void ManualDirectHeuristic(ActionSegment<float> continuousActions)
    {
        // Motor convention from DroneSceneBootstrapper:
        // 0 Top, 1 Bottom, 2 Left, 3 Right, 4 Front, 5 Back
        float hoverCommand = Input.GetKey(KeyCode.Space) ? 0.70f : 0.35f;
        SetActionMotorCommand(continuousActions, 1, hoverCommand); // bottom motor gives upward force

        if (Input.GetKey(KeyCode.LeftShift))
        {
            SetActionMotorCommand(continuousActions, 0, 0.70f); // top motor gives downward force
        }

        if (Input.GetKey(KeyCode.D))
        {
            SetActionMotorCommand(continuousActions, 2, 0.70f); // left motor pushes right
        }

        if (Input.GetKey(KeyCode.A))
        {
            SetActionMotorCommand(continuousActions, 3, 0.70f); // right motor pushes left
        }

        if (Input.GetKey(KeyCode.S))
        {
            SetActionMotorCommand(continuousActions, 4, 0.70f); // front motor pushes backward
        }

        if (Input.GetKey(KeyCode.W))
        {
            SetActionMotorCommand(continuousActions, 5, 0.70f); // back motor pushes forward
        }

        for (int i = MotorCount; i < continuousActions.Length; i++)
        {
            continuousActions[i] = 0f;
        }
    }

    /// <summary>
    /// Simple baseline controller used for comparison with the learned controller.
    /// It computes a desired world force, then allocates that force to motors that can point toward it.
    /// This gives a clear control-allocation baseline that automatically avoids weak/failed motors.
    /// </summary>
    private void BaselineAllocatorHeuristic(ActionSegment<float> continuousActions)
    {
        if (rb == null)
        {
            rb = GetComponent<Rigidbody>();
        }

        Vector3 positionError = TargetWorldPosition - transform.position;
        Vector3 desiredAcceleration = positionKp * positionError - velocityKd * rb.linearVelocity;
        desiredAcceleration = Vector3.ClampMagnitude(desiredAcceleration, maxDesiredAcceleration);

        // Non-gravity force required from the motors.
        Vector3 desiredWorldForce = rb.mass * (desiredAcceleration - Physics.gravity);
        AllocateDesiredForceToMotorActions(desiredWorldForce, continuousActions);
    }

    private void AllocateDesiredForceToMotorActions(Vector3 desiredWorldForce, ActionSegment<float> continuousActions)
    {
        float desiredMagnitude = desiredWorldForce.magnitude;
        if (desiredMagnitude < 0.001f)
        {
            for (int i = 0; i < MotorCount; i++)
            {
                SetActionMotorCommand(continuousActions, i, 0f);
                SetActionGimbalCommand(continuousActions, i, 0f, 0f);
            }
            return;
        }

        Vector3 desiredDirection = desiredWorldForce / desiredMagnitude;
        float maxGimbalRad = Mathf.Deg2Rad * Mathf.Max(0f, maxGimbalAngleDeg);

        float[] capacityAlongDesired = new float[MotorCount];
        Vector3[] selectedDirections = new Vector3[MotorCount];
        float totalCapacityAlongDesired = 0f;

        for (int i = 0; i < MotorCount; i++)
        {
            DroneMotor motor = GetMotor(i);
            if (motor == null || motor.health <= 0.001f || motor.maxThrustNewtons <= 0.001f)
            {
                capacityAlongDesired[i] = 0f;
                selectedDirections[i] = Vector3.zero;
                continue;
            }

            Vector3 baseDirection = motor.BaseThrustDirectionWorld;
            Vector3 selectedDirection = Vector3.RotateTowards(baseDirection, desiredDirection, maxGimbalRad, 0f).normalized;
            float alignment = Mathf.Max(0f, Vector3.Dot(selectedDirection, desiredDirection));

            float capacity = motor.maxThrustNewtons * motor.health * alignment;
            capacityAlongDesired[i] = capacity;
            selectedDirections[i] = selectedDirection;
            totalCapacityAlongDesired += capacity;
        }

        if (totalCapacityAlongDesired <= 0.001f)
        {
            return;
        }

        for (int i = 0; i < MotorCount; i++)
        {
            DroneMotor motor = GetMotor(i);
            if (motor == null || capacityAlongDesired[i] <= 0.001f)
            {
                SetActionMotorCommand(continuousActions, i, 0f);
                SetActionGimbalCommand(continuousActions, i, 0f, 0f);
                continue;
            }

            float shareAlongDesired = desiredMagnitude * (capacityAlongDesired[i] / totalCapacityAlongDesired);
            float alignment = Mathf.Max(0.001f, Vector3.Dot(selectedDirections[i], desiredDirection));
            float requiredThrust = shareAlongDesired / alignment;
            float command01 = requiredThrust / Mathf.Max(0.001f, motor.maxThrustNewtons * motor.health);

            Vector2 pitchYaw = WorldDirectionToPitchYawDeg(motor, selectedDirections[i]);
            SetActionMotorCommand(continuousActions, i, Mathf.Clamp01(command01));
            SetActionGimbalCommand(continuousActions, i, pitchYaw.x, pitchYaw.y);
        }
    }

    private Vector2 WorldDirectionToPitchYawDeg(DroneMotor motor, Vector3 worldDirection)
    {
        if (motor == null || worldDirection.sqrMagnitude < 0.001f)
        {
            return Vector2.zero;
        }

        Vector3 localDirection = motor.transform.InverseTransformDirection(worldDirection.normalized);
        localDirection.Normalize();

        // Matches DroneMotor.GetWorldThrustDirection: yaw(z) * pitch(x) * local up.
        float pitchDeg = Mathf.Asin(Mathf.Clamp(localDirection.z, -1f, 1f)) * Mathf.Rad2Deg;
        float yawDeg = Mathf.Atan2(-localDirection.x, localDirection.y) * Mathf.Rad2Deg;

        pitchDeg = Mathf.Clamp(pitchDeg, -maxGimbalAngleDeg, maxGimbalAngleDeg);
        yawDeg = Mathf.Clamp(yawDeg, -maxGimbalAngleDeg, maxGimbalAngleDeg);

        return new Vector2(pitchDeg, yawDeg);
    }

    private void ApplyBaselineAttitudeTorqueAssist()
    {
        if (rb == null)
        {
            return;
        }

        Vector3 tiltAxis = Vector3.Cross(transform.up, Vector3.up);
        Vector3 attitudeTorque = tiltAxis * attitudeKp;
        Vector3 dampingTorque = -rb.angularVelocity * attitudeKd;
        float yawErrorDeg = Mathf.DeltaAngle(transform.eulerAngles.y, targetYawDeg);
        Vector3 yawTorque = Vector3.up * (yawErrorDeg * Mathf.Deg2Rad * yawKp);

        Vector3 totalTorque = attitudeTorque + dampingTorque + yawTorque;
        totalTorque = Vector3.ClampMagnitude(totalTorque, maxAssistTorque);
        rb.AddTorque(totalTorque, ForceMode.Force);
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

    public DroneMotor GetMotorByIndex(int motorIndex)
    {
        return GetMotor(motorIndex);
    }

    public float GetLastThrustCommand(int motorIndex)
    {
        if (motorIndex < 0 || motorIndex >= lastThrustCommands.Length)
        {
            return 0f;
        }

        return lastThrustCommands[motorIndex];
    }

    public float GetLastPitchCommandDeg(int motorIndex)
    {
        if (motorIndex < 0 || motorIndex >= lastPitchCommandsDeg.Length)
        {
            return 0f;
        }

        return lastPitchCommandsDeg[motorIndex];
    }

    public float GetLastYawCommandDeg(int motorIndex)
    {
        if (motorIndex < 0 || motorIndex >= lastYawCommandsDeg.Length)
        {
            return 0f;
        }

        return lastYawCommandsDeg[motorIndex];
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
        float positionError = CurrentPositionError;
        float positionScore = Mathf.Exp(-positionError);

        float velocityPenalty = Mathf.Clamp01(rb.linearVelocity.magnitude / Mathf.Max(maxSpeed, 0.001f));
        float orientationPenalty = Vector3.Angle(transform.up, Vector3.up) / 180f;
        float angularPenalty = Mathf.Clamp01(rb.angularVelocity.magnitude / Mathf.Max(maxAngularSpeed, 0.001f));
        float energyPenalty = AverageSquaredThrustCommand();
        int damagedMotorCount = CountDamagedMotors();

        AddReward(survivalReward);
        AddReward(positionRewardWeight * positionScore);
        AddReward(-velocityPenaltyWeight * velocityPenalty);
        AddReward(-orientationPenaltyWeight * orientationPenalty);
        AddReward(-angularVelocityPenaltyWeight * angularPenalty);
        AddReward(-energyPenaltyWeight * energyPenalty);

        if (positionError < 0.35f && rb.linearVelocity.magnitude < 0.5f)
        {
            AddReward(nearTargetBonus);

            if (damagedMotorCount > 0)
            {
                AddReward(failureRecoveryBonus * damagedMotorCount);
            }
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

    private int CountDamagedMotors()
    {
        int count = 0;
        for (int i = 0; i < MotorCount; i++)
        {
            if (GetMotorHealth(i) < 0.95f)
            {
                count++;
            }
        }

        return count;
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

    private float ComputeAverageThrustCommand()
    {
        float sum = 0f;
        for (int i = 0; i < MotorCount; i++)
        {
            sum += lastThrustCommands[i];
        }

        return sum / MotorCount;
    }

    private float ComputeAverageMotorHealth()
    {
        float sum = 0f;
        for (int i = 0; i < MotorCount; i++)
        {
            sum += GetMotorHealth(i);
        }

        return sum / MotorCount;
    }

    private float SumMotorEnergyProxy()
    {
        float sum = 0f;
        for (int i = 0; i < MotorCount; i++)
        {
            DroneMotor motor = GetMotor(i);
            if (motor != null)
            {
                sum += motor.energyProxyThisEpisode;
            }
        }

        return sum;
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

    private void SetActionGimbalCommand(ActionSegment<float> continuousActions, int motorIndex, float pitchDeg, float yawDeg)
    {
        if (continuousActions.Length < ContinuousActionCount)
        {
            return;
        }

        continuousActions[MotorCount + motorIndex] = Mathf.Clamp(pitchDeg / Mathf.Max(maxGimbalAngleDeg, 0.001f), -1f, 1f);
        continuousActions[MotorCount * 2 + motorIndex] = Mathf.Clamp(yawDeg / Mathf.Max(maxGimbalAngleDeg, 0.001f), -1f, 1f);
    }

    private void OnDrawGizmos()
    {
        Gizmos.DrawWireSphere(TargetWorldPosition, 0.2f);
        Gizmos.DrawLine(transform.position, TargetWorldPosition);
    }
}
