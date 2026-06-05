using UnityEngine;

/// <summary>
/// Simple motor/thruster model for the spherical drone.
/// Attach one DroneMotor component to each motor GameObject.
///
/// Important convention:
/// - transform.up is treated as the motor's default force direction.
/// - The optional pitch/yaw gimbal commands rotate that direction.
/// - The applied force is command01 * maxThrustNewtons * health.
/// </summary>
public class DroneMotor : MonoBehaviour
{
    [Header("Motor identity")]
    public string motorName = "Motor";
    public int motorIndex = 0;

    [Header("Thrust model")]
    [Tooltip("Maximum force produced by this motor when health = 1 and command = 1.")]
    public float maxThrustNewtons = 15f;

    [Tooltip("Motor health from 0 to 1. A failed motor should be set near 0.")]
    [Range(0f, 1f)]
    public float health = 1f;

    [Header("Runtime debug values")]
    [Range(0f, 1f)]
    public float lastCommand01 = 0f;
    public float lastPitchDeg = 0f;
    public float lastYawDeg = 0f;
    public float lastEffectiveThrust = 0f;
    public float energyProxyThisEpisode = 0f;

    [Header("Debug drawing")]
    public bool drawDebugForce = true;
    public float debugForceScale = 0.08f;

    public Vector3 BaseThrustDirectionWorld
    {
        get { return transform.up.normalized; }
    }

    /// <summary>
    /// Returns the motor force direction after applying a simplified two-axis gimbal.
    /// This is not a detailed mechanical gimbal model; it is a useful starter approximation.
    /// </summary>
    public Vector3 GetWorldThrustDirection(float pitchDeg, float yawDeg)
    {
        Quaternion pitchRotation = Quaternion.AngleAxis(pitchDeg, transform.right);
        Quaternion yawRotation = Quaternion.AngleAxis(yawDeg, transform.forward);
        Vector3 direction = yawRotation * pitchRotation * transform.up;
        return direction.normalized;
    }

    /// <summary>
    /// Applies this motor's force to the drone rigidbody.
    /// Returns the effective thrust in Newtons.
    /// </summary>
    public float ApplyMotor(Rigidbody droneRigidbody, float command01, float pitchDeg, float yawDeg, float deltaTime)
    {
        if (droneRigidbody == null)
        {
            return 0f;
        }

        lastCommand01 = Mathf.Clamp01(command01);
        lastPitchDeg = pitchDeg;
        lastYawDeg = yawDeg;

        float clampedHealth = Mathf.Clamp01(health);
        lastEffectiveThrust = lastCommand01 * maxThrustNewtons * clampedHealth;

        Vector3 force = GetWorldThrustDirection(lastPitchDeg, lastYawDeg) * lastEffectiveThrust;
        droneRigidbody.AddForceAtPosition(force, transform.position, ForceMode.Force);

        // Simple normalized energy proxy. This is not battery physics.
        // It is useful for comparing motor-use strategies.
        energyProxyThisEpisode += lastCommand01 * lastCommand01 * Mathf.Max(deltaTime, 0f);

        return lastEffectiveThrust;
    }

    public void SetHealth(float newHealth)
    {
        health = Mathf.Clamp01(newHealth);
    }

    public void ResetEpisodeStats()
    {
        lastCommand01 = 0f;
        lastPitchDeg = 0f;
        lastYawDeg = 0f;
        lastEffectiveThrust = 0f;
        energyProxyThisEpisode = 0f;
    }

    private void OnDrawGizmos()
    {
        if (!drawDebugForce)
        {
            return;
        }

        Vector3 forceDirection = GetWorldThrustDirection(lastPitchDeg, lastYawDeg);
        float drawLength = Mathf.Max(0.2f, lastEffectiveThrust * debugForceScale);

        Gizmos.DrawLine(transform.position, transform.position + forceDirection * drawLength);
        Gizmos.DrawWireSphere(transform.position, 0.04f);
    }
}
