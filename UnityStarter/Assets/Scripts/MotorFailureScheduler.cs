using UnityEngine;

/// <summary>
/// Manual motor-failure utility for demos.
/// Press F to damage a random motor.
/// Press R to repair all motors.
/// </summary>
public class MotorFailureScheduler : MonoBehaviour
{
    public SixMotorDroneAgent agent;
    public KeyCode failRandomMotorKey = KeyCode.F;
    public KeyCode repairAllMotorsKey = KeyCode.R;
    public Vector2 manualFailureHealthRange = new Vector2(0f, 0.25f);

    private void Reset()
    {
        agent = GetComponent<SixMotorDroneAgent>();
    }

    private void Update()
    {
        if (agent == null)
        {
            return;
        }

        if (Input.GetKeyDown(failRandomMotorKey))
        {
            FailRandomMotor();
        }

        if (Input.GetKeyDown(repairAllMotorsKey))
        {
            RepairAllMotors();
        }
    }

    [ContextMenu("Fail Random Motor")]
    public void FailRandomMotor()
    {
        int motorIndex = Random.Range(0, SixMotorDroneAgent.MotorCount);
        float damagedHealth = Random.Range(manualFailureHealthRange.x, manualFailureHealthRange.y);
        agent.FailMotor(motorIndex, damagedHealth);
        Debug.Log("Damaged motor " + motorIndex + " to health " + damagedHealth.ToString("F2"));
    }

    [ContextMenu("Repair All Motors")]
    public void RepairAllMotors()
    {
        agent.SetAllMotorHealth(1f);
        Debug.Log("All motors repaired.");
    }
}
