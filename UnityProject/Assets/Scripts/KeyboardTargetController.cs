using UnityEngine;

/// <summary>
/// Lets you move the target marker during a manual demo.
/// This is useful for showing command tracking before or after training.
/// </summary>
public class KeyboardTargetController : MonoBehaviour
{
    public SixMotorDroneAgent agent;
    public float targetMoveSpeed = 1.5f;
    public float yawMoveSpeedDeg = 60f;

    [Header("Keys")]
    public KeyCode moveUpKey = KeyCode.I;
    public KeyCode moveDownKey = KeyCode.K;
    public KeyCode moveLeftKey = KeyCode.J;
    public KeyCode moveRightKey = KeyCode.L;
    public KeyCode moveForwardKey = KeyCode.U;
    public KeyCode moveBackwardKey = KeyCode.O;
    public KeyCode yawLeftKey = KeyCode.Q;
    public KeyCode yawRightKey = KeyCode.E;

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

        Vector3 delta = Vector3.zero;

        if (Input.GetKey(moveUpKey))
        {
            delta.y += 1f;
        }

        if (Input.GetKey(moveDownKey))
        {
            delta.y -= 1f;
        }

        if (Input.GetKey(moveRightKey))
        {
            delta.x += 1f;
        }

        if (Input.GetKey(moveLeftKey))
        {
            delta.x -= 1f;
        }

        if (Input.GetKey(moveForwardKey))
        {
            delta.z += 1f;
        }

        if (Input.GetKey(moveBackwardKey))
        {
            delta.z -= 1f;
        }

        if (delta.sqrMagnitude > 0f)
        {
            agent.MoveTargetLocal(delta.normalized * targetMoveSpeed * Time.deltaTime);
        }

        if (Input.GetKey(yawLeftKey))
        {
            agent.targetYawDeg -= yawMoveSpeedDeg * Time.deltaTime;
        }

        if (Input.GetKey(yawRightKey))
        {
            agent.targetYawDeg += yawMoveSpeedDeg * Time.deltaTime;
        }
    }
}
