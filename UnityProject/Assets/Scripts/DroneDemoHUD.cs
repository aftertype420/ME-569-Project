using UnityEngine;

/// <summary>
/// Lightweight on-screen status panel for demos and screen recordings.
/// Attach this to SphericalDroneAgent.
/// </summary>
public class DroneDemoHUD : MonoBehaviour
{
    public SixMotorDroneAgent agent;
    public bool showHud = true;
    public KeyCode toggleHudKey = KeyCode.H;

    private GUIStyle labelStyle;
    private GUIStyle boxStyle;

    private void Reset()
    {
        agent = GetComponent<SixMotorDroneAgent>();
    }

    private void Awake()
    {
        if (agent == null)
        {
            agent = GetComponent<SixMotorDroneAgent>();
        }
    }

    private void Update()
    {
        if (Input.GetKeyDown(toggleHudKey))
        {
            showHud = !showHud;
        }
    }

    private void OnGUI()
    {
        if (!showHud || agent == null)
        {
            return;
        }

        EnsureStyles();

        GUILayout.BeginArea(new Rect(12f, 12f, 420f, 290f), boxStyle);
        GUILayout.Label("Six-Motor Spherical Drone Digital Twin", labelStyle);
        GUILayout.Space(6f);
        GUILayout.Label("Mode: " + agent.heuristicControlMode, labelStyle);
        GUILayout.Label("Episode: " + agent.EpisodeIndex + "   t = " + agent.EpisodeTimer.ToString("F1") + " s", labelStyle);
        GUILayout.Label("Position error: " + agent.CurrentPositionError.ToString("F2") + " m", labelStyle);
        GUILayout.Label("Speed: " + agent.CurrentSpeed.ToString("F2") + " m/s", labelStyle);
        GUILayout.Label("Angular speed: " + agent.CurrentAngularSpeed.ToString("F2") + " rad/s", labelStyle);
        GUILayout.Label("Energy proxy: " + agent.CurrentEnergyProxy.ToString("F2"), labelStyle);
        GUILayout.Label("Average thrust command: " + agent.AverageThrustCommand.ToString("F2"), labelStyle);
        GUILayout.Label("Average motor health: " + agent.AverageMotorHealth.ToString("F2"), labelStyle);
        GUILayout.Space(6f);
        GUILayout.Label("Controls: I/K/J/L/U/O target, Q/E yaw, F fail motor, R repair, H hide HUD", labelStyle);
        GUILayout.EndArea();
    }

    private void EnsureStyles()
    {
        if (labelStyle != null && boxStyle != null)
        {
            return;
        }

        labelStyle = new GUIStyle(GUI.skin.label);
        labelStyle.fontSize = 16;
        labelStyle.normal.textColor = Color.white;

        boxStyle = new GUIStyle(GUI.skin.box);
        boxStyle.padding = new RectOffset(12, 12, 12, 12);
        boxStyle.normal.textColor = Color.white;
    }
}
