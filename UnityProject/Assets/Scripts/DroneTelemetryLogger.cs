using System;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

/// <summary>
/// Logs drone performance to a CSV file for report figures and baseline-vs-ML comparison.
/// Attach this to SphericalDroneAgent.
/// CSV files are written to Application.persistentDataPath.
/// </summary>
public class DroneTelemetryLogger : MonoBehaviour
{
    public SixMotorDroneAgent agent;
    public bool logOnPlay = true;
    public float samplePeriodSeconds = 0.2f;
    public string fileNamePrefix = "spherical_drone_telemetry";
    public bool logMotorDetails = true;

    private StreamWriter writer;
    private float timer;
    private string activePath;

    private void Reset()
    {
        agent = GetComponent<SixMotorDroneAgent>();
    }

    private void Start()
    {
        if (agent == null)
        {
            agent = GetComponent<SixMotorDroneAgent>();
        }

        if (logOnPlay)
        {
            StartLogging();
        }
    }

    private void Update()
    {
        if (writer == null || agent == null)
        {
            return;
        }

        timer += Time.deltaTime;
        if (timer < samplePeriodSeconds)
        {
            return;
        }

        timer = 0f;
        WriteRow();
    }

    [ContextMenu("Start Logging")]
    public void StartLogging()
    {
        StopLogging();

        string safeTime = DateTime.Now.ToString("yyyyMMdd_HHmmss", CultureInfo.InvariantCulture);
        string fileName = fileNamePrefix + "_" + safeTime + ".csv";
        activePath = Path.Combine(Application.persistentDataPath, fileName);

        writer = new StreamWriter(activePath, false, Encoding.UTF8);
        writer.WriteLine(BuildHeader());
        writer.Flush();

        Debug.Log("Drone telemetry logging started: " + activePath);
    }

    [ContextMenu("Stop Logging")]
    public void StopLogging()
    {
        if (writer != null)
        {
            writer.Flush();
            writer.Close();
            writer = null;
            Debug.Log("Drone telemetry logging stopped: " + activePath);
        }
    }

    [ContextMenu("Print Log Folder")]
    public void PrintLogFolder()
    {
        Debug.Log("Drone telemetry folder: " + Application.persistentDataPath);
    }

    private string BuildHeader()
    {
        StringBuilder sb = new StringBuilder();
        sb.Append("time,episode,episode_time,mode,pos_x,pos_y,pos_z,target_x,target_y,target_z,position_error,speed,angular_speed,energy_proxy,average_thrust,average_health");

        if (logMotorDetails)
        {
            for (int i = 0; i < SixMotorDroneAgent.MotorCount; i++)
            {
                sb.Append(",m").Append(i).Append("_health");
                sb.Append(",m").Append(i).Append("_cmd");
                sb.Append(",m").Append(i).Append("_pitch_deg");
                sb.Append(",m").Append(i).Append("_yaw_deg");
                sb.Append(",m").Append(i).Append("_effective_thrust");
                sb.Append(",m").Append(i).Append("_energy");
            }
        }

        return sb.ToString();
    }

    private void WriteRow()
    {
        CultureInfo c = CultureInfo.InvariantCulture;
        Vector3 pos = agent.transform.position;
        Vector3 target = agent.TargetWorldPosition;

        StringBuilder sb = new StringBuilder();
        sb.Append(Time.time.ToString("G6", c));
        Append(sb, agent.EpisodeIndex, c);
        Append(sb, agent.EpisodeTimer, c);
        sb.Append(',').Append(agent.heuristicControlMode.ToString());
        Append(sb, pos.x, c);
        Append(sb, pos.y, c);
        Append(sb, pos.z, c);
        Append(sb, target.x, c);
        Append(sb, target.y, c);
        Append(sb, target.z, c);
        Append(sb, agent.CurrentPositionError, c);
        Append(sb, agent.CurrentSpeed, c);
        Append(sb, agent.CurrentAngularSpeed, c);
        Append(sb, agent.CurrentEnergyProxy, c);
        Append(sb, agent.AverageThrustCommand, c);
        Append(sb, agent.AverageMotorHealth, c);

        if (logMotorDetails)
        {
            for (int i = 0; i < SixMotorDroneAgent.MotorCount; i++)
            {
                DroneMotor motor = agent.GetMotorByIndex(i);
                if (motor == null)
                {
                    Append(sb, 0f, c);
                    Append(sb, 0f, c);
                    Append(sb, 0f, c);
                    Append(sb, 0f, c);
                    Append(sb, 0f, c);
                    Append(sb, 0f, c);
                    continue;
                }

                Append(sb, motor.health, c);
                Append(sb, agent.GetLastThrustCommand(i), c);
                Append(sb, agent.GetLastPitchCommandDeg(i), c);
                Append(sb, agent.GetLastYawCommandDeg(i), c);
                Append(sb, motor.lastEffectiveThrust, c);
                Append(sb, motor.energyProxyThisEpisode, c);
            }
        }

        writer.WriteLine(sb.ToString());
        writer.Flush();
    }

    private static void Append(StringBuilder sb, float value, CultureInfo cultureInfo)
    {
        sb.Append(',').Append(value.ToString("G6", cultureInfo));
    }

    private static void Append(StringBuilder sb, int value, CultureInfo cultureInfo)
    {
        sb.Append(',').Append(value.ToString(cultureInfo));
    }

    private void OnDisable()
    {
        StopLogging();
    }

    private void OnApplicationQuit()
    {
        StopLogging();
    }
}
