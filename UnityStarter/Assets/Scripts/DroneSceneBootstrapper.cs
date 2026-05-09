using UnityEngine;

/// <summary>
/// Creates a simple test scene at runtime so the starter project can run quickly.
///
/// Usage:
/// 1. Create an empty GameObject named Bootstrapper.
/// 2. Attach this script.
/// 3. Use the component menu or the context menu to run Build Spherical Drone Scene.
/// 4. Add ML-Agents Behavior Parameters and Decision Requester to the generated
///    SphericalDroneAgent GameObject.
/// 5. Press Play.
/// </summary>
public class DroneSceneBootstrapper : MonoBehaviour
{
    [Header("Build options")]
    public bool buildOnStart = false;
    public bool createGround = true;
    public bool createCameraAndLight = true;

    [Header("Drone geometry")]
    public float bodyRadius = 0.5f;
    public float motorDistanceFromCenter = 0.8f;
    public float motorVisualScale = 0.16f;

    [Header("Drone physics")]
    public float droneMass = 1.5f;
    public float motorMaxThrustNewtons = 18f;
    public float linearDrag = 0.2f;
    public float angularDrag = 0.6f;
    public Vector3 startPosition = new Vector3(0f, 2f, 0f);

    private void Start()
    {
        if (buildOnStart)
        {
            BuildScene();
        }
    }

    [ContextMenu("Build Spherical Drone Scene")]
    public void BuildScene()
    {
        DeleteExistingGeneratedObjects();

        GameObject droneRoot = new GameObject("SphericalDroneAgent");
        droneRoot.transform.position = startPosition;

        Rigidbody rb = droneRoot.AddComponent<Rigidbody>();
        rb.mass = droneMass;
        rb.drag = linearDrag;
        rb.angularDrag = angularDrag;
        rb.useGravity = true;

        SphereCollider collider = droneRoot.AddComponent<SphereCollider>();
        collider.radius = bodyRadius;

        GameObject bodyVisual = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        bodyVisual.name = "SphericalBodyVisual";
        bodyVisual.transform.SetParent(droneRoot.transform, false);
        bodyVisual.transform.localScale = Vector3.one * bodyRadius * 2f;
        SafeDestroy(bodyVisual.GetComponent<Collider>());

        DroneMotor[] motors = new DroneMotor[SixMotorDroneAgent.MotorCount];
        motors[0] = CreateMotor(droneRoot.transform, "TopMotor", 0, new Vector3(0f, motorDistanceFromCenter, 0f), Vector3.down);
        motors[1] = CreateMotor(droneRoot.transform, "BottomMotor", 1, new Vector3(0f, -motorDistanceFromCenter, 0f), Vector3.up);
        motors[2] = CreateMotor(droneRoot.transform, "LeftMotor", 2, new Vector3(-motorDistanceFromCenter, 0f, 0f), Vector3.right);
        motors[3] = CreateMotor(droneRoot.transform, "RightMotor", 3, new Vector3(motorDistanceFromCenter, 0f, 0f), Vector3.left);
        motors[4] = CreateMotor(droneRoot.transform, "FrontMotor", 4, new Vector3(0f, 0f, motorDistanceFromCenter), Vector3.back);
        motors[5] = CreateMotor(droneRoot.transform, "BackMotor", 5, new Vector3(0f, 0f, -motorDistanceFromCenter), Vector3.forward);

        GameObject target = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        target.name = "TargetMarker";
        target.transform.position = new Vector3(0f, 2f, 0f);
        target.transform.localScale = Vector3.one * 0.25f;
        SafeDestroy(target.GetComponent<Collider>());

        SixMotorDroneAgent agent = droneRoot.AddComponent<SixMotorDroneAgent>();
        agent.motors = motors;
        agent.targetMarker = target.transform;
        agent.hoverHeight = startPosition.y;
        agent.targetPositionLocal = target.transform.position;

        KeyboardTargetController keyboardTarget = droneRoot.AddComponent<KeyboardTargetController>();
        keyboardTarget.agent = agent;

        MotorFailureScheduler failureScheduler = droneRoot.AddComponent<MotorFailureScheduler>();
        failureScheduler.agent = agent;

        if (createGround)
        {
            GameObject ground = GameObject.CreatePrimitive(PrimitiveType.Plane);
            ground.name = "GroundPlane";
            ground.transform.position = Vector3.zero;
            ground.transform.localScale = new Vector3(2.5f, 1f, 2.5f);
        }

        if (createCameraAndLight)
        {
            CreateCameraAndLight(droneRoot.transform);
        }

        Debug.Log("Starter drone scene built. Add Behavior Parameters and Decision Requester to SphericalDroneAgent before training.");
    }

    private void DeleteExistingGeneratedObjects()
    {
        SafeDestroy(GameObject.Find("SphericalDroneAgent"));
        SafeDestroy(GameObject.Find("TargetMarker"));
        SafeDestroy(GameObject.Find("GroundPlane"));
        SafeDestroy(GameObject.Find("Main Camera"));
        SafeDestroy(GameObject.Find("Directional Light"));
    }

    private void SafeDestroy(Object objectToDestroy)
    {
        if (objectToDestroy == null)
        {
            return;
        }

        if (Application.isPlaying)
        {
            Destroy(objectToDestroy);
        }
        else
        {
            DestroyImmediate(objectToDestroy);
        }
    }

    private DroneMotor CreateMotor(Transform parent, string motorName, int motorIndex, Vector3 localPosition, Vector3 localForceDirection)
    {
        GameObject motorObject = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        motorObject.name = motorName;
        motorObject.transform.SetParent(parent, false);
        motorObject.transform.localPosition = localPosition;
        motorObject.transform.localRotation = Quaternion.FromToRotation(Vector3.up, localForceDirection.normalized);
        motorObject.transform.localScale = new Vector3(motorVisualScale, motorVisualScale * 0.5f, motorVisualScale);
        SafeDestroy(motorObject.GetComponent<Collider>());

        DroneMotor motor = motorObject.AddComponent<DroneMotor>();
        motor.motorName = motorName;
        motor.motorIndex = motorIndex;
        motor.maxThrustNewtons = motorMaxThrustNewtons;
        motor.health = 1f;

        return motor;
    }

    private void CreateCameraAndLight(Transform target)
    {
        GameObject lightObject = new GameObject("Directional Light");
        Light light = lightObject.AddComponent<Light>();
        light.type = LightType.Directional;
        light.intensity = 1.1f;
        lightObject.transform.rotation = Quaternion.Euler(45f, -30f, 0f);

        GameObject cameraObject = new GameObject("Main Camera");
        Camera camera = cameraObject.AddComponent<Camera>();
        camera.tag = "MainCamera";
        camera.transform.position = new Vector3(4f, 3f, -6f);
        camera.transform.LookAt(target.position);
        camera.fieldOfView = 55f;
    }
}
