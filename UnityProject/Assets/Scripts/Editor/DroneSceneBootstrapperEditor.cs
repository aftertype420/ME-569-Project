#if UNITY_EDITOR
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

[CustomEditor(typeof(DroneSceneBootstrapper))]
public class DroneSceneBootstrapperEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        GUILayout.Space(12);

        GUIStyle buttonStyle = new GUIStyle(GUI.skin.button);
        buttonStyle.fontStyle = FontStyle.Bold;
        buttonStyle.fixedHeight = 36;

        if (GUILayout.Button("Build Spherical Drone Scene", buttonStyle))
        {
            DroneSceneBootstrapper builder = (DroneSceneBootstrapper)target;
            builder.BuildScene();

            if (!Application.isPlaying)
            {
                EditorUtility.SetDirty(builder);
                EditorSceneManager.MarkSceneDirty(builder.gameObject.scene);
            }
        }
    }
}
#endif
