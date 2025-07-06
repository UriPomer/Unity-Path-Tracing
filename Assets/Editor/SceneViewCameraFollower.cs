#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;

[InitializeOnLoad]
public static class CameraTwoWaySync
{
    private static float lastDistance = 10f;

    static CameraTwoWaySync()
    {
        EditorApplication.update += Update;
        EditorApplication.playModeStateChanged += OnPlayModeChanged;
    }
    
    private static void OnPlayModeChanged(PlayModeStateChange state)
    {
        if (state == PlayModeStateChange.EnteredPlayMode)
        {
            var sv = SceneView.lastActiveSceneView;
            var cam = Camera.main;
            if (sv != null && cam != null)
            {
                cam.transform.position = sv.camera.transform.position;
                cam.transform.rotation = sv.camera.transform.rotation;
            }
        }
    }
    
    private static void Update()
    {
        var sceneView = SceneView.lastActiveSceneView;
        var mainCam   = Camera.main;
        if (sceneView == null || mainCam == null) return;

        // 当前哪个窗口有焦点
        var focused = EditorWindow.focusedWindow;
        var name    = focused != null ? focused.GetType().Name : "";

        if (name == "SceneView")
        {
            var svCam = sceneView.camera.transform;
            var mTf   = mainCam.transform;
            mTf.position = svCam.position;
            mTf.rotation = svCam.rotation;
            return;
        }

        if (EditorApplication.isPlaying && name == "GameView")
        {
            if (sceneView.cameraDistance != lastDistance)
                lastDistance = sceneView.cameraDistance;

            sceneView.pivot    = mainCam.transform.position + mainCam.transform.forward * lastDistance;
            sceneView.rotation = mainCam.transform.rotation;
            sceneView.Repaint();
        }
    }
}
#endif