using System;
using System.Collections.Generic;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

[CustomEditor(typeof(RayTracingPointLight))]
public class RayTracingPointLightEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        RayTracingPointLight pointLight = (RayTracingPointLight)target;
        Light unityLight = pointLight.AttachedLight;

        EditorGUILayout.Space();
        EditorGUILayout.LabelField("Runtime Persistence", EditorStyles.boldLabel);

        using (new EditorGUI.DisabledScope(!Application.isPlaying || unityLight == null))
        {
            if (GUILayout.Button("Apply Play Mode Values To Scene"))
            {
                RayTracingPointLightPlayModePersistence.QueueSave(pointLight);
            }
        }

        if (Application.isPlaying)
        {
            EditorGUILayout.HelpBox(
                "Click the button to write the current runtime Light parameters and Source Radius back to the scene after Play mode exits.",
                MessageType.Info
            );
        }
    }
}

[InitializeOnLoad]
public static class RayTracingPointLightPlayModePersistence
{
    private const string SessionKey = "UnityPathTracing.RayTracingPointLight.PlayModeOverrides";

    [Serializable]
    private class SaveData
    {
        public string scenePath;
        public string hierarchyPath;
        public float lightIntensity;
        public float lightRange;
        public Color lightColor;
        public bool lightEnabled;
        public float sourceRadius;
    }

    [Serializable]
    private class SaveDataCollection
    {
        public List<SaveData> items = new List<SaveData>();
    }

    static RayTracingPointLightPlayModePersistence()
    {
        EditorApplication.playModeStateChanged += OnPlayModeStateChanged;
    }

    public static void QueueSave(RayTracingPointLight runtimePointLight)
    {
        if (!Application.isPlaying || runtimePointLight == null)
            return;

        Light light = runtimePointLight.AttachedLight;
        if (light == null)
            return;

        SaveDataCollection collection = Load();
        string hierarchyPath = GetHierarchyPath(runtimePointLight.transform);

        SaveData data = new SaveData
        {
            scenePath = runtimePointLight.gameObject.scene.path,
            hierarchyPath = hierarchyPath,
            lightIntensity = light.intensity,
            lightRange = light.range,
            lightColor = light.color,
            lightEnabled = light.enabled,
            sourceRadius = runtimePointLight.SourceRadius
        };

        int existingIndex = collection.items.FindIndex(x =>
            x.scenePath == data.scenePath &&
            x.hierarchyPath == data.hierarchyPath
        );

        if (existingIndex >= 0)
            collection.items[existingIndex] = data;
        else
            collection.items.Add(data);

        Save(collection);
        Debug.Log($"Queued play mode light override for '{runtimePointLight.name}'. Exit Play mode to apply it to the scene.");
    }

    private static void OnPlayModeStateChanged(PlayModeStateChange state)
    {
        if (state == PlayModeStateChange.EnteredEditMode)
            ApplyPending();
    }

    private static void ApplyPending()
    {
        SaveDataCollection collection = Load();
        if (collection.items.Count == 0)
            return;

        bool appliedAny = false;

        foreach (SaveData item in collection.items)
        {
            GameObject target = FindGameObject(item.scenePath, item.hierarchyPath);
            if (target == null)
                continue;

            RayTracingPointLight pointLight = target.GetComponent<RayTracingPointLight>();
            Light light = target.GetComponent<Light>();
            if (pointLight == null || light == null)
                continue;

            Undo.RecordObject(pointLight, "Apply Play Mode RayTracing Point Light");
            Undo.RecordObject(light, "Apply Play Mode Unity Light");

            pointLight.SourceRadius = item.sourceRadius;
            light.intensity = item.lightIntensity;
            light.range = item.lightRange;
            light.color = item.lightColor;
            light.enabled = item.lightEnabled;

            EditorUtility.SetDirty(pointLight);
            EditorUtility.SetDirty(light);
            PrefabUtility.RecordPrefabInstancePropertyModifications(pointLight);
            PrefabUtility.RecordPrefabInstancePropertyModifications(light);
            EditorSceneManager.MarkSceneDirty(target.scene);
            appliedAny = true;
        }

        Save(new SaveDataCollection());

        if (appliedAny)
            Debug.Log("Applied queued play mode RayTracingPointLight values back to the scene. Save the scene to keep them.");
    }

    private static SaveDataCollection Load()
    {
        string json = SessionState.GetString(SessionKey, string.Empty);
        if (string.IsNullOrEmpty(json))
            return new SaveDataCollection();

        SaveDataCollection collection = JsonUtility.FromJson<SaveDataCollection>(json);
        return collection ?? new SaveDataCollection();
    }

    private static void Save(SaveDataCollection collection)
    {
        SessionState.SetString(SessionKey, JsonUtility.ToJson(collection));
    }

    private static string GetHierarchyPath(Transform target)
    {
        List<string> segments = new List<string>();
        Transform current = target;
        while (current != null)
        {
            segments.Add($"{current.GetSiblingIndex()}#{current.name}");
            current = current.parent;
        }

        segments.Reverse();
        return string.Join("/", segments);
    }

    private static GameObject FindGameObject(string scenePath, string hierarchyPath)
    {
        Scene scene = SceneManager.GetSceneByPath(scenePath);
        if (!scene.IsValid() || !scene.isLoaded)
            return null;

        string[] segments = hierarchyPath.Split('/');
        if (segments.Length == 0)
            return null;

        GameObject[] roots = scene.GetRootGameObjects();
        Transform current = FindChildBySegment(roots, segments[0]);
        if (current == null)
            return null;

        for (int i = 1; i < segments.Length; ++i)
        {
            current = FindChildBySegment(current, segments[i]);
            if (current == null)
                return null;
        }

        return current.gameObject;
    }

    private static Transform FindChildBySegment(GameObject[] roots, string segment)
    {
        ParseSegment(segment, out int siblingIndex, out string objectName);
        int currentIndex = 0;
        foreach (GameObject root in roots)
        {
            if (currentIndex == siblingIndex && root.name == objectName)
                return root.transform;
            currentIndex++;
        }

        return null;
    }

    private static Transform FindChildBySegment(Transform parent, string segment)
    {
        ParseSegment(segment, out int siblingIndex, out string objectName);
        for (int i = 0; i < parent.childCount; ++i)
        {
            Transform child = parent.GetChild(i);
            if (child.GetSiblingIndex() == siblingIndex && child.name == objectName)
                return child;
        }

        return null;
    }

    private static void ParseSegment(string segment, out int siblingIndex, out string objectName)
    {
        int splitIndex = segment.IndexOf('#');
        if (splitIndex <= 0)
        {
            siblingIndex = 0;
            objectName = segment;
            return;
        }

        siblingIndex = int.Parse(segment.Substring(0, splitIndex));
        objectName = segment.Substring(splitIndex + 1);
    }
}
