using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(Tracing))]
[CanEditMultipleObjects]
public class TracingEditor : Editor
{
    public override void OnInspectorGUI()
    {
        serializedObject.Update();

        DrawSection("Core", () =>
        {
            Draw("tracingShader", "Tracing Shader");
            Draw("TraceDepth", "Trace Depth");
            Draw("targetFrameRate", "Target Frame Rate");
            Draw("FrameLimit", "Frame Limit");
        });

        DrawSection("Sky And Sun", () =>
        {
            Draw("skyboxTexture", "Skybox");
            Draw("SkyboxIntensity", "Sky Intensity");
            Draw("SunAngularRadius", "Sun Angular Radius");
        });

        DrawSection("View Modes", () =>
        {
            Draw("OnlyDrawAlbedo", "Albedo Only");
            Draw("OnlyDrawNormals", "Normals Only");
            Draw("OnlyDrawDepth", "Depth Only");
        });

        DrawSection("Direct Light Sampling", () =>
        {
            SerializedProperty di = serializedObject.FindProperty("UseReSTIRDI");
            using (new EditorGUI.DisabledScope(!di.hasMultipleDifferentValues && !di.boolValue))
                Draw("DirectLightRISCandidateCount", "DI Candidate Count");
        });

        DrawSection("ReSTIR DI", () =>
        {
            Draw("UseReSTIRDI", "Use ReSTIR DI");
        });

        DrawSection("ReSTIR GI", () =>
        {
            Draw("UseReSTIRGI", "Use ReSTIR GI");
            Draw("WriteReSTIRGIDiagnostics", "Write GI Diagnostics");
            Draw("WriteReSTIRGIDiagnosticDetails", "Write GI Diagnostic Details");
            Draw("ReSTIRGIDiagnosticFrameInterval", "GI Diagnostic Frame Interval");
            SerializedProperty gi = serializedObject.FindProperty("UseReSTIRGI");
            SerializedProperty depth = serializedObject.FindProperty("TraceDepth");
            if (!gi.hasMultipleDifferentValues && gi.boolValue &&
                !depth.hasMultipleDifferentValues && depth.intValue <= 1)
                EditorGUILayout.HelpBox("ReSTIR GI requires Trace Depth greater than 1.", MessageType.Warning);
        });

        SerializedProperty albedoOnly = serializedObject.FindProperty("OnlyDrawAlbedo");
        SerializedProperty normalsOnly = serializedObject.FindProperty("OnlyDrawNormals");
        SerializedProperty depthOnly = serializedObject.FindProperty("OnlyDrawDepth");
        if (albedoOnly.boolValue || normalsOnly.boolValue || depthOnly.boolValue)
            EditorGUILayout.HelpBox("Lighting debug view is active; ReSTIR DI and GI are not dispatched.", MessageType.Info);

        DrawSection("Output", () =>
        {
            Draw("AccumulateFrames", "Denoise (Frame Accumulation)");
            EditorGUILayout.HelpBox("开启：多帧平均降低噪声。关闭：显示当前帧。此选项不使用空间降噪滤镜；切换时重新累积。", MessageType.Info);
            Draw("ToneMap", "Tone Map");
            SerializedProperty toneMap = serializedObject.FindProperty("ToneMap");
            using (new EditorGUI.DisabledScope(!toneMap.hasMultipleDifferentValues && !toneMap.boolValue))
                Draw("Exposure", "Exposure");
        });

        DrawSection("Gizmos", () =>
        {
            Draw("drawGizmos", "Enable Gizmos");
            Draw("DrawTLAS", "Show TLAS");
            Draw("DrawBLAS", "Show BLAS");
            Draw("DrawMeshNode", "Show Mesh Nodes");
            Draw("DrawTLASBVH", "Show TLAS BVH");
        });

        serializedObject.ApplyModifiedProperties();
    }

    private void DrawSection(string title, System.Action drawContent)
    {
        EditorGUILayout.Space();
        using (new EditorGUILayout.VerticalScope(EditorStyles.helpBox))
        {
            EditorGUILayout.LabelField(title, EditorStyles.boldLabel);
            drawContent();
        }
    }

    private void Draw(string propertyName, string label)
    {
        SerializedProperty property = serializedObject.FindProperty(propertyName);
        if (property == null)
            return;

        EditorGUILayout.PropertyField(property, new GUIContent(label), true);
    }
}
