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
            Draw("SunFocus", "Sun Focus");
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
            Draw("DirectLightRISCandidateCount", "Candidate Count");
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
        });

        DrawSection("Output", () =>
        {
            Draw("Denoise", "Denoise");
            Draw("ToneMap", "Tone Map");
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
