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
            Draw("UseDirectLightReservoirRIS", "Use RIS");
            Draw("DirectLightRISCandidateCount", "Candidate Count");
        });

        DrawSection("Prev-Frame Neighbor Replay", () =>
        {
            EditorGUILayout.HelpBox(
                "This replays previous-frame reservoirs from nearby pixels. It is not the paper's full spatial reuse pass yet.",
                MessageType.Info);
            EditorGUILayout.HelpBox(
                "Mode 5 colors: Gray = no first-hit surface, Magenta = replay path off, Blue = waiting for previous-frame history, Cyan = no direct-light candidates, Black = current pixel has no valid local direct-light sample, White = no stored previous reservoir, Orange = stored reservoir invalid, Red = valid source only, Yellow = compatible but re-eval failed, Green = re-evaluated. Debug view bypasses tone mapping and exposure.",
                MessageType.None);
            Draw("UseDirectLightReservoirNeighborReuse", "Use Prev-Frame Neighbor Replay");
            Draw("DirectLightNeighborReuseCount", "Replay Neighbor Count");
        });

        DrawSection("Direct Light Debug", () =>
        {
            Draw("DirectLightDebugViewMode", "Debug View");
            Draw("DirectLightDiagnosticPixelOffset", "Diagnostic Pixel Offset");
        });

        DrawSection("Center Pixel Diagnostics", () =>
        {
            Tracing tracing = (Tracing)target;
            string diagnosticsText = string.Join("\n", tracing.GetDirectLightDiagnosticsSummaryLines());

            using (new EditorGUILayout.HorizontalScope())
            {
                GUILayout.FlexibleSpace();
                if (GUILayout.Button("Copy All", GUILayout.Width(80f)))
                    EditorGUIUtility.systemCopyBuffer = diagnosticsText;
            }

            EditorGUILayout.SelectableLabel(diagnosticsText, EditorStyles.textArea, GUILayout.MinHeight(90f));
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
