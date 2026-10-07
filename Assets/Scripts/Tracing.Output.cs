using System;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.SceneManagement;
using Stopwatch = System.Diagnostics.Stopwatch;
#if UNITY_EDITOR
using UnityEditor;
#endif

// One renderer owns these resources; partial files group its pipeline responsibilities.
public partial class Tracing
{
    private int _rasterCullingMask;
    private bool _rasterCullingSuppressed;

    // The compute tracer supplies the complete image. Avoid rasterizing the
    // same scene into a source image that Render() never reads.
    private void OnPreCull()
    {
        if (!_runtimeStarted || tracingShader == null || _rasterCullingSuppressed) return;
        _rasterCullingMask = cam.cullingMask;
        _rasterCullingSuppressed = true;
        cam.cullingMask = 0;
    }

    private void OnPostRender() => RestoreRasterCulling();

    private void RestoreRasterCulling()
    {
        if (!_rasterCullingSuppressed) return;
        cam.cullingMask = _rasterCullingMask;
        _rasterCullingSuppressed = false;
    }

    private void BlitToDisplay(RenderTexture source, RenderTexture destination)
    {
        if (!ToneMap || _toneMapMaterial == null)
        {
            Graphics.Blit(source, destination);
            return;
        }

        _toneMapMaterial.SetFloat("_Exposure", Exposure);
        Graphics.Blit(source, destination, _toneMapMaterial);
    }

    private void ApplyFrameRateLimit()
    {
        Application.targetFrameRate = targetFrameRate;
        QualitySettings.vSyncCount = 0;
        OnDemandRendering.renderFrameInterval = 1;
#if UNITY_EDITOR
        if (Application.isPlaying)
            SetEditorGameViewVSync(false);
#endif
    }

    private void CaptureFrameRateLimit()
    {
        if (_hasCapturedFrameRateState)
            return;

        _previousTargetFrameRate = Application.targetFrameRate;
        _previousVSyncCount = QualitySettings.vSyncCount;
        _previousRenderFrameInterval = OnDemandRendering.renderFrameInterval;
        _hasCapturedFrameRateState = true;
    }

    private void RestoreFrameRateLimit()
    {
        if (!_hasCapturedFrameRateState)
            return;

        Application.targetFrameRate = _previousTargetFrameRate;
        QualitySettings.vSyncCount = _previousVSyncCount;
        OnDemandRendering.renderFrameInterval = _previousRenderFrameInterval;
#if UNITY_EDITOR
        RestoreEditorGameViewVSync();
#endif
        _hasCapturedFrameRateState = false;
    }

    private void OnDrawGizmos()
    {
        if (!drawGizmos)
        {
            return;
        }

        var bnodes = BVHBuilder.GetBLASNodes();
        var meshNodes = BVHBuilder.GetMeshNodes();
        var tlasNodes = BVHBuilder.GetTLASNodes();
        var transforms = BVHBuilder.GetTransforms();

        if (DrawTLASBVH && tlasNodes != null && tlasNodes.Count > 0)
        {
            var tlasBVH = BVHBuilder.tlasTree;
            var orderedInfos = BVHBuilder.tlasTree.OriginTriOrMeshStartIndices;

            Queue<BVH.BVHNode> q = new();
            q.Enqueue(tlasBVH.BVHRoot);

            Color colLeaf  = new(0, 1,   0);
            Color colInner = new(0, 0.4f, 1);

            while (q.Count > 0)
            {
                BVH.BVHNode n = q.Dequeue();
                Gizmos.color = n.IsLeaf() ? colLeaf : colInner;
                if (n.IsLeaf())
                {
                    for (int i = n.OriginTriOrMeshStartIndex; i < n.OriginTriOrMeshEndIndex; ++i)
                    {
                        var mesh = meshNodes[orderedInfos[i]];
                        var transform_ = transforms[mesh.TransformIdx * 2];
                        Vector3 LocalCenter  = (mesh.BoundMin + mesh.BoundMax) * 0.5f;

                        var WorldCenter = transform_.MultiplyPoint3x4(LocalCenter);

                        TransformUtils.TransformSize(transform_, (mesh.BoundMax - mesh.BoundMin), out var WorldSize);

                        Gizmos.DrawWireCube(WorldCenter, WorldSize);
                    }
                }

                if (!n.IsLeaf())
                {
                    if (n.LeftChild != null)
                    {
                        q.Enqueue(n.LeftChild);
                        q.Enqueue(n.RightChild);
                    }
                }
            }
        }

        // GroundTruth
        if (DrawMeshNode && meshNodes != null && transforms != null)
        {
            for (int i = 0; i < meshNodes.Count; ++i)
            {
                var n            = meshNodes[i];
                var l2w          = transforms[n.TransformIdx * 2];
                TransformUtils.TransformSize(l2w, (n.BoundMax - n.BoundMin), out var WorldSize);

                Vector3 WorldCenter = l2w.MultiplyPoint3x4((n.BoundMin + n.BoundMax) * 0.5f);

                Gizmos.color = Color.yellow;
                Gizmos.DrawWireCube(WorldCenter, WorldSize);
            }
        }

        if (tlasNodes == null || tlasNodes.Count == 0) return;

        if (DrawTLAS)
        {
            Span<int> stackTLAS = stackalloc int[64];
            int sp = 0;
            stackTLAS[0] = 0;

            while (sp >= 0)
            {
                int idx = stackTLAS[sp--];
                if (idx < 0 || idx >= tlasNodes.Count) continue;

                var n = tlasNodes[idx];

                Vector3 center = (n.BoundMin + n.BoundMax) * 0.5f;
                Vector3 size = n.BoundMax - n.BoundMin;

                Gizmos.color = (n.TransformIdx >= 0)
                    ? new Color(1.0f, 0.4f, 0.6f)
                    : new Color(0.0f, 1.0f, 0.0f);
                if(n.TransformIdx >= 0)
                {
                    Gizmos.DrawWireCube(center, size);
                }

                if (n.TransformIdx < 0)
                {
                    stackTLAS[++sp] = n.Index + 1;
                    stackTLAS[++sp] = n.Index;

                    Gizmos.DrawWireCube(center, size);
                }
            }
        }

        if (bnodes != null && DrawBLAS && transforms != null && meshNodes != null)
        {
            for (int i = 0; i < meshNodes.Count; i++)
            {
                var meshNode = meshNodes[i];
                var localToWorld = transforms[meshNode.TransformIdx * 2];

                Gizmos.color = Color.green;

                int stackPtr = 0;
                Span<int> stack = stackalloc int[32];
                stack[stackPtr] = meshNode.Index;

                while (stackPtr >= 0 && stackPtr < 32)
                {
                    var idx = stack[stackPtr--];
                    var bnode = bnodes[idx];
                    TransformUtils.TransformSize(localToWorld, (bnode.BoundMax - bnode.BoundMin), out var WorldSize);
                    Vector3 WorldCenter = localToWorld.MultiplyPoint3x4((bnode.BoundMin + bnode.BoundMax) * 0.5f);

                    Color color = Color.red;
                    color.a = 0.5f;
                    Gizmos.color = color;
                    Gizmos.DrawWireCube(WorldCenter, WorldSize);

                    if(bnode.PrimitiveEndIdx < 0)
                    {
                        stack[++stackPtr] = bnode.Index;
                        stack[++stackPtr] = bnode.Index + 1;
                    }
                }
            }
        }
    }

#if UNITY_EDITOR
    private static Type GetEditorGameViewType()
    {
        if (_editorGameViewType == null)
            _editorGameViewType = typeof(EditorWindow).Assembly.GetType("UnityEditor.GameView");
        return _editorGameViewType;
    }

    private static PropertyInfo GetEditorGameViewVSyncProperty()
    {
        if (_editorGameViewVSyncProperty == null)
        {
            Type type = GetEditorGameViewType();
            if (type != null)
                _editorGameViewVSyncProperty = type.GetProperty("vSyncEnabled", BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
        }

        return _editorGameViewVSyncProperty;
    }

    private static void SetEditorGameViewVSync(bool enabled)
    {
        Type type = GetEditorGameViewType();
        PropertyInfo property = GetEditorGameViewVSyncProperty();
        if (type == null || property == null || !property.CanWrite)
            return;

        EditorWindow[] windows = Resources.FindObjectsOfTypeAll<EditorWindow>();
        foreach (EditorWindow window in windows)
        {
            if (window == null || !type.IsInstanceOfType(window))
                continue;

            if (!_editorGameViewVSyncStates.ContainsKey(window))
            {
                object currentValue = property.GetValue(window);
                if (currentValue is bool currentBool)
                    _editorGameViewVSyncStates[window] = currentBool;
            }

            property.SetValue(window, enabled);
            window.Repaint();
        }
    }

    private static void RestoreEditorGameViewVSync()
    {
        if (_editorGameViewVSyncStates.Count == 0)
            return;

        Type type = GetEditorGameViewType();
        PropertyInfo property = GetEditorGameViewVSyncProperty();
        if (type == null || property == null || !property.CanWrite)
        {
            _editorGameViewVSyncStates.Clear();
            return;
        }

        EditorWindow[] windows = Resources.FindObjectsOfTypeAll<EditorWindow>();
        foreach (EditorWindow window in windows)
        {
            if (window == null || !type.IsInstanceOfType(window))
                continue;

            if (!_editorGameViewVSyncStates.TryGetValue(window, out bool previousValue))
                continue;

            property.SetValue(window, previousValue);
            window.Repaint();
        }

        _editorGameViewVSyncStates.Clear();
    }
#endif

    private void StartReSTIRDiagnostics()
    {
        if (!Application.isPlaying || _restirDiagnostics != null)
            return;

        tracingShader.DisableKeyword("RESTIR_TELEMETRY_ENABLED");
        if (!WriteReSTIRGIDiagnostics)
            return;

        bool enableGpuTelemetry = false;
#if UNITY_EDITOR || DEVELOPMENT_BUILD
        enableGpuTelemetry = true;
#endif
        int width = cam != null ? Mathf.Max(cam.pixelWidth, 1) : 1;
        int height = cam != null ? Mathf.Max(cam.pixelHeight, 1) : 1;
        var settings = new ReSTIRDiagnosticsSettings(
            Path.Combine(Application.dataPath, "..", "Tools", "Output"),
            SceneManager.GetActiveScene().name,
            width,
            height,
            enableGpuTelemetry,
            Mathf.Max(ReSTIRGIDiagnosticFrameInterval, 1),
            UseReSTIRDI,
            IsReSTIRGIActive,
            false);

        try
        {
            _restirDiagnostics = ReSTIRDiagnosticsSession.Start(settings);
            if (_restirDiagnostics.GpuTelemetryEnabled)
            {
                _restirTelemetry = new ComputeBuffer(
                    ReSTIRTelemetryLayout.PacketWordCount,
                    sizeof(uint),
                    ComputeBufferType.Raw);
                tracingShader.EnableKeyword("RESTIR_TELEMETRY_ENABLED");
                BindReSTIRTelemetryBuffers();
            }
            else if (enableGpuTelemetry)
            {
                _restirDiagnostics.RecordStateChange("async_readback_unsupported", sampleCount);
            }
        }
        catch (Exception ex)
        {
            _restirDiagnostics = null;
            Debug.LogError($"[ReSTIR][Session] failed to start diagnostics: {ex.Message}");
        }
    }

    private void BindReSTIRTelemetryBuffers()
    {
        if (_restirTelemetry == null || _restirTelemetryKernels == null)
            return;

        for (int i = 0; i < _restirTelemetryKernels.Length; i++)
            tracingShader.SetBuffer(_restirTelemetryKernels[i], "ReSTIRTelemetry", _restirTelemetry);
    }

    private void BeginReSTIRTelemetryCapture()
    {
        _captureReSTIRTelemetryThisFrame = false;
        tracingShader.SetInt("_RestirTelemetryEnabled", 0);
        if ((!UseReSTIRDI && !IsReSTIRGIActive) || _restirDiagnostics == null || _restirTelemetry == null)
            return;

        BindReSTIRTelemetryBuffers();

        int directInitialSlot = UseReSTIRDI ? (_lastDirectReservoirOutputIdx + 1) % 3 : -1;
        int directFinalSlot = UseReSTIRDI
            ? (_hasDirectRestirHistory ? (_lastDirectReservoirOutputIdx + 2) % 3 : directInitialSlot)
            : -1;
        int indirectInitialSlot = IsReSTIRGIActive ? (_lastIndirectReservoirOutputIdx + 1) % 3 : -1;
        int indirectFinalSlot = IsReSTIRGIActive ? _lastIndirectReservoirOutputIdx : -1;

        _captureReSTIRTelemetryThisFrame = _restirDiagnostics.BeginCapture(
            (int)frameId,
            sampleCount,
            _currentRenderWidth,
            _currentRenderHeight,
            UseReSTIRDI,
            IsReSTIRGIActive,
            directInitialSlot,
            directFinalSlot,
            indirectInitialSlot,
            indirectFinalSlot);
        if (!_captureReSTIRTelemetryThisFrame)
            return;

        int modeFlags = (UseReSTIRDI ? 1 : 0) |
            (IsReSTIRGIActive ? 2 : 0) |
            (_hasDirectRestirHistory ? 4 : 0) |
            (_hasIndirectRestirHistory ? 8 : 0);
        int telemetrySampleStride = WriteReSTIRGIDiagnosticDetails ? 16 : ReSTIRTelemetrySampleStride;
        int diagnosticPixelIndex = GetReSTIRDiagnosticPixelIndex();
        tracingShader.SetInt("_RestirTelemetryEnabled", 1);
        tracingShader.SetInt("_RestirTelemetrySampleStride", telemetrySampleStride);
        tracingShader.SetInt("_RestirTelemetrySamplePhase", sampleCount % telemetrySampleStride);
        tracingShader.SetInt("_RestirTelemetrySelectedPixelIndex", diagnosticPixelIndex);
        tracingShader.SetInt("_RestirTelemetryGeneration", _restirDiagnostics.Generation);
        tracingShader.SetInt("_RestirTelemetrySampleCount", sampleCount);
        tracingShader.SetInt("_RestirTelemetryModeFlags", modeFlags);
        tracingShader.SetInt("_RestirTelemetryDirectInitialSlot", directInitialSlot);
        tracingShader.SetInt("_RestirTelemetryDirectFinalSlot", directFinalSlot);
        tracingShader.SetInt("_RestirTelemetryIndirectInitialSlot", indirectInitialSlot);
        tracingShader.SetInt("_RestirTelemetryIndirectFinalSlot", indirectFinalSlot);
        tracingShader.Dispatch(
            kernelClearReSTIRTelemetry,
            (ReSTIRTelemetryLayout.PacketWordCount + 63) / 64,
            1,
            1);
    }

    private void EndReSTIRTelemetryCapture(double frameMilliseconds)
    {
        if (!_captureReSTIRTelemetryThisFrame)
            return;

        _restirDiagnostics.RequestCapture(_restirTelemetry, frameMilliseconds);
        _captureReSTIRTelemetryThisFrame = false;
        tracingShader.SetInt("_RestirTelemetryEnabled", 0);
    }

    private void CaptureReSTIRFrameTelemetry(int pixelCount)
    {
        if (!_captureReSTIRTelemetryThisFrame)
            return;

        tracingShader.SetBuffer(kernelCaptureReSTIRFrame, "GlobalColors", _globalColors);
        tracingShader.Dispatch(kernelCaptureReSTIRFrame, (pixelCount + 63) / 64, 1, 1);
    }

    private void StopReSTIRDiagnostics()
    {
        if (_restirDiagnostics != null && _restirDiagnostics.ReadbackPending)
            AsyncGPUReadback.WaitAllRequests();

        _restirDiagnostics?.Dispose();
        _restirDiagnostics = null;
        _restirTelemetry?.Release();
        _restirTelemetry = null;
        _captureReSTIRTelemetryThisFrame = false;
        if (tracingShader != null)
            tracingShader.DisableKeyword("RESTIR_TELEMETRY_ENABLED");
    }
    private void ClearAccumulationRenderTargets()
    {
        if (target != null)
            ClearRenderTexture(target);

        if (frameConverged != null)
            ClearRenderTexture(frameConverged);
    }

    private static void ClearRenderTexture(RenderTexture renderTexture)
    {
        if (renderTexture == null)
            return;

        RenderTexture previous = RenderTexture.active;
        RenderTexture.active = renderTexture;
        GL.Clear(false, true, Color.clear);
        RenderTexture.active = previous;
    }

}
