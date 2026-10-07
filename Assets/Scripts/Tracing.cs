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

[RequireComponent(typeof(LightManager))]
public partial class Tracing : MonoBehaviour
{
    public ComputeShader tracingShader;

    private Camera cam;
    private LightManager _lights;
    private RenderTexture target;

    [Header("Skybox Settings")]
    [SerializeField]
    private Texture skyboxTexture;
    [SerializeField, Range(0.0f, 10.0f)]
    float SkyboxIntensity = 1.0f;
    [SerializeField, Range(0.004f, 0.1f)]
    float SunAngularRadius = 0.1f;

    [SerializeField, Range(1, 8)]
    int TraceDepth = 3;

    [SerializeField, Range(15,240)]
    int targetFrameRate = 90;

    [SerializeField, Min(0)]
    int FrameLimit = 0;

    [Header("Debug")]
    [SerializeField] bool OnlyDrawAlbedo = false;
    [SerializeField] bool OnlyDrawNormals = false;
    [SerializeField] bool OnlyDrawDepth = false;
    [SerializeField, Range(1, 16)] int DirectLightRISCandidateCount = 1;
    [SerializeField] bool UseReSTIRDI = false;
    [SerializeField] bool UseReSTIRGI = false;
    [SerializeField] bool WriteReSTIRGIDiagnostics = true;
    [SerializeField] bool WriteReSTIRGIDiagnosticDetails = false;
    [SerializeField, Min(1)] int ReSTIRGIDiagnosticFrameInterval = 8;

    [Header("Display")]
    [SerializeField] bool UseTemporalDenoising = true;
    [UnityEngine.Serialization.FormerlySerializedAs("Denoise")]
    [SerializeField] bool AccumulateFrames = true;
    [SerializeField] bool ToneMap = true;
    [SerializeField, Range(0.1f, 8.0f)] float Exposure = 1.0f;

    [Header("Draw Gizmos")]
    [SerializeField]
    private bool drawGizmos = false;
    [SerializeField]
    private bool DrawTLAS = true;
    [SerializeField] private bool DrawBLAS = true;
    [SerializeField] private bool DrawMeshNode = true;
    [SerializeField] private bool DrawTLASBVH = true;

    private int sampleCount = 0;
    private Material _addMaterial;
    private Material _toneMapMaterial;

    private RenderTexture frameConverged;

    private int prevWidth, prevHeight, prevTraceDepth;
    private int _currentRenderWidth;
    private int _currentRenderHeight;

    // Cached arrays and objects to avoid per-frame GC allocations
    private int[] bvhKernels;
    private CommandBuffer cmdBuffer;
    private string[] bounceNames = new string[24]; // 8 bounces × 3 phases = 24
    private int _lastLightStateHash = int.MinValue;
    private Texture _oldSkyboxTexture;
    private int _oldDebugMode;
    private float _oldSkyboxIntensity = 1.0f;
    private float _oldSunAngularRadius = 0.1f;
    private int _oldTraceDepth = 3;
    private int _oldTargetFrameRate = 90;
    private int _oldDirectLightRISCandidateCount = 1;
    private bool _oldUseReSTIRDI = false;
    private bool _oldUseReSTIRGI = false;
    private bool _oldAccumulateFrames = true;
    private bool _oldTemporalDenoising = true;
    private TemporalDenoiser _denoiser;
    private bool TemporalDenoisingActive => UseTemporalDenoising && DebugMode == 0;
    private RenderTexture DisplayTexture => TemporalDenoisingActive && _denoiser?.Output != null
        ? _denoiser.Output : AccumulateFrames ? frameConverged : target;
    private bool _hasPrimarySurfaceHistory = false;
    private Matrix4x4 _previousCameraViewProjection = Matrix4x4.identity;
    private Vector3 _previousRestirCameraPosition;
    private int _previousTargetFrameRate = -1;
    private int _previousVSyncCount = -1;
    private int _previousRenderFrameInterval = -1;
    private bool _hasCapturedFrameRateState = false;
    private ReSTIRDiagnosticsSession _restirDiagnostics;
    private int[] _restirTelemetryKernels;
    private bool _captureReSTIRTelemetryThisFrame;
    private bool _runtimeStarted;
    private bool _reportedSlowReSTIRGIFrame;
    private bool IsReSTIRGIActive => UseReSTIRGI && TraceDepth > 1;
#if UNITY_EDITOR
    private static Type _editorGameViewType;
    private static PropertyInfo _editorGameViewVSyncProperty;
    private static readonly Dictionary<EditorWindow, bool> _editorGameViewVSyncStates = new Dictionary<EditorWindow, bool>();
#endif

    private void Awake()
    {
        _lights = GetComponent<LightManager>();
        EnsureMaterials();
    }

    private void OnEnable()
    {
        if (!Application.isPlaying)
            return;

        CaptureFrameRateLimit();
        ApplyFrameRateLimit();
        if (_runtimeStarted)
        {
            cmdBuffer = new CommandBuffer();
            ResetSampleCount("renderer_enabled");
            StartReSTIRDiagnostics();
        }
    }

    private void OnRenderImage(RenderTexture source, RenderTexture destination)
    {
        Render(source, destination);
    }

    private Vector2Int GetRenderDimensions(RenderTexture source, RenderTexture destination)
    {
        if (source != null && source.width > 0 && source.height > 0)
            return new Vector2Int(source.width, source.height);

        if (destination != null && destination.width > 0 && destination.height > 0)
            return new Vector2Int(destination.width, destination.height);

        Camera renderCamera = cam != null ? cam : GetComponent<Camera>();
        if (renderCamera != null && renderCamera.pixelWidth > 0 && renderCamera.pixelHeight > 0)
            return new Vector2Int(renderCamera.pixelWidth, renderCamera.pixelHeight);

        return new Vector2Int(1, 1);
    }

    private void Render(RenderTexture source, RenderTexture destination)
    {
        long renderStartTimestamp = Stopwatch.GetTimestamp();
        long giStartTimestamp = 0;
        long giEndTimestamp = 0;
        EnsureMaterials();

        Vector2Int renderDimensions = GetRenderDimensions(source, destination);
        if ((_currentRenderWidth > 0 && _currentRenderHeight > 0) &&
            (_currentRenderWidth != renderDimensions.x || _currentRenderHeight != renderDimensions.y))
        {
            ResetSampleCount("resolution_changed");
        }

        _currentRenderWidth = renderDimensions.x;
        _currentRenderHeight = renderDimensions.y;

        if (target == null || target.width != renderDimensions.x || target.height != renderDimensions.y)
        {
            ReleaseRenderTexture(ref target);
            target = new RenderTexture(renderDimensions.x, renderDimensions.y, 0, RenderTextureFormat.ARGBFloat,
                RenderTextureReadWrite.Linear);
            target.filterMode = FilterMode.Point;
            target.wrapMode   = TextureWrapMode.Clamp;
            target.enableRandomWrite = true;
            target.Create();
        }
        if (frameConverged == null ||
            frameConverged.width != renderDimensions.x ||
            frameConverged.height != renderDimensions.y)
        {
            ReleaseRenderTexture(ref frameConverged);
            frameConverged = new RenderTexture(renderDimensions.x, renderDimensions.y, 0, RenderTextureFormat.ARGBFloat, RenderTextureReadWrite.Linear);
            frameConverged.filterMode = FilterMode.Point;
            frameConverged.wrapMode   = TextureWrapMode.Clamp;
            frameConverged.enableRandomWrite = true;
            frameConverged.Create();
        }

        bool sceneChanged = BVHBuilder.Validate();
        Matrix4x4 viewProjection = GL.GetGPUProjectionMatrix(cam.projectionMatrix, false) * cam.worldToCameraMatrix;
        bool cameraMoved = _hasPrimarySurfaceHistory && viewProjection != _previousCameraViewProjection;
        if (sceneChanged)
        {
            if (_denoiser != null && _denoiser.GeometryRevision == BVHBuilder.GeometryRevision)
            {
                ResetAccumulationOnly("scene_moved");
                // Stored GI secondary vertices are not yet mapped through object
                // motion. Discard reservoirs, while preserving denoiser motion.
                ResetReservoirHistory();
            }
            else
                ResetSampleCount("scene_changed");
        }
        else if (cameraMoved)
        {
            ResetAccumulationOnly("camera_moved");
        }

        CreateBuffersIfNeeded(renderDimensions.x, renderDimensions.y);
        CreateReSTIRBuffersIfNeeded(renderDimensions.x * renderDimensions.y);
        if (FrameLimit > 0 && sampleCount >= FrameLimit)
        {
            BlitToDisplay(DisplayTexture, destination);
            return;
        }

        // Rotate only when a frame will be rendered. The last completed primary
        // surface becomes history; the other buffer receives this frame's hits.
        if (UseReSTIRDI || IsReSTIRGIActive || TemporalDenoisingActive)
            (_primarySurfaceHistory, _primarySurfaceHistoryPrev) = (_primarySurfaceHistoryPrev, _primarySurfaceHistory);

        if (TemporalDenoisingActive)
        {
            _denoiser ??= new TemporalDenoiser();
            _denoiser.Prepare(cam, renderDimensions.x, renderDimensions.y, tracingShader, kernelTrace);
        }
        else
        {
            _denoiser?.Dispose(); _denoiser = null;
        }
        SetShaderParameters();
        _restirDiagnostics?.RecordRenderModes(UseReSTIRDI, IsReSTIRGIActive);
        sampleCount++;

        int pixelCount = _currentRenderWidth * _currentRenderHeight;

        // 1. Generate primary rays
        tracingShader.Dispatch(kernelGenerate, (pixelCount + 63) / 64, 1, 1);

        // 2. Per-bounce loop (skip when debug modes set throughput=0)
        bool debugMode = OnlyDrawAlbedo || OnlyDrawNormals || OnlyDrawDepth;

        if (!debugMode)
        {
            tracingShader.SetInt("CurBounce", 0);
            tracingShader.SetBuffer(kernelTrace, "GlobalRays", _globalRaysA);
            tracingShader.SetBuffer(kernelTrace, "GlobalHits", _globalHits);
            tracingShader.Dispatch(kernelTrace, (pixelCount + 63) / 64, 1, 1);

            BeginReSTIRTelemetryCapture();

            if (UseReSTIRDI)
                DispatchReSTIRDI(pixelCount);

            if (IsReSTIRGIActive)
            {
                giStartTimestamp = Stopwatch.GetTimestamp();
                DispatchReSTIRGI(pixelCount);
                giEndTimestamp = Stopwatch.GetTimestamp();
            }

            tracingShader.SetBuffer(kernelShade, "ShadeRays", _globalRaysA);
            tracingShader.SetBuffer(kernelShade, "GlobalRays2", _globalRaysB);
            tracingShader.SetBuffer(kernelShade, "ShadeHits", _globalHits);
            tracingShader.Dispatch(kernelShade, (pixelCount + 63) / 64, 1, 1);

            tracingShader.SetInt("Type", 1);
            tracingShader.Dispatch(kernelTransfer, 1, 1, 1);

            cmdBuffer.Clear();
            cmdBuffer.name = bounceNames[2];
            cmdBuffer.DispatchCompute(tracingShader, kernelShadow, _indirectArgs, 0);
            Graphics.ExecuteCommandBuffer(cmdBuffer);

            bool readA = false;
            for (int bounce = 1; bounce < TraceDepth; bounce++)
            {
                tracingShader.SetInt("CurBounce", bounce);

                // Ping-pong: bind read buffer to GlobalRays, write buffer to GlobalRays2
                var readBuf = readA ? _globalRaysA : _globalRaysB;
                var writeBuf = readA ? _globalRaysB : _globalRaysA;
                tracingShader.SetBuffer(kernelTrace, "GlobalRays", readBuf);
                tracingShader.SetBuffer(kernelTrace, "GlobalHits", _globalHits);
                tracingShader.SetBuffer(kernelShade, "ShadeRays", readBuf);
                tracingShader.SetBuffer(kernelShade, "GlobalRays2", writeBuf);
                tracingShader.SetBuffer(kernelShade, "ShadeHits", _globalHits);

                // Transfer0 (Type=0): compute trace/shade dispatch args
                tracingShader.SetInt("Type", 0);
                tracingShader.Dispatch(kernelTransfer, 1, 1, 1);

                // Trace (indirect dispatch)
                cmdBuffer.Clear();
                cmdBuffer.name = bounceNames[bounce * 3];
                cmdBuffer.DispatchCompute(tracingShader, kernelTrace, _indirectArgs, 0);
                Graphics.ExecuteCommandBuffer(cmdBuffer);

                // Shade (indirect dispatch, same count as trace)
                cmdBuffer.Clear();
                cmdBuffer.name = bounceNames[bounce * 3 + 1];
                cmdBuffer.DispatchCompute(tracingShader, kernelShade, _indirectArgs, 0);
                Graphics.ExecuteCommandBuffer(cmdBuffer);

                // Transfer1 (Type=1): compute shadow dispatch args
                tracingShader.SetInt("Type", 1);
                tracingShader.Dispatch(kernelTransfer, 1, 1, 1);

                // Shadow (indirect dispatch)
                cmdBuffer.Clear();
                cmdBuffer.name = bounceNames[bounce * 3 + 2];
                cmdBuffer.DispatchCompute(tracingShader, kernelShadow, _indirectArgs, 0);
                Graphics.ExecuteCommandBuffer(cmdBuffer);

                readA = !readA;
            }

            CaptureReSTIRFrameTelemetry(pixelCount);
        }

        // 3. Finalize
        tracingShader.Dispatch(kernelFinalize,
            Mathf.CeilToInt(_currentRenderWidth / 8.0f),
            Mathf.CeilToInt(_currentRenderHeight / 8.0f), 1);

        if (TemporalDenoisingActive)
            _denoiser.Reconstruct(cam, target, _primarySurfaceHistory, _primarySurfaceHistoryPrev);
        else if (AccumulateFrames)
        {
            _addMaterial.SetFloat("_Sample", sampleCount);
            Graphics.Blit(target, frameConverged, _addMaterial);
        }
        BlitToDisplay(DisplayTexture, destination);

        _hasPrimarySurfaceHistory = true;
        double renderMilliseconds =
            (Stopwatch.GetTimestamp() - renderStartTimestamp) * 1000.0 / Stopwatch.Frequency;
        if (giStartTimestamp != 0 && renderMilliseconds > 1000.0 && !_reportedSlowReSTIRGIFrame)
        {
            _reportedSlowReSTIRGIFrame = true;
            double beforeGiMilliseconds = (giStartTimestamp - renderStartTimestamp) * 1000.0 / Stopwatch.Frequency;
            double giDispatchMilliseconds = (giEndTimestamp - giStartTimestamp) * 1000.0 / Stopwatch.Frequency;
            Debug.LogWarning($"[ReSTIR][Performance] slow GI frame: total={renderMilliseconds:F1} ms, beforeGI={beforeGiMilliseconds:F1} ms, dispatchGI={giDispatchMilliseconds:F1} ms, afterGI={renderMilliseconds - beforeGiMilliseconds - giDispatchMilliseconds:F1} ms. CPU wall time includes shader compilation and GPU synchronization; inspect Profiler for GPU kernel time.");
        }
        EndReSTIRTelemetryCapture(renderMilliseconds);
    }

    private void Update()
    {
        if (_oldTargetFrameRate != targetFrameRate)
        {
            ApplyFrameRateLimit();
            _oldTargetFrameRate = targetFrameRate;
        }
        bool resetRequired = false;
        string runtimeStateChangeReason = null;

        _lights.UpdateLights();
        int lightStateHash = _lights.ComputeLightStateHash();
        if (lightStateHash != _lastLightStateHash)
        {
            _lastLightStateHash = lightStateHash;
            resetRequired = true;
            runtimeStateChangeReason = runtimeStateChangeReason ?? "light_state_changed";
        }

        bool materialChanged = BVHBuilder.ReloadMaterials();
        resetRequired |= materialChanged;
        if (materialChanged)
            runtimeStateChangeReason = runtimeStateChangeReason ?? "material_state_changed";

        if (HaveRuntimeSettingsChanged())
        {
            if (runtimeStateChangeReason == null)
            {
                if (_oldUseReSTIRGI != UseReSTIRGI)
                    runtimeStateChangeReason = "restir_gi_toggled";
                else if (_oldUseReSTIRDI != UseReSTIRDI)
                    runtimeStateChangeReason = "restir_di_toggled";
                else if (_oldTraceDepth != TraceDepth)
                    runtimeStateChangeReason = "trace_depth_changed";
                else if (_oldAccumulateFrames != AccumulateFrames)
                    runtimeStateChangeReason = "accumulation_toggled";
                else
                    runtimeStateChangeReason = "runtime_settings_changed";
            }
            CacheRuntimeSettings();
            resetRequired = true;
        }

        if (resetRequired)
            ResetSampleCount(runtimeStateChangeReason ?? "runtime_reset");

    }

    private void OnValidate()
    {
        if (!Application.isPlaying)
            return;

        ApplyFrameRateLimit();
    }

    private uint frameId = 0;

    private void OnDisable()
    {
        RestoreRasterCulling();
        _denoiser?.Dispose(); _denoiser = null;
        StopReSTIRDiagnostics();
        RestoreFrameRateLimit();
        ReleaseRenderTargets();
        ReleaseBuffers();
        ReleaseCommandBuffer();
        ReleaseMaterials();
        BVHBuilder.Destroy();
    }

    private int DebugMode => (OnlyDrawAlbedo ? 1 : 0) | (OnlyDrawNormals ? 2 : 0) | (OnlyDrawDepth ? 4 : 0);

    private bool HaveRuntimeSettingsChanged()
    {
        return _oldSkyboxTexture != skyboxTexture ||
               _oldDebugMode != DebugMode ||
               _oldSkyboxIntensity != SkyboxIntensity ||
               _oldSunAngularRadius != SunAngularRadius ||
               _oldTraceDepth != TraceDepth ||
               _oldDirectLightRISCandidateCount != DirectLightRISCandidateCount ||
               _oldUseReSTIRDI != UseReSTIRDI ||
               _oldUseReSTIRGI != UseReSTIRGI ||
               _oldAccumulateFrames != AccumulateFrames ||
               _oldTemporalDenoising != UseTemporalDenoising;
    }

    private void CacheRuntimeSettings()
    {
        ApplyFrameRateLimit();
        _oldSkyboxTexture = skyboxTexture;
        _oldDebugMode = DebugMode;
        _oldSkyboxIntensity = SkyboxIntensity;
        _oldSunAngularRadius = SunAngularRadius;
        _oldTraceDepth = TraceDepth;
        _oldTargetFrameRate = targetFrameRate;
        _oldDirectLightRISCandidateCount = DirectLightRISCandidateCount;
        _oldUseReSTIRDI = UseReSTIRDI;
        _oldUseReSTIRGI = UseReSTIRGI;
        _oldAccumulateFrames = AccumulateFrames;
        _oldTemporalDenoising = UseTemporalDenoising;
    }

    private void ResetSampleCount(string reason = "full_reset")
    {
        _denoiser?.Reset();
        _restirDiagnostics?.ScheduleResetCapture(reason, sampleCount);
        sampleCount = 0;
        frameId = 0;
        _hasPrimarySurfaceHistory = false;
        ResetReservoirHistory();
        ClearAccumulationRenderTargets();
    }

    private void ResetAccumulationOnly(string reason = "accumulation_reset")
    {
        _restirDiagnostics?.ScheduleResetCapture(reason, sampleCount);
        sampleCount = 0;
        ClearAccumulationRenderTargets();
    }

    private void ResetReservoirHistory()
    {
        _hasDirectRestirHistory = false;
        _hasIndirectRestirHistory = false;
        _lastDirectReservoirOutputIdx = 0;
        _lastIndirectReservoirOutputIdx = 0;
    }

}
