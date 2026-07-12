using System;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.SceneManagement;
#if UNITY_EDITOR
using UnityEditor;
#endif

[RequireComponent(typeof(LightManager))]
[RequireComponent(typeof(LightCullingManager))]
public class Tracing : MonoBehaviour
{
    public ComputeShader tracingShader;

    private Camera cam;
    private RenderTexture target;

    [Header("Skybox Settings")]
    [SerializeField]
    private Texture skyboxTexture;
    [SerializeField, Range(0.0f, 10.0f)]
    float SkyboxIntensity = 1.0f;
    [SerializeField, Range(1.0f, 200.0f)]
    float SunFocus = 5.0f;
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
    [SerializeField] bool Denoise = true;
    private bool _OldDenoise = true;

    [Header("Display")]
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
    private LightCullingManager lightCullingManager;

    // Multi-pass kernel indices
    private int kernelGenerate;
    private int kernelTrace;
    private int kernelShade;
    private int kernelShadow;
    private int kernelFinalize;
    private int kernelCopyPrimarySurfaceHistory;
    private int kernelTransfer;
    private int kernelPrepareLights;
    private int kernelBuildLightCdf;
    private int kernelGenerateInitial;
    private int kernelTemporalResampling;
    private int kernelShadeDISamples;
    private int kernelGenerateGISecondarySurfaces;
    private int kernelShadeGISecondarySurfaces;
    private int kernelTemporalGIResampling;
    private int kernelSpatialGIResampling;
    private int kernelShadeGISamples;
    private int kernelClearReSTIRTelemetry;

    // Multi-pass buffers
    private ComputeBuffer _globalRaysA;
    private ComputeBuffer _globalRaysB;
    private ComputeBuffer _globalHits;
    private ComputeBuffer _primarySurfaceHistory;
    private ComputeBuffer _primarySurfaceHistoryPrev;
    private ComputeBuffer _shadowRays;
    private ComputeBuffer _directLightReservoirs;
    private ComputeBuffer _indirectReservoirs;
    private ComputeBuffer _secondarySurfaces;
    private ComputeBuffer _restirDebugData;
    private ComputeBuffer _restirTelemetry;
    private ComputeBuffer _lightDataPacked;
    private int _lastDirectReservoirOutputIdx = 0;
    private int _lastIndirectReservoirOutputIdx = 0;
    private int _restirShadingReservoirIdx = 0;
    private bool _hasDirectRestirHistory = false;
    private bool _hasIndirectRestirHistory = false;
    private ComputeBuffer _globalColors;
    private ComputeBuffer _bufferSizes;
    private ComputeBuffer _indirectArgs;

    // Struct sizes (must match HLSL layout)
    private const int RayDataStride = 48;         // float3+float3+uint+uint+float3+float = 12+12+4+4+12+4
    private const int HitDataStride = 76;          // 4×(float3+scalar) + 2×float + float = 4×16+8+4
    private const int ShadowRayDataStride = 48;    // 3×(float3+scalar)
    private const int DirectLightReservoirStride = 80; // matches DirectLightReservoirData (5 float4 rows)
    private const int IndirectReservoirStride = 80; // matches IndirectReservoirData (5 float4 rows)
    private const int SecondarySurfaceStride = 96; // matches SecondarySurfaceData (6 float4 rows)
    private const int RestirDebugDataStride = 16; // float4
    private const int RestirDebugDataCount = 5;
    private const int ReSTIRTelemetrySampleStride = 64;
    private const int PathContributionStride = 32; // two float3 lanes plus explicit HLSL padding
    private const int BufferSizeDataStride = 8;    // int+int = 4+4
    private const int IndirectArgsStride = 4;      // uint x3 = 3 elements x 4 bytes each

    private int prevWidth, prevHeight, prevTraceDepth;
    private int _currentRenderWidth;
    private int _currentRenderHeight;

    // Cached arrays and objects to avoid per-frame GC allocations
    private int[] bvhKernels;
    private int[] lightKernels;
    private CommandBuffer cmdBuffer;
    private int[] sizesData = new int[32]; // Max 15 bounces: (15+1)*2 = 32
    private string[] bounceNames = new string[24]; // 8 bounces × 3 phases = 24
    private int _lastLightStateHash = int.MinValue;
    private bool _oldToneMap = true;
    private float _oldExposure = 1.0f;
    private float _oldSkyboxIntensity = 1.0f;
    private float _oldSunFocus = 5.0f;
    private float _oldSunAngularRadius = 0.1f;
    private int _oldTraceDepth = 3;
    private int _oldTargetFrameRate = 90;
    private int _oldFrameLimit = 0;
    private int _oldDirectLightRISCandidateCount = 1;
    private bool _oldUseReSTIRDI = false;
    private bool _oldUseReSTIRGI = false;
    private bool _hasPrimarySurfaceHistory = false;
    private Matrix4x4 _previousCameraViewProjection = Matrix4x4.identity;
    private int _previousTargetFrameRate = -1;
    private int _previousVSyncCount = -1;
    private int _previousRenderFrameInterval = -1;
    private bool _hasCapturedFrameRateState = false;
    private bool _warnedAboutReSTIRGIWithoutDI = false;
    private ReSTIRDiagnosticsSession _restirDiagnostics;
    private int[] _restirTelemetryKernels;
    private bool _captureReSTIRTelemetryThisFrame;
    private bool _runtimeStarted;
#if UNITY_EDITOR
    private static Type _editorGameViewType;
    private static PropertyInfo _editorGameViewVSyncProperty;
    private static readonly Dictionary<EditorWindow, bool> _editorGameViewVSyncStates = new Dictionary<EditorWindow, bool>();
#endif

    private void Awake()
    {
        EnsureMaterials();
    }

    private void OnEnable()
    {
        if (!Application.isPlaying)
            return;

        CaptureFrameRateLimit();
        ApplyFrameRateLimit();
        if (_runtimeStarted && _restirDiagnostics == null)
            StartReSTIRDiagnostics();
    }

    private void Start()
    {
        cam = GetComponent<Camera>();
        lightCullingManager = GetComponent<LightCullingManager>();
        LightManager.Instance.UpdateLights();
        _OldDenoise = Denoise;

        // Find kernel indices
        kernelGenerate = tracingShader.FindKernel("kernel_generate");
        kernelTrace = tracingShader.FindKernel("kernel_trace");
        kernelShade = tracingShader.FindKernel("kernel_shade");
        kernelShadow = tracingShader.FindKernel("kernel_shadow");
        kernelFinalize = tracingShader.FindKernel("kernel_finalize");
        kernelCopyPrimarySurfaceHistory = tracingShader.FindKernel("kernel_copy_primary_surface_history");
        kernelTransfer = tracingShader.FindKernel("TransferKernel");
        kernelPrepareLights = tracingShader.FindKernel("kernel_prepare_lights");
        kernelBuildLightCdf = tracingShader.FindKernel("kernel_build_light_cdf");
        kernelGenerateInitial = tracingShader.FindKernel("kernel_generate_initial");
        kernelTemporalResampling = tracingShader.FindKernel("kernel_temporal_resampling");
        kernelShadeDISamples = tracingShader.FindKernel("kernel_shade_di_samples");
        kernelGenerateGISecondarySurfaces = tracingShader.FindKernel("kernel_generate_gi_secondary_surfaces");
        kernelShadeGISecondarySurfaces = tracingShader.FindKernel("kernel_shade_gi_secondary_surfaces");
        kernelTemporalGIResampling = tracingShader.FindKernel("kernel_temporal_gi_resampling");
        kernelSpatialGIResampling = tracingShader.FindKernel("kernel_spatial_gi_resampling");
        kernelShadeGISamples = tracingShader.FindKernel("kernel_shade_gi_samples");
        kernelClearReSTIRTelemetry = tracingShader.FindKernel("kernel_clear_restir_telemetry");

        // Pre-allocate reusable arrays and names
        bvhKernels = new int[]
        {
            kernelGenerate,
            kernelTrace,
            kernelShade,
            kernelShadow,
            kernelGenerateInitial,
            kernelTemporalResampling,
            kernelShadeDISamples,
            kernelGenerateGISecondarySurfaces,
            kernelShadeGISecondarySurfaces,
            kernelTemporalGIResampling,
            kernelSpatialGIResampling,
            kernelShadeGISamples
        };
        lightKernels = new int[]
        {
            kernelGenerate,
            kernelTrace,
            kernelShade,
            kernelGenerateInitial,
            kernelShadeDISamples,
            kernelGenerateGISecondarySurfaces,
            kernelShadeGISecondarySurfaces,
            kernelSpatialGIResampling,
            kernelShadeGISamples
        };
        _restirTelemetryKernels = new int[]
        {
            kernelClearReSTIRTelemetry,
            kernelGenerateInitial,
            kernelTemporalResampling,
            kernelShadeDISamples,
            kernelGenerateGISecondarySurfaces,
            kernelShadeGISecondarySurfaces,
            kernelTemporalGIResampling,
            kernelSpatialGIResampling,
            kernelShadeGISamples
        };
        cmdBuffer = new CommandBuffer();
        CacheRuntimeSettings();
        _lastLightStateHash = LightManager.Instance.ComputeLightStateHash();
        for (int i = 0; i < bounceNames.Length; i++)
        {
            int bounce = i / 3;
            int phase = i % 3;
            bounceNames[i] = "PT_B" + bounce + "_" + (phase == 0 ? "Trace" : phase == 1 ? "Shade" : "Shadow");
        }
        _runtimeStarted = true;
        StartReSTIRDiagnostics();
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

    private void CreateBuffersIfNeeded(int width, int height)
    {
        if (width == prevWidth && height == prevHeight && TraceDepth == prevTraceDepth) return;

        ReleaseBuffers();
        _hasPrimarySurfaceHistory = false;

        int pixelCount = width * height;

        _globalRaysA = new ComputeBuffer(pixelCount, RayDataStride);
        _globalRaysB = new ComputeBuffer(pixelCount, RayDataStride);
        _globalHits = new ComputeBuffer(pixelCount, HitDataStride);
        _primarySurfaceHistory = new ComputeBuffer(pixelCount, HitDataStride);
        _primarySurfaceHistoryPrev = new ComputeBuffer(pixelCount, HitDataStride);
        // Keep headroom for future direct-light candidates/reservoir experiments.
        _shadowRays = new ComputeBuffer(pixelCount * 2, ShadowRayDataStride);
        _directLightReservoirs = new ComputeBuffer(pixelCount * 3, DirectLightReservoirStride);
        _indirectReservoirs = new ComputeBuffer(pixelCount * 3, IndirectReservoirStride);
        _secondarySurfaces = new ComputeBuffer(pixelCount, SecondarySurfaceStride);
        _restirDebugData = new ComputeBuffer(RestirDebugDataCount, RestirDebugDataStride);
        _globalColors = new ComputeBuffer(pixelCount, PathContributionStride);
        _bufferSizes = new ComputeBuffer(TraceDepth + 1, BufferSizeDataStride);
        _indirectArgs = new ComputeBuffer(3, IndirectArgsStride, ComputeBufferType.IndirectArguments);

        int maxLights = Mathf.Max(LightManager.Instance?.GetPointLightsCount() ?? 0, 1) + 1;
        _lightDataPacked?.Release();
        _lightDataPacked = new ComputeBuffer(maxLights * 2, sizeof(float) * 16);

        _hasDirectRestirHistory = false;
        _hasIndirectRestirHistory = false;
        _lastDirectReservoirOutputIdx = 0;
        _lastIndirectReservoirOutputIdx = 0;

        prevWidth = width;
        prevHeight = height;
        prevTraceDepth = TraceDepth;
    }

    private void ReleaseBuffers()
    {
        _globalRaysA?.Release(); _globalRaysA = null;
        _globalRaysB?.Release(); _globalRaysB = null;
        _globalHits?.Release(); _globalHits = null;
        _primarySurfaceHistory?.Release(); _primarySurfaceHistory = null;
        _primarySurfaceHistoryPrev?.Release(); _primarySurfaceHistoryPrev = null;
        _shadowRays?.Release(); _shadowRays = null;
        _directLightReservoirs?.Release(); _directLightReservoirs = null;
        _indirectReservoirs?.Release(); _indirectReservoirs = null;
        _secondarySurfaces?.Release(); _secondarySurfaces = null;
        _restirDebugData?.Release(); _restirDebugData = null;
        _globalColors?.Release(); _globalColors = null;
        _bufferSizes?.Release(); _bufferSizes = null;
        _indirectArgs?.Release(); _indirectArgs = null;
        _lightDataPacked?.Release(); _lightDataPacked = null;
        _hasDirectRestirHistory = false;
        _hasIndirectRestirHistory = false;
        _lastDirectReservoirOutputIdx = 0;
        _lastIndirectReservoirOutputIdx = 0;
        prevWidth = 0;
        prevHeight = 0;
        prevTraceDepth = 0;
    }

    private void Render(RenderTexture source, RenderTexture destination)
    {
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
            if (target != null) target.Release();
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
            if (frameConverged != null)
                frameConverged.Release();
            frameConverged = new RenderTexture(renderDimensions.x, renderDimensions.y, 0, RenderTextureFormat.ARGBFloat, RenderTextureReadWrite.Linear);
            frameConverged.filterMode = FilterMode.Point;
            frameConverged.wrapMode   = TextureWrapMode.Clamp;
            frameConverged.enableRandomWrite = true;
            frameConverged.Create();
        }

        bool sceneChanged = BVHBuilder.Validate();
        bool cameraMoved = Camera.main != null && Camera.main.transform.hasChanged;
        if (sceneChanged)
        {
            ResetSampleCount("scene_changed");
        }
        else if (cameraMoved)
        {
            ResetAccumulationOnly("camera_moved");
        }

        if (Camera.main != null)
            Camera.main.transform.hasChanged = false;

        CreateBuffersIfNeeded(renderDimensions.x, renderDimensions.y);
        if (FrameLimit > 0 && sampleCount >= FrameLimit)
        {
            if (Denoise)
                BlitToDisplay(frameConverged, destination);
            else
                BlitToDisplay(target, destination);
            return;
        }
        // 执行光源剔除
        if (lightCullingManager != null)
        {
            lightCullingManager.PerformLightCulling();
        }

        SetShaderParameters();
        sampleCount++;

        int pixelCount = _currentRenderWidth * _currentRenderHeight;

        // Initialize BufferSizes: [0].traceRays = pixelCount, rest = 0
        int sizeCount = (TraceDepth + 1) * 2;
        if (sizesData.Length < sizeCount)
            System.Array.Resize(ref sizesData, sizeCount);
        System.Array.Clear(sizesData, 0, sizeCount);
        sizesData[0] = pixelCount; // [0].traceRays
        _bufferSizes.SetData(sizesData, 0, 0, sizeCount);

        // 1. Generate primary rays
        tracingShader.Dispatch(kernelGenerate, (pixelCount + 63) / 64, 1, 1);

        // 2. Per-bounce loop (skip when debug modes set throughput=0)
        bool debugMode = OnlyDrawAlbedo || OnlyDrawNormals || OnlyDrawDepth;

        if (!debugMode)
        {
            WarnIfReSTIRDirectLightingConfigurationIsRisky();

            tracingShader.SetInt("CurBounce", 0);
            tracingShader.SetBuffer(kernelTrace, "GlobalRays", _globalRaysA);
            tracingShader.SetBuffer(kernelTrace, "GlobalHits", _globalHits);
            tracingShader.Dispatch(kernelTrace, (pixelCount + 63) / 64, 1, 1);

            BeginReSTIRTelemetryCapture();

            if (UseReSTIRDI)
                DispatchReSTIRDI(pixelCount);

            if (UseReSTIRGI)
                DispatchReSTIRGI(pixelCount);

            EndReSTIRTelemetryCapture();

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
        }

        // 3. Finalize
        tracingShader.Dispatch(kernelFinalize,
            Mathf.CeilToInt(_currentRenderWidth / 8.0f),
            Mathf.CeilToInt(_currentRenderHeight / 8.0f), 1);

        tracingShader.Dispatch(kernelCopyPrimarySurfaceHistory, (pixelCount + 63) / 64, 1, 1);

        if (Denoise)
        {
            _addMaterial.SetFloat("_Sample", sampleCount);
            Graphics.Blit(target, frameConverged, _addMaterial);
            BlitToDisplay(frameConverged, destination);
        }
        else
        {
            BlitToDisplay(target, destination);
        }

        _hasPrimarySurfaceHistory = true;
    }

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
            UseReSTIRGI,
            Denoise);

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
        if ((!UseReSTIRDI && !UseReSTIRGI) || _restirDiagnostics == null || _restirTelemetry == null)
            return;

        BindReSTIRTelemetryBuffers();

        int directInitialSlot = UseReSTIRDI ? (_lastDirectReservoirOutputIdx + 1) % 3 : -1;
        int directFinalSlot = UseReSTIRDI
            ? (_hasDirectRestirHistory ? (_lastDirectReservoirOutputIdx + 2) % 3 : directInitialSlot)
            : -1;
        int indirectInitialSlot = UseReSTIRGI ? (_lastIndirectReservoirOutputIdx + 1) % 3 : -1;
        int indirectFinalSlot = UseReSTIRGI ? _lastIndirectReservoirOutputIdx : -1;

        _captureReSTIRTelemetryThisFrame = _restirDiagnostics.BeginCapture(
            (int)frameId,
            sampleCount,
            _currentRenderWidth,
            _currentRenderHeight,
            UseReSTIRDI,
            UseReSTIRGI,
            directInitialSlot,
            directFinalSlot,
            indirectInitialSlot,
            indirectFinalSlot);
        if (!_captureReSTIRTelemetryThisFrame)
            return;

        int modeFlags = (UseReSTIRDI ? 1 : 0) |
            (UseReSTIRGI ? 2 : 0) |
            (_hasDirectRestirHistory ? 4 : 0) |
            (_hasIndirectRestirHistory ? 8 : 0) |
            (Denoise ? 16 : 0);
        int telemetrySampleStride = WriteReSTIRGIDiagnosticDetails ? 16 : ReSTIRTelemetrySampleStride;
        tracingShader.SetInt("_RestirTelemetryEnabled", 1);
        tracingShader.SetInt("_RestirTelemetrySampleStride", telemetrySampleStride);
        tracingShader.SetInt("_RestirTelemetrySamplePhase", sampleCount % telemetrySampleStride);
        tracingShader.SetInt("_RestirTelemetryGeneration", _restirDiagnostics.Generation);
        tracingShader.SetInt("_RestirTelemetrySampleCount", sampleCount);
        tracingShader.SetInt("_RestirTelemetryModeFlags", modeFlags);
        tracingShader.SetInt("_RestirTelemetryDirectInitialSlot", directInitialSlot);
        tracingShader.SetInt("_RestirTelemetryDirectFinalSlot", directFinalSlot);
        tracingShader.SetInt("_RestirTelemetryIndirectInitialSlot", indirectInitialSlot);
        tracingShader.SetInt("_RestirTelemetryIndirectFinalSlot", indirectFinalSlot);
        BindRestirCommonParams(kernelClearReSTIRTelemetry);
        tracingShader.Dispatch(
            kernelClearReSTIRTelemetry,
            (ReSTIRTelemetryLayout.PacketWordCount + 63) / 64,
            1,
            1);
    }

    private void EndReSTIRTelemetryCapture()
    {
        if (!_captureReSTIRTelemetryThisFrame)
            return;

        _restirDiagnostics.RequestCapture(_restirTelemetry);
        _captureReSTIRTelemetryThisFrame = false;
        tracingShader.SetInt("_RestirTelemetryEnabled", 0);
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

    private void DispatchReSTIRDI(int pixelCount)
    {
        if (!UseReSTIRDI) return;

        int initialIdx = (_lastDirectReservoirOutputIdx + 1) % 3;
        int temporalIdx = (_lastDirectReservoirOutputIdx + 2) % 3;
        int prevIdx = _lastDirectReservoirOutputIdx;

        int lightCount = 1 + LightManager.Instance.GetPointLightsCount();
        int maxLightSlots = _lightDataPacked != null ? _lightDataPacked.count : 0;
        lightCount = Mathf.Min(lightCount, maxLightSlots);

        BindRestirCommonParams(kernelPrepareLights);
        BindSceneBuffersToKernel(kernelPrepareLights);
        tracingShader.SetBuffer(kernelPrepareLights, "_LightDataPacked", _lightDataPacked);
        tracingShader.SetInt("_LightDataPackedCount", lightCount);
        tracingShader.Dispatch(kernelPrepareLights, (lightCount + 63) / 64, 1, 1);

        tracingShader.SetBuffer(kernelBuildLightCdf, "_LightDataPacked", _lightDataPacked);
        tracingShader.SetInt("_LightDataPackedCount", lightCount);
        tracingShader.Dispatch(kernelBuildLightCdf, 1, 1, 1);

        BindRestirCommonParams(kernelGenerateInitial);
        BindSceneBuffersToKernel(kernelGenerateInitial);
        tracingShader.SetBuffer(kernelGenerateInitial, "DirectLightReservoirs", _directLightReservoirs);
        tracingShader.SetBuffer(kernelGenerateInitial, "_RestirGbuffer", _globalHits);
        tracingShader.SetBuffer(kernelGenerateInitial, "_RestirLightData", _lightDataPacked);
        tracingShader.SetInt("_RestirLightCount", lightCount);
        tracingShader.SetInt("_RestirInitialReservoirOffset", initialIdx * pixelCount);
        tracingShader.SetInt("_RestirCandidateCount", DirectLightRISCandidateCount);
        tracingShader.Dispatch(kernelGenerateInitial, (pixelCount + 63) / 64, 1, 1);

        bool useTemporal = _hasDirectRestirHistory;
        if (useTemporal)
        {
            BindRestirCommonParams(kernelTemporalResampling);
            BindSceneBuffersToKernel(kernelTemporalResampling);
            tracingShader.SetBuffer(kernelTemporalResampling, "DirectLightReservoirs", _directLightReservoirs);
            tracingShader.SetBuffer(kernelTemporalResampling, "ReSTIRDebugData", _restirDebugData);
            tracingShader.SetInt("_RestirInitialReservoirOffset", initialIdx * pixelCount);
            tracingShader.SetInt("_RestirTemporalReservoirOffset", temporalIdx * pixelCount);
            tracingShader.SetInt("_RestirPrevReservoirOffset", prevIdx * pixelCount);
            tracingShader.SetBuffer(kernelTemporalResampling, "_RestirGbuffer", _globalHits);
            tracingShader.SetBuffer(kernelTemporalResampling, "_RestirGbufferPrevious", _primarySurfaceHistoryPrev);
            tracingShader.SetInt("_RestirDebugPixelIndex", 0);
            tracingShader.Dispatch(kernelTemporalResampling, (pixelCount + 63) / 64, 1, 1);
        }

        int shadingReservoirIdx = useTemporal ? temporalIdx : initialIdx;

        BindRestirCommonParams(kernelShadeDISamples);
        BindSceneBuffersToKernel(kernelShadeDISamples);
        tracingShader.SetBuffer(kernelShadeDISamples, "DirectLightReservoirs", _directLightReservoirs);
        tracingShader.SetBuffer(kernelShadeDISamples, "GlobalColors", _globalColors);
        tracingShader.SetInt("_RestirShadingReservoirOffset", shadingReservoirIdx * pixelCount);
        tracingShader.Dispatch(kernelShadeDISamples, (pixelCount + 63) / 64, 1, 1);

        _lastDirectReservoirOutputIdx = shadingReservoirIdx;
        _restirShadingReservoirIdx = shadingReservoirIdx;
        _hasDirectRestirHistory = true;
    }

    private void DispatchReSTIRGI(int pixelCount)
    {
        int initialIdx = (_lastIndirectReservoirOutputIdx + 1) % 3;
        int temporalIdx = (_lastIndirectReservoirOutputIdx + 2) % 3;
        int prevIdx = _lastIndirectReservoirOutputIdx;
        int spatialIdx = prevIdx;

        BindRestirCommonParams(kernelGenerateGISecondarySurfaces);
        BindSceneBuffersToKernel(kernelGenerateGISecondarySurfaces);
        tracingShader.SetBuffer(kernelGenerateGISecondarySurfaces, "_RestirGbuffer", _globalHits);
        tracingShader.SetBuffer(kernelGenerateGISecondarySurfaces, "SecondarySurfaces", _secondarySurfaces);
        tracingShader.SetBuffer(kernelGenerateGISecondarySurfaces, "SecondarySurfacesRead", _secondarySurfaces);
        tracingShader.Dispatch(kernelGenerateGISecondarySurfaces, (pixelCount + 63) / 64, 1, 1);

        BindRestirCommonParams(kernelShadeGISecondarySurfaces);
        BindSceneBuffersToKernel(kernelShadeGISecondarySurfaces);
        tracingShader.SetBuffer(kernelShadeGISecondarySurfaces, "IndirectReservoirs", _indirectReservoirs);
        tracingShader.SetBuffer(kernelShadeGISecondarySurfaces, "SecondarySurfaces", _secondarySurfaces);
        tracingShader.SetBuffer(kernelShadeGISecondarySurfaces, "IndirectReservoirsRead", _indirectReservoirs);
        tracingShader.SetBuffer(kernelShadeGISecondarySurfaces, "SecondarySurfacesRead", _secondarySurfaces);
        tracingShader.SetBuffer(kernelShadeGISecondarySurfaces, "GlobalColors", _globalColors);
        tracingShader.SetBuffer(kernelShadeGISecondarySurfaces, "ReSTIRDebugData", _restirDebugData);
        tracingShader.SetBuffer(kernelShadeGISecondarySurfaces, "_RestirGbuffer", _globalHits);
        tracingShader.SetInt("_RestirInitialReservoirOffset", initialIdx * pixelCount);
        tracingShader.SetInt("_RestirDebugPixelIndex", 0);
        tracingShader.Dispatch(kernelShadeGISecondarySurfaces, (pixelCount + 63) / 64, 1, 1);

        bool useTemporal = _hasIndirectRestirHistory;
        if (useTemporal)
        {
            BindRestirCommonParams(kernelTemporalGIResampling);
            BindSceneBuffersToKernel(kernelTemporalGIResampling);
            tracingShader.SetBuffer(kernelTemporalGIResampling, "IndirectReservoirs", _indirectReservoirs);
            tracingShader.SetBuffer(kernelTemporalGIResampling, "IndirectReservoirsRead", _indirectReservoirs);
            tracingShader.SetBuffer(kernelTemporalGIResampling, "ReSTIRDebugData", _restirDebugData);
            tracingShader.SetBuffer(kernelTemporalGIResampling, "_RestirGbuffer", _globalHits);
            tracingShader.SetBuffer(kernelTemporalGIResampling, "_RestirGbufferPrevious", _primarySurfaceHistoryPrev);
            tracingShader.SetInt("_RestirInitialReservoirOffset", initialIdx * pixelCount);
            tracingShader.SetInt("_RestirTemporalReservoirOffset", temporalIdx * pixelCount);
            tracingShader.SetInt("_RestirPrevReservoirOffset", prevIdx * pixelCount);
            tracingShader.SetInt("_RestirDebugPixelIndex", 0);
            tracingShader.Dispatch(kernelTemporalGIResampling, (pixelCount + 63) / 64, 1, 1);
        }

        int shadingReservoirIdx = useTemporal ? temporalIdx : initialIdx;

        BindRestirCommonParams(kernelSpatialGIResampling);
        BindSceneBuffersToKernel(kernelSpatialGIResampling);
        tracingShader.SetBuffer(kernelSpatialGIResampling, "IndirectReservoirs", _indirectReservoirs);
        tracingShader.SetBuffer(kernelSpatialGIResampling, "ReSTIRDebugData", _restirDebugData);
        tracingShader.SetBuffer(kernelSpatialGIResampling, "_RestirGbuffer", _globalHits);
        tracingShader.SetInt("_RestirShadingReservoirOffset", shadingReservoirIdx * pixelCount);
        tracingShader.SetInt("_RestirSpatialReservoirOffset", spatialIdx * pixelCount);
        tracingShader.SetInt("_RestirDebugPixelIndex", 0);
        tracingShader.Dispatch(kernelSpatialGIResampling, (pixelCount + 63) / 64, 1, 1);

        shadingReservoirIdx = spatialIdx;

        BindRestirCommonParams(kernelShadeGISamples);
        BindSceneBuffersToKernel(kernelShadeGISamples);
        tracingShader.SetBuffer(kernelShadeGISamples, "IndirectReservoirs", _indirectReservoirs);
        tracingShader.SetBuffer(kernelShadeGISamples, "IndirectReservoirsRead", _indirectReservoirs);
        tracingShader.SetBuffer(kernelShadeGISamples, "GlobalColors", _globalColors);
        tracingShader.SetBuffer(kernelShadeGISamples, "ReSTIRDebugData", _restirDebugData);
        tracingShader.SetBuffer(kernelShadeGISamples, "_RestirGbuffer", _globalHits);
        tracingShader.SetInt("_RestirInitialReservoirOffset", initialIdx * pixelCount);
        tracingShader.SetInt("_RestirShadingReservoirOffset", shadingReservoirIdx * pixelCount);
        tracingShader.SetInt("_RestirDebugPixelIndex", 0);
        tracingShader.Dispatch(kernelShadeGISamples, (pixelCount + 63) / 64, 1, 1);

        _lastIndirectReservoirOutputIdx = shadingReservoirIdx;
        _hasIndirectRestirHistory = true;
    }

    private void BindRestirCommonParams(int kernel)
    {
        tracingShader.SetInt("_ScreenWidth", _currentRenderWidth);
        tracingShader.SetInt("_ScreenHeight", _currentRenderHeight);
        tracingShader.SetInt("_FrameCount", (int)frameId);
    }

    private void BindSceneBuffersToKernel(int kernel)
    {
        if (BVHBuilder.VertexBuffer != null) tracingShader.SetBuffer(kernel, "_Vertices", BVHBuilder.VertexBuffer);
        if (BVHBuilder.IndexBuffer != null) tracingShader.SetBuffer(kernel, "_Indices", BVHBuilder.IndexBuffer);
        if (BVHBuilder.NormalBuffer != null) tracingShader.SetBuffer(kernel, "_Normals", BVHBuilder.NormalBuffer);
        if (BVHBuilder.TangentBuffer != null) tracingShader.SetBuffer(kernel, "_Tangents", BVHBuilder.TangentBuffer);
        if (BVHBuilder.UVBuffer != null) tracingShader.SetBuffer(kernel, "_UVs", BVHBuilder.UVBuffer);
        if (BVHBuilder.MaterialBuffer != null) tracingShader.SetBuffer(kernel, "_Materials", BVHBuilder.MaterialBuffer);
        if (BVHBuilder.MeshNodeBuffer != null)
        {
            tracingShader.SetInt("_TLASNodesCount", BVHBuilder.GetTLASNodes().Count);
            tracingShader.SetBuffer(kernel, "_TLASNodes", BVHBuilder.MeshNodeBuffer);
        }
        if (BVHBuilder.BLASBuffer != null)
        {
            tracingShader.SetBuffer(kernel, "_BNodes", BVHBuilder.BLASBuffer);
            tracingShader.SetInt("_BNodesCount", BVHBuilder.GetBLASNodes().Count);
        }
        if (BVHBuilder.TransformBuffer != null) tracingShader.SetBuffer(kernel, "_Transforms", BVHBuilder.TransformBuffer);
        tracingShader.SetBuffer(kernel, "_PointLights", LightManager.Instance.pointLightsBuffer);
        tracingShader.SetInt("_PointLightsCount", LightManager.Instance.GetPointLightsCount());
        if (BVHBuilder.AlbedoTextures != null) tracingShader.SetTexture(kernel, "_AlbedoTextures", BVHBuilder.AlbedoTextures);
        if (BVHBuilder.EmissionTextures != null) tracingShader.SetTexture(kernel, "_EmitTextures", BVHBuilder.EmissionTextures);
        if (BVHBuilder.MetallicTextures != null) tracingShader.SetTexture(kernel, "_MetallicTextures", BVHBuilder.MetallicTextures);
        if (BVHBuilder.NormalTextures != null) tracingShader.SetTexture(kernel, "_NormalTextures", BVHBuilder.NormalTextures);
        if (BVHBuilder.RoughnessTextures != null) tracingShader.SetTexture(kernel, "_RoughnessTextures", BVHBuilder.RoughnessTextures);
    }



    private void WarnIfReSTIRDirectLightingConfigurationIsRisky()
    {
        if (UseReSTIRGI && !UseReSTIRDI)
        {
            if (!_warnedAboutReSTIRGIWithoutDI)
            {
                Debug.LogWarning("ReSTIR GI is enabled while ReSTIR DI is disabled. Primary-bounce direct lighting now depends on the non-ReSTIR path being skipped, so dark or flickering direct-light frames are expected in this configuration.");
                _warnedAboutReSTIRGIWithoutDI = true;
            }
        }
        else
        {
            _warnedAboutReSTIRGIWithoutDI = false;
        }
    }
























    private void Update()
    {
        bool resetRequired = false;
        string runtimeStateChangeReason = null;

        if (_OldDenoise != Denoise)
        {
            _OldDenoise = Denoise;
            resetRequired = true;
            runtimeStateChangeReason = runtimeStateChangeReason ?? "denoise_toggled";
        }

        LightManager.Instance.UpdateLights();
        int lightStateHash = LightManager.Instance.ComputeLightStateHash();
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
                else if (_oldToneMap != ToneMap || !Mathf.Approximately(_oldExposure, Exposure))
                    runtimeStateChangeReason = "display_settings_changed";
                else
                    runtimeStateChangeReason = "runtime_settings_changed";
            }
            CacheRuntimeSettings();
            resetRequired = true;
        }

        if (resetRequired)
            ResetSampleCount(runtimeStateChangeReason ?? "runtime_reset");

        LightManager.Instance.UpdateBuffer(tracingShader, lightKernels);
    }

    private void OnValidate()
    {
        if (!Application.isPlaying)
            return;

        ApplyFrameRateLimit();
    }

    private uint frameId = 0;

    private void SetShaderParameters()
    {
        tracingShader.SetInt("_FrameCount", (int)frameId++);

        // Per-pixel jitter for temporal AA (used by GenRayByID)
        float jx = UnityEngine.Random.value - 0.5f;
        float jy = UnityEngine.Random.value - 0.5f;
        tracingShader.SetVector("_PixelOffset", new Vector2(jx, jy));

        tracingShader.SetVector("_Resolution", new Vector2(_currentRenderWidth, _currentRenderHeight));
        tracingShader.SetInt("_TraceDepth", TraceDepth);
        tracingShader.SetMatrix("_CameraToWorld", cam.cameraToWorldMatrix);
        Matrix4x4 gpuProjection = GL.GetGPUProjectionMatrix(cam.projectionMatrix, false);
        tracingShader.SetMatrix("_CameraInverseProjection", cam.projectionMatrix.inverse);
        tracingShader.SetMatrix("_PreviousCameraViewProjection", _previousCameraViewProjection);
        tracingShader.SetMatrix("_RestirPreviousViewProjection", _previousCameraViewProjection);
        tracingShader.SetFloat("_SunFocus", SunFocus);
        tracingShader.SetFloat("_SunAngularRadius", SunAngularRadius);
        tracingShader.SetFloat("_SkyboxIntensity", SkyboxIntensity);

        // Screen dimensions for multi-pass
        tracingShader.SetInt("_ScreenWidth", _currentRenderWidth);
        tracingShader.SetInt("_ScreenHeight", _currentRenderHeight);

        // Set texture on all kernels that use _Result
        tracingShader.SetTexture(kernelGenerate, "_Result", target);
        tracingShader.SetTexture(kernelFinalize, "_Result", target);

        // Set skybox on kernels that need it
        tracingShader.SetTexture(kernelGenerate, "_SkyboxTexture", skyboxTexture);
        tracingShader.SetTexture(kernelTrace, "_SkyboxTexture", skyboxTexture);
        tracingShader.SetTexture(kernelShade, "_SkyboxTexture", skyboxTexture);
        tracingShader.SetTexture(kernelGenerateGISecondarySurfaces, "_SkyboxTexture", skyboxTexture);
        tracingShader.SetTexture(kernelShadeGISecondarySurfaces, "_SkyboxTexture", skyboxTexture);
        tracingShader.SetTexture(kernelShadeGISamples, "_SkyboxTexture", skyboxTexture);

        // Set multi-pass buffers on all relevant kernels
        tracingShader.SetBuffer(kernelGenerate, "GlobalRays", _globalRaysA);
        tracingShader.SetBuffer(kernelGenerate, "GlobalColors", _globalColors);
        tracingShader.SetBuffer(kernelGenerate, "GlobalHits", _globalHits);
        tracingShader.SetBuffer(kernelGenerate, "PrimarySurfaceHistory", _primarySurfaceHistory);
        tracingShader.SetBuffer(kernelGenerate, "PrimarySurfaceHistoryPrev", _primarySurfaceHistoryPrev);
        tracingShader.SetBuffer(kernelGenerate, "DirectLightReservoirs", _directLightReservoirs);
        tracingShader.SetBuffer(kernelGenerate, "IndirectReservoirs", _indirectReservoirs);
        tracingShader.SetBuffer(kernelGenerate, "SecondarySurfaces", _secondarySurfaces);
        tracingShader.SetBuffer(kernelTrace, "BufferSizes", _bufferSizes);
        tracingShader.SetBuffer(kernelShade, "GlobalColors", _globalColors);
        tracingShader.SetBuffer(kernelShade, "PrimarySurfaceHistory", _primarySurfaceHistory);
        tracingShader.SetBuffer(kernelShade, "PrimarySurfaceHistoryPrev", _primarySurfaceHistoryPrev);
        tracingShader.SetBuffer(kernelShade, "ShadowRaysBuffer", _shadowRays);
        tracingShader.SetBuffer(kernelShade, "DirectLightReservoirs", _directLightReservoirs);
        tracingShader.SetBuffer(kernelShade, "IndirectReservoirs", _indirectReservoirs);
        tracingShader.SetBuffer(kernelShade, "SecondarySurfaces", _secondarySurfaces);
        tracingShader.SetBuffer(kernelShade, "BufferSizes", _bufferSizes);
        tracingShader.SetBuffer(kernelShadow, "ShadowRaysBuffer", _shadowRays);
        tracingShader.SetBuffer(kernelShadow, "GlobalColors", _globalColors);
        tracingShader.SetBuffer(kernelShadow, "DirectLightReservoirs", _directLightReservoirs);
        tracingShader.SetBuffer(kernelShadow, "BufferSizes", _bufferSizes);
        tracingShader.SetBuffer(kernelCopyPrimarySurfaceHistory, "PrimarySurfaceHistory", _primarySurfaceHistory);
        tracingShader.SetBuffer(kernelCopyPrimarySurfaceHistory, "PrimarySurfaceHistoryPrevRW", _primarySurfaceHistoryPrev);
        tracingShader.SetBuffer(kernelTransfer, "BufferSizes", _bufferSizes);
        tracingShader.SetBuffer(kernelTransfer, "IndirectArgs", _indirectArgs);
        tracingShader.SetBuffer(kernelFinalize, "GlobalColors", _globalColors);

        // Rebind scene buffers/textures every frame. Shader recompiles during play can
        // invalidate texture bindings even when the BVH itself did not change.
        foreach (int k in bvhKernels)
        {
            if (BVHBuilder.VertexBuffer != null) tracingShader.SetBuffer(k, "_Vertices", BVHBuilder.VertexBuffer);
            if (BVHBuilder.IndexBuffer != null) tracingShader.SetBuffer(k, "_Indices", BVHBuilder.IndexBuffer);
            if (BVHBuilder.NormalBuffer != null) tracingShader.SetBuffer(k, "_Normals", BVHBuilder.NormalBuffer);
            if (BVHBuilder.TangentBuffer != null) tracingShader.SetBuffer(k, "_Tangents", BVHBuilder.TangentBuffer);
            if (BVHBuilder.UVBuffer != null) tracingShader.SetBuffer(k, "_UVs", BVHBuilder.UVBuffer);
            if (BVHBuilder.MaterialBuffer != null) tracingShader.SetBuffer(k, "_Materials", BVHBuilder.MaterialBuffer);
            if (BVHBuilder.MeshNodeBuffer != null)
            {
                tracingShader.SetInt("_TLASNodesCount", BVHBuilder.GetTLASNodes().Count);
                tracingShader.SetBuffer(k, "_TLASNodes", BVHBuilder.MeshNodeBuffer);
            }
            if (BVHBuilder.BLASBuffer != null)
            {
                tracingShader.SetBuffer(k, "_BNodes", BVHBuilder.BLASBuffer);
                tracingShader.SetInt("_BNodesCount", BVHBuilder.GetBLASNodes().Count);
            }
            if (BVHBuilder.TransformBuffer != null) tracingShader.SetBuffer(k, "_Transforms", BVHBuilder.TransformBuffer);
            if (BVHBuilder.AlbedoTextures != null) tracingShader.SetTexture(k, "_AlbedoTextures", BVHBuilder.AlbedoTextures);
            if (BVHBuilder.EmissionTextures != null) tracingShader.SetTexture(k, "_EmitTextures", BVHBuilder.EmissionTextures);
            if (BVHBuilder.MetallicTextures != null) tracingShader.SetTexture(k, "_MetallicTextures", BVHBuilder.MetallicTextures);
            if (BVHBuilder.NormalTextures != null) tracingShader.SetTexture(k, "_NormalTextures", BVHBuilder.NormalTextures);
            if (BVHBuilder.RoughnessTextures != null) tracingShader.SetTexture(k, "_RoughnessTextures", BVHBuilder.RoughnessTextures);
        }

        tracingShader.SetBool("_OnlyDrawDepth", OnlyDrawDepth);
        tracingShader.SetBool("_OnlyDrawNormals", OnlyDrawNormals);
        tracingShader.SetBool("_OnlyDrawAlbedo", OnlyDrawAlbedo);
        tracingShader.SetBool("_HasPrimarySurfaceHistory", _hasPrimarySurfaceHistory);
        tracingShader.SetInt("_DirectLightRISCandidateCount", DirectLightRISCandidateCount);
        tracingShader.SetBool("_UseReSTIRDI", UseReSTIRDI);
        tracingShader.SetBool("_UseReSTIRGI", UseReSTIRGI);
        tracingShader.SetFloat("_CameraFar", cam.farClipPlane);

        // 设置光源缓冲区（绑定到所有需要的kernel）
        if (lightCullingManager != null)
        {
            lightCullingManager.SetTracingShaderBuffers(tracingShader, lightKernels);
        }

        // Bind current light data immediately before dispatch so rendering does not depend on Update() timing.
        LightManager.Instance.UpdateBuffer(tracingShader, lightKernels);
        _previousCameraViewProjection = gpuProjection * cam.worldToCameraMatrix;
    }

    private void OnDisable()
    {
        StopReSTIRDiagnostics();
        RestoreFrameRateLimit();
        ReleaseRenderTargets();
        ReleaseBuffers();
        ReleaseCommandBuffer();
        ReleaseMaterials();
        BVHBuilder.Destroy();
    }

    private void OnApplicationQuit()
    {
        StopReSTIRDiagnostics();
        RestoreFrameRateLimit();
        ReleaseRenderTargets();
        ReleaseBuffers();
        ReleaseCommandBuffer();
        ReleaseMaterials();
        BVHBuilder.Destroy();
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

    private void ReleaseRenderTargets()
    {
        if (target != null)
        {
            target.Release();
            target = null;
        }

        if (frameConverged != null)
        {
            frameConverged.Release();
            frameConverged = null;
        }

        _currentRenderWidth = 0;
        _currentRenderHeight = 0;
    }

    private void ReleaseMaterials()
    {
        if (_addMaterial != null)
        {
            Destroy(_addMaterial);
            _addMaterial = null;
        }

        if (_toneMapMaterial != null)
        {
            Destroy(_toneMapMaterial);
            _toneMapMaterial = null;
        }
    }

    private void EnsureMaterials()
    {
        if (_addMaterial == null)
            _addMaterial = new Material(Shader.Find("Hidden/AddShader"));
        if (_toneMapMaterial == null)
            _toneMapMaterial = new Material(Shader.Find("Hidden/ToneMapShader"));
    }

    private bool HaveRuntimeSettingsChanged()
    {
        return _oldToneMap != ToneMap ||
               _oldExposure != Exposure ||
               _oldSkyboxIntensity != SkyboxIntensity ||
               _oldSunFocus != SunFocus ||
               _oldSunAngularRadius != SunAngularRadius ||
               _oldTraceDepth != TraceDepth ||
               _oldTargetFrameRate != targetFrameRate ||
               _oldFrameLimit != FrameLimit ||
               _oldDirectLightRISCandidateCount != DirectLightRISCandidateCount ||
               _oldUseReSTIRDI != UseReSTIRDI ||
               _oldUseReSTIRGI != UseReSTIRGI;
    }

    private void CacheRuntimeSettings()
    {
        ApplyFrameRateLimit();
        _oldToneMap = ToneMap;
        _oldExposure = Exposure;
        _oldSkyboxIntensity = SkyboxIntensity;
        _oldSunFocus = SunFocus;
        _oldSunAngularRadius = SunAngularRadius;
        _oldTraceDepth = TraceDepth;
        _oldTargetFrameRate = targetFrameRate;
        _oldFrameLimit = FrameLimit;
        _oldDirectLightRISCandidateCount = DirectLightRISCandidateCount;
        _oldUseReSTIRDI = UseReSTIRDI;
        _oldUseReSTIRGI = UseReSTIRGI;
    }

    private void ReleaseCommandBuffer()
    {
        if (cmdBuffer == null) return;

        cmdBuffer.Release();
        cmdBuffer = null;
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

    private void ResetSampleCount(string reason = "full_reset")
    {
        _restirDiagnostics?.ScheduleResetCapture(reason, sampleCount);
        sampleCount = 0;
        frameId = 0;
        _hasPrimarySurfaceHistory = false;
        _hasDirectRestirHistory = false;
        _hasIndirectRestirHistory = false;
        _lastDirectReservoirOutputIdx = 0;
        _lastIndirectReservoirOutputIdx = 0;
        ClearAccumulationRenderTargets();
    }

    private void ResetAccumulationOnly(string reason = "accumulation_reset")
    {
        _restirDiagnostics?.ScheduleResetCapture(reason, sampleCount);
        sampleCount = 0;
        frameId = 0;
        _hasDirectRestirHistory = false;
        _hasIndirectRestirHistory = false;
        _lastDirectReservoirOutputIdx = 0;
        _lastIndirectReservoirOutputIdx = 0;
        ClearAccumulationRenderTargets();
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
