using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Text;
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
    private int kernelGenerateInitial;
    private int kernelTemporalResampling;
    private int kernelShadeDISamples;
    private int kernelGenerateGISecondarySurfaces;
    private int kernelShadeGISecondarySurfaces;
    private int kernelTemporalGIResampling;
    private int kernelSpatialGIResampling;
    private int kernelShadeGISamples;

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
    [StructLayout(LayoutKind.Sequential)]
    private struct LightDataPackedCPU
    {
        public Vector3 position; public float range;
        public Vector3 color; public float intensity;
        public Vector3 direction; public float sourceRadius;
        public float power; public float cdf;
        public uint lightType; public uint originalIndex;
    }
    [StructLayout(LayoutKind.Sequential)]
    private struct DirectLightReservoirDataCPU
    {
        public Vector3 origin; public float maxDist;
        public Vector3 direction; public float targetLum;
        public Vector3 contribution; public float weightSum;
        public Vector3 surfaceNormal; public float proposalPdf;
        public uint lightType;
        public uint lightIndex;
        public uint sampleCount;
        public float selectedWeight;
    }
    [StructLayout(LayoutKind.Sequential)]
    private struct IndirectReservoirDataCPU
    {
        public Vector3 secondaryPosition; public float proposalPdf;
        public Vector3 secondaryNormal; public float targetLum;
        public Vector3 radiance; public float weightSum;
        public Vector3 contribution; public float selectedWeight;
        public Vector3 primaryNormal; public float sampleCount;
    }
    [StructLayout(LayoutKind.Sequential)]
    private struct SecondarySurfaceDataCPU
    {
        public Vector3 position; public float proposalPdf;
        public Vector3 normal; public float primaryDistance;
        public Vector3 throughput; public float flags;
        public Vector3 albedo; public float roughness;
        public Vector3 emissionRadiance; public float metallic;
        public float alpha;
        public float ior;
        public float mode;
        public float reserved;
    }
    [StructLayout(LayoutKind.Sequential)]
    private struct HitDataCPU
    {
        public Vector3 position; public float distance;
        public Vector3 normal; public float mode;
        public Vector3 albedo; public float emissionIntensity;
        public Vector3 emission; public float roughness;
        public float metallic; public float alpha;
        public float ior;
    }
    [StructLayout(LayoutKind.Sequential)]
    private struct PathContributionCPU
    {
        public Vector3 L; public float padding0;
        public Vector3 throughput; public float padding1;
    }
    private readonly HitDataCPU[] _spatialNeighborHitsReadback = new HitDataCPU[9];
    private readonly IndirectReservoirDataCPU[] _spatialNeighborReservoirsReadback = new IndirectReservoirDataCPU[9];
    private readonly Vector4[] _restirGISpatialDebugReadback = new Vector4[RestirDebugDataCount];
    private readonly PathContributionCPU[] _globalColorProbeReadback = new PathContributionCPU[1];
    private readonly DirectLightReservoirDataCPU[] _directLightProbeReadback = new DirectLightReservoirDataCPU[1];
    private int _previousTargetFrameRate = -1;
    private int _previousVSyncCount = -1;
    private int _previousRenderFrameInterval = -1;
    private bool _hasCapturedFrameRateState = false;
    private readonly IndirectReservoirDataCPU[] _initialGIProbeReadback = new IndirectReservoirDataCPU[1];
    private readonly IndirectReservoirDataCPU[] _activeGIProbeReadback = new IndirectReservoirDataCPU[1];
    private readonly SecondarySurfaceDataCPU[] _secondarySurfaceProbeReadback = new SecondarySurfaceDataCPU[1];
    private readonly SecondarySurfaceDataCPU[] _secondarySurfaceProbeSentinelWrite = new SecondarySurfaceDataCPU[1];
    private readonly HitDataCPU[] _giPrimaryHitReadback = new HitDataCPU[1];
    private readonly StringBuilder _giDiagnosticBuilder = new StringBuilder(512);
    private bool _warnedAboutReSTIRGIWithoutDI = false;
    private string _currentDiagnosticOutputDir;
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
        kernelGenerateInitial = tracingShader.FindKernel("kernel_generate_initial");
        kernelTemporalResampling = tracingShader.FindKernel("kernel_temporal_resampling");
        kernelShadeDISamples = tracingShader.FindKernel("kernel_shade_di_samples");
        kernelGenerateGISecondarySurfaces = tracingShader.FindKernel("kernel_generate_gi_secondary_surfaces");
        kernelShadeGISecondarySurfaces = tracingShader.FindKernel("kernel_shade_gi_secondary_surfaces");
        kernelTemporalGIResampling = tracingShader.FindKernel("kernel_temporal_gi_resampling");
        kernelSpatialGIResampling = tracingShader.FindKernel("kernel_spatial_gi_resampling");
        kernelShadeGISamples = tracingShader.FindKernel("kernel_shade_gi_samples");

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
        cmdBuffer = new CommandBuffer();
        CacheRuntimeSettings();
        _lastLightStateHash = LightManager.Instance.ComputeLightStateHash();
        for (int i = 0; i < bounceNames.Length; i++)
        {
            int bounce = i / 3;
            int phase = i % 3;
            bounceNames[i] = "PT_B" + bounce + "_" + (phase == 0 ? "Trace" : phase == 1 ? "Shade" : "Shadow");
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
            ResetSampleCount();
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
            ResetSampleCount();
        }
        else if (cameraMoved)
        {
            ResetAccumulationOnly();
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

            if (UseReSTIRDI)
                DispatchReSTIRDI(pixelCount);

            if (UseReSTIRGI)
            {
                DispatchReSTIRGI(pixelCount);
                WriteReSTIRGIProbeIfNeeded(pixelCount);
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

    private void DispatchReSTIRDI(int pixelCount)
    {
        if (!UseReSTIRDI) return;

        int initialIdx = (_lastDirectReservoirOutputIdx + 1) % 3;
        int temporalIdx = (_lastDirectReservoirOutputIdx + 2) % 3;
        int prevIdx = _lastDirectReservoirOutputIdx;

        int lightCount = 1 + LightManager.Instance.GetPointLightsCount();
        // Clamp to buffer capacity
        int maxLightSlots = _lightDataPacked != null ? _lightDataPacked.count : 0;
        lightCount = Mathf.Min(lightCount, maxLightSlots);

        // --- 1. Prepare lights ---
        BindRestirCommonParams(kernelPrepareLights);
        BindSceneBuffersToKernel(kernelPrepareLights);
        tracingShader.SetBuffer(kernelPrepareLights, "_LightDataPacked", _lightDataPacked);
        tracingShader.SetInt("_LightDataPackedCount", lightCount);
        tracingShader.Dispatch(kernelPrepareLights, (lightCount + 63) / 64, 1, 1);

        // --- 1b. Build CDF on CPU ---
        if (lightCount > 0 && _lightDataPacked != null)
        {
            var lightData = new LightDataPackedCPU[lightCount];
            _lightDataPacked.GetData(lightData, 0, 0, lightCount);
            float runningCdf = 0f;
            for (int i = 0; i < lightData.Length; i++)
            {
                runningCdf += lightData[i].power;
                lightData[i].cdf = runningCdf;
            }
            _lightDataPacked.SetData(lightData, 0, 0, lightCount);
        }

        // --- 2. Generate initial reservoirs ---
        BindRestirCommonParams(kernelGenerateInitial);
        BindSceneBuffersToKernel(kernelGenerateInitial);
        tracingShader.SetBuffer(kernelGenerateInitial, "DirectLightReservoirs", _directLightReservoirs);
        tracingShader.SetBuffer(kernelGenerateInitial, "_RestirGbuffer", _globalHits);
        tracingShader.SetBuffer(kernelGenerateInitial, "_RestirLightData", _lightDataPacked);
        tracingShader.SetInt("_RestirLightCount", lightCount);
        tracingShader.SetInt("_RestirInitialReservoirOffset", initialIdx * pixelCount);
        tracingShader.SetInt("_RestirCandidateCount", DirectLightRISCandidateCount);
        tracingShader.Dispatch(kernelGenerateInitial, (pixelCount + 63) / 64, 1, 1);

        // --- 3. Temporal resampling ---
        bool useTemporal = _hasDirectRestirHistory;
        if (useTemporal)
        {
            BindRestirCommonParams(kernelTemporalResampling);
            BindSceneBuffersToKernel(kernelTemporalResampling);
            tracingShader.SetBuffer(kernelTemporalResampling, "DirectLightReservoirs", _directLightReservoirs);
            tracingShader.SetInt("_RestirInitialReservoirOffset", initialIdx * pixelCount);
            tracingShader.SetInt("_RestirTemporalReservoirOffset", temporalIdx * pixelCount);
            tracingShader.SetInt("_RestirPrevReservoirOffset", prevIdx * pixelCount);
            tracingShader.SetBuffer(kernelTemporalResampling, "_RestirGbuffer", _globalHits);
            tracingShader.SetBuffer(kernelTemporalResampling, "_RestirGbufferPrevious", _primarySurfaceHistoryPrev);
            tracingShader.Dispatch(kernelTemporalResampling, (pixelCount + 63) / 64, 1, 1);
        }

        int shadingReservoirIdx = useTemporal ? temporalIdx : initialIdx;

        // --- 4. Shade DI samples ---
        BindRestirCommonParams(kernelShadeDISamples);
        BindSceneBuffersToKernel(kernelShadeDISamples);
        tracingShader.SetBuffer(kernelShadeDISamples, "DirectLightReservoirs", _directLightReservoirs);
        tracingShader.SetBuffer(kernelShadeDISamples, "GlobalColors", _globalColors);
        tracingShader.SetInt("_RestirShadingReservoirOffset", shadingReservoirIdx * pixelCount);
        int diProbePixelIndex = GetCenterProbePixelIndex();
        PathContributionCPU globalColorBeforeDI = default;
        DirectLightReservoirDataCPU selectedDIReservoir = default;
        bool capturedDirectLightBeforeShading = false;
        bool captureDirectLightProbe = UseReSTIRGI && WriteReSTIRGIDiagnostics && ReSTIRGIDiagnosticFrameInterval > 0 && (sampleCount % ReSTIRGIDiagnosticFrameInterval) == 0;
        if (captureDirectLightProbe && diProbePixelIndex >= 0)
        {
            try
            {
                int directReservoirIndex = shadingReservoirIdx * pixelCount + diProbePixelIndex;
                _directLightReservoirs.GetData(_directLightProbeReadback, 0, directReservoirIndex, 1);
                _globalColors.GetData(_globalColorProbeReadback, 0, diProbePixelIndex, 1);
                selectedDIReservoir = _directLightProbeReadback[0];
                globalColorBeforeDI = _globalColorProbeReadback[0];
                capturedDirectLightBeforeShading = true;
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[ReSTIR DI] Diagnostic pre-readback failed: {ex.Message}");
            }
        }
        tracingShader.Dispatch(kernelShadeDISamples, (pixelCount + 63) / 64, 1, 1);

        if (captureDirectLightProbe && diProbePixelIndex >= 0 && capturedDirectLightBeforeShading)
        {
            try
            {
                _globalColors.GetData(_globalColorProbeReadback, 0, diProbePixelIndex, 1);
                PathContributionCPU globalColorAfterDI = _globalColorProbeReadback[0];
                Vector3 directLightDelta = globalColorAfterDI.L - globalColorBeforeDI.L;
                Debug.Log(
                    $"[ReSTIR DI] Shading probe sampleCount={sampleCount} pixel=({diProbePixelIndex % _currentRenderWidth},{diProbePixelIndex / _currentRenderWidth}) " +
                    $"reservoirValid={IsDirectReservoirNumericallyValid(selectedDIReservoir)} targetLum={selectedDIReservoir.targetLum} " +
                    $"selectedWeight={selectedDIReservoir.selectedWeight} globalLightDelta=({directLightDelta.x},{directLightDelta.y},{directLightDelta.z})");
                AppendReSTIRDISummaryJson(
                    diProbePixelIndex % _currentRenderWidth,
                    diProbePixelIndex / _currentRenderWidth,
                    selectedDIReservoir,
                    globalColorBeforeDI,
                    globalColorAfterDI);
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[ReSTIR DI] Diagnostic readback failed: {ex.Message}");
            }
        }

        // Advance rotation
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

        int stage1ProbeX = -1;
        int stage1ProbeY = -1;
        int stage1ProbePixelIndex = -1;
        Vector2Int[] diagnosticProbePixels = null;
        bool captureStage1Probe = WriteReSTIRGIDiagnostics && ReSTIRGIDiagnosticFrameInterval > 0 && (sampleCount % ReSTIRGIDiagnosticFrameInterval) == 0;
        if (captureStage1Probe)
        {
            stage1ProbeX = Mathf.Clamp(_currentRenderWidth / 2, 0, Mathf.Max(0, _currentRenderWidth - 1));
            stage1ProbeY = Mathf.Clamp(_currentRenderHeight / 2, 0, Mathf.Max(0, _currentRenderHeight - 1));
            stage1ProbePixelIndex = stage1ProbeY * _currentRenderWidth + stage1ProbeX;
            diagnosticProbePixels = new[]
            {
                new Vector2Int(stage1ProbeX, stage1ProbeY),
                new Vector2Int(Mathf.Clamp(stage1ProbeX - _currentRenderWidth / 8, 0, Mathf.Max(0, _currentRenderWidth - 1)), stage1ProbeY),
                new Vector2Int(Mathf.Clamp(stage1ProbeX + _currentRenderWidth / 8, 0, Mathf.Max(0, _currentRenderWidth - 1)), stage1ProbeY),
                new Vector2Int(stage1ProbeX, Mathf.Clamp(stage1ProbeY - _currentRenderHeight / 8, 0, Mathf.Max(0, _currentRenderHeight - 1))),
                new Vector2Int(stage1ProbeX, Mathf.Clamp(stage1ProbeY + _currentRenderHeight / 8, 0, Mathf.Max(0, _currentRenderHeight - 1)))
            };
            _secondarySurfaceProbeSentinelWrite[0] = new SecondarySurfaceDataCPU
            {
                position = new Vector3(42f, 43f, 44f),
                proposalPdf = 45f,
                normal = new Vector3(46f, 47f, 48f),
                primaryDistance = 49f,
                throughput = new Vector3(50f, 51f, 52f),
                flags = 53f,
                albedo = new Vector3(54f, 55f, 56f),
                roughness = 57f,
                emissionRadiance = new Vector3(58f, 59f, 60f),
                metallic = 61f,
                alpha = 62f,
                ior = 63f,
                mode = 64f,
                reserved = 65f
            };
            _secondarySurfaces.SetData(_secondarySurfaceProbeSentinelWrite, 0, stage1ProbePixelIndex, 1);
        }
        tracingShader.Dispatch(kernelGenerateGISecondarySurfaces, (pixelCount + 63) / 64, 1, 1);

        if (captureStage1Probe)
        {
            try
            {
                _secondarySurfaces.GetData(_secondarySurfaceProbeReadback, 0, stage1ProbePixelIndex, 1);
                _globalHits.GetData(_giPrimaryHitReadback, 0, stage1ProbePixelIndex, 1);
                SecondarySurfaceDataCPU probe = _secondarySurfaceProbeReadback[0];
                HitDataCPU primary = _giPrimaryHitReadback[0];
                Debug.Log($"[ReSTIR GI] Stage1 secondary probe sampleCount={sampleCount} pixel=({stage1ProbeX},{stage1ProbeY}) flags={probe.flags} primaryDistance={probe.primaryDistance} proposalPdf={probe.proposalPdf} throughput=({probe.throughput.x},{probe.throughput.y},{probe.throughput.z}) primaryHitDistance={primary.distance}");
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[ReSTIR GI] Stage1 secondary probe readback failed: {ex.Message}");
            }
        }

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
        tracingShader.SetInt("_RestirDebugPixelIndex", stage1ProbePixelIndex >= 0 ? stage1ProbePixelIndex : 0);
        tracingShader.Dispatch(kernelShadeGISecondarySurfaces, (pixelCount + 63) / 64, 1, 1);

        if (captureStage1Probe)
        {
            try
            {
                int stage2ReservoirIndex = initialIdx * pixelCount + stage1ProbePixelIndex;
                _indirectReservoirs.GetData(_initialGIProbeReadback, 0, stage2ReservoirIndex, 1);
                _restirDebugData.GetData(_restirGISpatialDebugReadback, 0, 0, 3);
                IndirectReservoirDataCPU stage2Probe = _initialGIProbeReadback[0];
                Vector4 stage2Debug0 = _restirGISpatialDebugReadback[0];
                Vector4 stage2Debug1 = _restirGISpatialDebugReadback[1];
                Vector4 stage2Debug2 = _restirGISpatialDebugReadback[2];
                Debug.Log(
                    $"[ReSTIR GI] Initial reservoir probe sampleCount={sampleCount} pixel=({stage1ProbeX},{stage1ProbeY}) " +
                    $"proposalPdf={stage2Probe.proposalPdf} targetLum={stage2Probe.targetLum} weightSum={stage2Probe.weightSum} " +
                    $"selectedWeight={stage2Probe.selectedWeight} sampleCountM={stage2Probe.sampleCount} " +
                    $"radiance=({stage2Probe.radiance.x},{stage2Probe.radiance.y},{stage2Probe.radiance.z}) " +
                    $"contribution=({stage2Probe.contribution.x},{stage2Probe.contribution.y},{stage2Probe.contribution.z})");
                Debug.Log(
                    $"[ReSTIR GI] Initial shading debug sampleCount={sampleCount} pixel=({stage1ProbeX},{stage1ProbeY}) " +
                    $"code={stage2Debug0.x} data0=({stage2Debug0.y},{stage2Debug0.z},{stage2Debug0.w}) " +
                    $"data1=({stage2Debug1.x},{stage2Debug1.y},{stage2Debug1.z},{stage2Debug1.w}) " +
                    $"data2=({stage2Debug2.x},{stage2Debug2.y},{stage2Debug2.z},{stage2Debug2.w})");
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[ReSTIR GI] Initial reservoir probe readback failed: {ex.Message}");
            }
        }

        bool useTemporal = _hasIndirectRestirHistory;
        int temporalDebugProbeId = 0;
        int temporalDebugProbeX = stage1ProbeX;
        int temporalDebugProbeY = stage1ProbeY;
        int temporalDebugProbePixelIndex = stage1ProbePixelIndex;
        if (captureStage1Probe && diagnosticProbePixels != null)
        {
            if (!TrySelectActiveGIProbePixel(diagnosticProbePixels, pixelCount, initialIdx, out temporalDebugProbeId, out temporalDebugProbeX, out temporalDebugProbeY, out temporalDebugProbePixelIndex))
            {
                temporalDebugProbeId = 0;
                temporalDebugProbeX = stage1ProbeX;
                temporalDebugProbeY = stage1ProbeY;
                temporalDebugProbePixelIndex = stage1ProbePixelIndex;
            }
        }
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
            tracingShader.SetInt("_RestirDebugPixelIndex", temporalDebugProbePixelIndex >= 0 ? temporalDebugProbePixelIndex : 0);
            tracingShader.Dispatch(kernelTemporalGIResampling, (pixelCount + 63) / 64, 1, 1);

        }

        int shadingReservoirIdx = useTemporal ? temporalIdx : initialIdx;
        int selectedProbeId = 0;
        int selectedProbeX = stage1ProbeX;
        int selectedProbeY = stage1ProbeY;
        int selectedProbePixelIndex = stage1ProbePixelIndex;
        if (captureStage1Probe && diagnosticProbePixels != null)
        {
            if (!TrySelectActiveGIProbePixel(diagnosticProbePixels, pixelCount, shadingReservoirIdx, out selectedProbeId, out selectedProbeX, out selectedProbeY, out selectedProbePixelIndex))
            {
                selectedProbeId = 0;
                selectedProbeX = stage1ProbeX;
                selectedProbeY = stage1ProbeY;
                selectedProbePixelIndex = stage1ProbePixelIndex;
            }
        }

        if (captureStage1Probe && useTemporal)
        {
            try
            {
                int temporalReservoirIndex = temporalIdx * pixelCount + temporalDebugProbePixelIndex;
                _indirectReservoirs.GetData(_activeGIProbeReadback, 0, temporalReservoirIndex, 1);
                _restirDebugData.GetData(_restirGISpatialDebugReadback, 0, 0, RestirDebugDataCount);
                IndirectReservoirDataCPU temporalProbe = _activeGIProbeReadback[0];
                Vector4 temporalDebug0 = _restirGISpatialDebugReadback[0];
                Vector4 temporalDebug1 = _restirGISpatialDebugReadback[1];
                Vector4 temporalDebug2 = _restirGISpatialDebugReadback[2];
                Debug.Log(
                    $"[ReSTIR GI] Temporal reservoir probe sampleCount={sampleCount} probeId={temporalDebugProbeId} pixel=({temporalDebugProbeX},{temporalDebugProbeY}) " +
                    $"proposalPdf={temporalProbe.proposalPdf} targetLum={temporalProbe.targetLum} weightSum={temporalProbe.weightSum} " +
                    $"selectedWeight={temporalProbe.selectedWeight} sampleCountM={temporalProbe.sampleCount} " +
                    $"selectedPrevious={(temporalDebug0.x > 0.5f)} temporalOffset={temporalDebug0.y} fallback={(temporalDebug0.z > 0.5f)} " +
                    $"selectedPrevOriginalProposalPdf={temporalDebug1.z} selectedPrevReuseProposalPdf={temporalDebug1.w} jacobian={temporalDebug2.x} " +
                    $"pi={temporalDebug2.y} piSum={temporalDebug2.z} normDen={temporalDebug2.w} " +
                    $"radiance=({temporalProbe.radiance.x},{temporalProbe.radiance.y},{temporalProbe.radiance.z}) " +
                    $"contribution=({temporalProbe.contribution.x},{temporalProbe.contribution.y},{temporalProbe.contribution.z})");
                AppendReSTIRGITemporalSummaryJson(
                    temporalDebugProbeId,
                    temporalDebugProbeX,
                    temporalDebugProbeY,
                    temporalProbe,
                    temporalDebug0,
                    temporalDebug1,
                    temporalDebug2);
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[ReSTIR GI] Temporal reservoir probe readback failed: {ex.Message}");
            }
        }

        BindRestirCommonParams(kernelSpatialGIResampling);
        BindSceneBuffersToKernel(kernelSpatialGIResampling);
        tracingShader.SetBuffer(kernelSpatialGIResampling, "IndirectReservoirs", _indirectReservoirs);
        tracingShader.SetBuffer(kernelSpatialGIResampling, "ReSTIRDebugData", _restirDebugData);
        tracingShader.SetBuffer(kernelSpatialGIResampling, "_RestirGbuffer", _globalHits);
        tracingShader.SetInt("_RestirShadingReservoirOffset", shadingReservoirIdx * pixelCount);
        tracingShader.SetInt("_RestirSpatialReservoirOffset", spatialIdx * pixelCount);
        tracingShader.SetInt("_RestirDebugPixelIndex", selectedProbePixelIndex >= 0 ? selectedProbePixelIndex : 0);
        tracingShader.Dispatch(kernelSpatialGIResampling, (pixelCount + 63) / 64, 1, 1);

        if (captureStage1Probe)
        {
            try
            {
                int spatialReservoirIndex = spatialIdx * pixelCount + selectedProbePixelIndex;
                _indirectReservoirs.GetData(_activeGIProbeReadback, 0, spatialReservoirIndex, 1);
                IndirectReservoirDataCPU spatialProbe = _activeGIProbeReadback[0];
                Debug.Log(
                    $"[ReSTIR GI] Spatial reservoir probe sampleCount={sampleCount} probeId={selectedProbeId} pixel=({selectedProbeX},{selectedProbeY}) " +
                    $"proposalPdf={spatialProbe.proposalPdf} targetLum={spatialProbe.targetLum} weightSum={spatialProbe.weightSum} " +
                    $"selectedWeight={spatialProbe.selectedWeight} sampleCountM={spatialProbe.sampleCount} " +
                    $"radiance=({spatialProbe.radiance.x},{spatialProbe.radiance.y},{spatialProbe.radiance.z}) " +
                    $"contribution=({spatialProbe.contribution.x},{spatialProbe.contribution.y},{spatialProbe.contribution.z})");

                if (selectedProbePixelIndex >= 0)
                {
                    (int validSurfaceNeighbors, int compatibleNeighbors, int activeReservoirNeighbors, CenterSpatialNeighborDebugInfo[] details) =
                        ComputeCenterSpatialNeighborStats(selectedProbeX, selectedProbeY, shadingReservoirIdx, pixelCount);
                    _restirDebugData.GetData(_restirGISpatialDebugReadback, 0, 0, RestirDebugDataCount);
                    Vector4 spatialShaderStats = _restirGISpatialDebugReadback[0];
                    Vector4 spatialShaderFailStats0 = _restirGISpatialDebugReadback[1];
                    Vector4 spatialShaderFailStats1 = _restirGISpatialDebugReadback[2];
                    Vector4 spatialShaderSelectionStats = _restirGISpatialDebugReadback[3];
                    Vector4 spatialShaderNormalizationStats = _restirGISpatialDebugReadback[4];
                    string spatialSummary =
                        $"[ReSTIR GI] Spatial selected-probe stats sampleCount={sampleCount} selectedProbeId={selectedProbeId} pixel=({selectedProbeX},{selectedProbeY}) " +
                        $"validSurfaceNeighbors={validSurfaceNeighbors} compatibleNeighbors={compatibleNeighbors} activeReservoirNeighbors={activeReservoirNeighbors} " +
                        $"spatialShaderCompatibleNeighbors={spatialShaderStats.x} spatialShaderReevaluateNeighbors={spatialShaderStats.y} " +
                        $"spatialShaderJacobianNeighbors={spatialShaderStats.z} spatialShaderCombinedNeighbors={spatialShaderStats.w} " +
                        $"spatialShaderReevaluateFailInvalidSample={spatialShaderFailStats0.x} " +
                        $"spatialShaderReevaluateFailInvalidSurface={spatialShaderFailStats0.y} " +
                        $"spatialShaderReevaluateFailDistance={spatialShaderFailStats0.z} " +
                        $"spatialShaderReevaluateFailBrdf={spatialShaderFailStats0.w} " +
                        $"spatialShaderReevaluateFailBackfacing={spatialShaderFailStats1.x} " +
                        $"spatialShaderReevaluateFailZeroTarget={spatialShaderFailStats1.y} " +
                        $"selectedNeighborIndex={spatialShaderFailStats1.z} selectedTargetPdf={spatialShaderFailStats1.w} " +
                        $"selectedNeighborOriginalProposalPdf={spatialShaderSelectionStats.x} selectedNeighborReuseProposalPdf={spatialShaderSelectionStats.y} " +
                        $"selectedNeighborJacobian={spatialShaderSelectionStats.z} selectedNeighborTargetPdf={spatialShaderSelectionStats.w} " +
                        $"pi={spatialShaderNormalizationStats.x} piSum={spatialShaderNormalizationStats.y} normDen={spatialShaderNormalizationStats.z} currentTargetPdf={spatialShaderNormalizationStats.w}";
                    Debug.Log(spatialSummary);
                    AppendReSTIRGISpatialSummaryJson(
                        selectedProbeId,
                        selectedProbeX,
                        selectedProbeY,
                        validSurfaceNeighbors,
                        compatibleNeighbors,
                        activeReservoirNeighbors,
                        spatialShaderStats,
                        spatialShaderFailStats0,
                        spatialShaderFailStats1,
                        spatialShaderSelectionStats,
                        spatialShaderNormalizationStats);

                    if (WriteReSTIRGIDiagnosticDetails)
                    {
                        for (int detailIdx = 0; detailIdx < details.Length; detailIdx++)
                        {
                            CenterSpatialNeighborDebugInfo detail = details[detailIdx];
                            Debug.Log(
                                $"[ReSTIR GI] Spatial selected-probe detail sampleCount={sampleCount} selectedProbeId={selectedProbeId} idx={detail.neighborIndex} pixel=({detail.pixelX},{detail.pixelY}) " +
                                $"validSurface={detail.validSurface} compatible={detail.compatible} activeReservoir={detail.activeReservoir} " +
                                $"targetLum={detail.targetLum} sampleCountM={detail.sampleCount} dotCurrentNormalToSecondary={detail.dotCurrentNormalToSecondary} " +
                                $"neighborPos=({detail.neighborPosition.x},{detail.neighborPosition.y},{detail.neighborPosition.z}) " +
                                $"neighborNormal=({detail.neighborNormal.x},{detail.neighborNormal.y},{detail.neighborNormal.z}) " +
                                $"secondaryPos=({detail.secondaryPosition.x},{detail.secondaryPosition.y},{detail.secondaryPosition.z}) " +
                                $"radiance=({detail.radiance.x},{detail.radiance.y},{detail.radiance.z})");
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[ReSTIR GI] Spatial reservoir probe readback failed: {ex.Message}");
            }
        }

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
        tracingShader.SetInt("_RestirDebugPixelIndex", selectedProbePixelIndex >= 0 ? selectedProbePixelIndex : 0);
        PathContributionCPU globalColorBeforeFinal = default;
        bool capturedGlobalColorBeforeFinal = false;
        IndirectReservoirDataCPU finalShadeProbe = default;
        IndirectReservoirDataCPU initialShadeProbe = default;
        if (captureStage1Probe && selectedProbePixelIndex >= 0)
        {
            try
            {
                int finalReservoirIndex = shadingReservoirIdx * pixelCount + selectedProbePixelIndex;
                int initialReservoirSlotForFinalReadback = (shadingReservoirIdx + 2) % 3;
                int initialReservoirIndex = initialReservoirSlotForFinalReadback * pixelCount + selectedProbePixelIndex;
                _indirectReservoirs.GetData(_activeGIProbeReadback, 0, finalReservoirIndex, 1);
                _indirectReservoirs.GetData(_initialGIProbeReadback, 0, initialReservoirIndex, 1);
                _globalColors.GetData(_globalColorProbeReadback, 0, selectedProbePixelIndex, 1);
                finalShadeProbe = _activeGIProbeReadback[0];
                initialShadeProbe = _initialGIProbeReadback[0];
                globalColorBeforeFinal = _globalColorProbeReadback[0];
                capturedGlobalColorBeforeFinal = true;
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[ReSTIR GI] Final shading pre-readback failed: {ex.Message}");
            }
        }
        tracingShader.Dispatch(kernelShadeGISamples, (pixelCount + 63) / 64, 1, 1);

        if (captureStage1Probe && selectedProbePixelIndex >= 0 && capturedGlobalColorBeforeFinal)
        {
            try
            {
                _globalColors.GetData(_globalColorProbeReadback, 0, selectedProbePixelIndex, 1);
                _restirDebugData.GetData(_restirGISpatialDebugReadback, 0, 0, RestirDebugDataCount);
                PathContributionCPU globalColorAfterFinal = _globalColorProbeReadback[0];
                Vector4 finalDebug0 = _restirGISpatialDebugReadback[0];
                Vector4 finalDebug1 = _restirGISpatialDebugReadback[1];
                Vector4 finalDebug2 = _restirGISpatialDebugReadback[2];
                Vector3 globalLightDelta = globalColorAfterFinal.L - globalColorBeforeFinal.L;
                Debug.Log(
                    $"[ReSTIR GI] Final shading probe sampleCount={sampleCount} probeId={selectedProbeId} pixel=({selectedProbeX},{selectedProbeY}) " +
                    $"misFinalWeight={finalDebug0.z} misInitialWeight={finalDebug0.w} " +
                    $"finalContribution=({finalDebug1.x},{finalDebug1.y},{finalDebug1.z}) " +
                    $"globalLightDelta=({globalLightDelta.x},{globalLightDelta.y},{globalLightDelta.z})");
                AppendReSTIRGIFinalSummaryJson(
                    selectedProbeId,
                    selectedProbeX,
                    selectedProbeY,
                    initialShadeProbe,
                    finalShadeProbe,
                    globalColorBeforeFinal,
                    globalColorAfterFinal,
                    finalDebug0,
                    finalDebug1,
                    finalDebug2);
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[ReSTIR GI] Final shading probe readback failed: {ex.Message}");
            }
        }

        _lastIndirectReservoirOutputIdx = shadingReservoirIdx;
        _hasIndirectRestirHistory = true;
    }

    private struct CenterSpatialNeighborDebugInfo
    {
        public int neighborIndex;
        public int pixelX;
        public int pixelY;
        public bool validSurface;
        public bool compatible;
        public bool activeReservoir;
        public Vector3 neighborPosition;
        public Vector3 neighborNormal;
        public Vector3 secondaryPosition;
        public Vector3 radiance;
        public float targetLum;
        public float sampleCount;
        public float dotCurrentNormalToSecondary;
    }

    private bool TrySelectActiveGIProbePixel(
        Vector2Int[] probePixels,
        int pixelCount,
        int reservoirSlotIdx,
        out int selectedProbeId,
        out int selectedProbeX,
        out int selectedProbeY,
        out int selectedProbePixelIndex)
    {
        selectedProbeId = -1;
        selectedProbeX = 0;
        selectedProbeY = 0;
        selectedProbePixelIndex = 0;

        if (_indirectReservoirs == null || probePixels == null || probePixels.Length == 0)
            return false;

        for (int probeIdx = 0; probeIdx < probePixels.Length; probeIdx++)
        {
            int probeX = probePixels[probeIdx].x;
            int probeY = probePixels[probeIdx].y;
            int probePixelIndex = probeY * _currentRenderWidth + probeX;
            int reservoirIndex = reservoirSlotIdx * pixelCount + probePixelIndex;

            _indirectReservoirs.GetData(_activeGIProbeReadback, 0, reservoirIndex, 1);
            if (!IsIndirectReservoirValid(_activeGIProbeReadback[0]))
                continue;

            selectedProbeId = probeIdx;
            selectedProbeX = probeX;
            selectedProbeY = probeY;
            selectedProbePixelIndex = probePixelIndex;
            return true;
        }

        return false;
    }

    private (int validSurfaceNeighbors, int compatibleNeighbors, int activeReservoirNeighbors, CenterSpatialNeighborDebugInfo[] details) ComputeCenterSpatialNeighborStats(
        int centerX,
        int centerY,
        int shadingReservoirIdx,
        int pixelCount)
    {
        int width = _currentRenderWidth;
        int height = _currentRenderHeight;
        int[] neighborOffsetsX = { -1, 1, 0, 0, -1, 1, -1, 1 };
        int[] neighborOffsetsY = { 0, 0, -1, 1, -1, -1, 1, 1 };
        int centerLinear = centerY * width + centerX;

        _globalHits.GetData(_giPrimaryHitReadback, 0, centerLinear, 1);
        HitDataCPU centerHit = _giPrimaryHitReadback[0];
        if (!IsPrimaryHitValid(centerHit))
            return (0, 0, 0, Array.Empty<CenterSpatialNeighborDebugInfo>());

        int validSurfaceNeighbors = 0;
        int compatibleNeighbors = 0;
        int activeReservoirNeighbors = 0;
        List<CenterSpatialNeighborDebugInfo> details = new List<CenterSpatialNeighborDebugInfo>(8);
        Vector3 currentNormal = centerHit.normal.normalized;
        for (int i = 0; i < 8; i++)
        {
            int nx = centerX + neighborOffsetsX[i];
            int ny = centerY + neighborOffsetsY[i];
            if (nx < 0 || nx >= width || ny < 0 || ny >= height)
                continue;

            int neighborLinear = ny * width + nx;
            _globalHits.GetData(_spatialNeighborHitsReadback, 0, neighborLinear, 1);
            HitDataCPU neighborHit = _spatialNeighborHitsReadback[0];
            bool validSurface = IsPrimaryHitValid(neighborHit);
            bool compatible = false;
            bool activeReservoir = false;
            IndirectReservoirDataCPU reservoir = default;

            if (!validSurface)
            {
                details.Add(new CenterSpatialNeighborDebugInfo
                {
                    neighborIndex = i,
                    pixelX = nx,
                    pixelY = ny,
                    validSurface = false
                });
                continue;
            }

            validSurfaceNeighbors++;
            compatible = IsSpatialProbeCompatible(centerHit, neighborHit);
            if (compatible)
            {
                compatibleNeighbors++;
                _indirectReservoirs.GetData(_spatialNeighborReservoirsReadback, 0, shadingReservoirIdx * pixelCount + neighborLinear, 1);
                reservoir = _spatialNeighborReservoirsReadback[0];
                activeReservoir = IsIndirectReservoirValid(reservoir);
                if (activeReservoir)
                    activeReservoirNeighbors++;
            }

            Vector3 toSecondary = reservoir.secondaryPosition - centerHit.position;
            Vector3 toSecondaryDir = toSecondary.sqrMagnitude > 1e-8f ? toSecondary.normalized : Vector3.zero;
            details.Add(new CenterSpatialNeighborDebugInfo
            {
                neighborIndex = i,
                pixelX = nx,
                pixelY = ny,
                validSurface = true,
                compatible = compatible,
                activeReservoir = activeReservoir,
                neighborPosition = neighborHit.position,
                neighborNormal = neighborHit.normal,
                secondaryPosition = reservoir.secondaryPosition,
                radiance = reservoir.radiance,
                targetLum = reservoir.targetLum,
                sampleCount = reservoir.sampleCount,
                dotCurrentNormalToSecondary = Vector3.Dot(currentNormal, toSecondaryDir)
            });
        }

        return (validSurfaceNeighbors, compatibleNeighbors, activeReservoirNeighbors, details.ToArray());
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

    private void WriteReSTIRGIProbeIfNeeded(int pixelCount)
    {
        if (!UseReSTIRGI || !WriteReSTIRGIDiagnostics || _indirectReservoirs == null || pixelCount <= 0)
            return;

        if (ReSTIRGIDiagnosticFrameInterval < 1)
            ReSTIRGIDiagnosticFrameInterval = 1;

        if ((sampleCount % ReSTIRGIDiagnosticFrameInterval) != 0)
            return;

        Vector2Int center = new Vector2Int(
            Mathf.Clamp(_currentRenderWidth / 2, 0, Mathf.Max(0, _currentRenderWidth - 1)),
            Mathf.Clamp(_currentRenderHeight / 2, 0, Mathf.Max(0, _currentRenderHeight - 1)));
        Vector2Int[] probePixels =
        {
            center,
            new Vector2Int(Mathf.Clamp(center.x - _currentRenderWidth / 8, 0, Mathf.Max(0, _currentRenderWidth - 1)), center.y),
            new Vector2Int(Mathf.Clamp(center.x + _currentRenderWidth / 8, 0, Mathf.Max(0, _currentRenderWidth - 1)), center.y),
            new Vector2Int(center.x, Mathf.Clamp(center.y - _currentRenderHeight / 8, 0, Mathf.Max(0, _currentRenderHeight - 1))),
            new Vector2Int(center.x, Mathf.Clamp(center.y + _currentRenderHeight / 8, 0, Mathf.Max(0, _currentRenderHeight - 1)))
        };

        for (int probeIdx = 0; probeIdx < probePixels.Length; probeIdx++)
        {
            int probeX = probePixels[probeIdx].x;
            int probeY = probePixels[probeIdx].y;
            int probePixelIndex = probeY * _currentRenderWidth + probeX;
            int activeReservoirIndex = _lastIndirectReservoirOutputIdx * pixelCount + probePixelIndex;
            int initialReservoirSlot = (_lastIndirectReservoirOutputIdx + 2) % 3;
            int initialReservoirIndex = initialReservoirSlot * pixelCount + probePixelIndex;

            try
            {
                _indirectReservoirs.GetData(_initialGIProbeReadback, 0, initialReservoirIndex, 1);
                _indirectReservoirs.GetData(_activeGIProbeReadback, 0, activeReservoirIndex, 1);
                _secondarySurfaces.GetData(_secondarySurfaceProbeReadback, 0, probePixelIndex, 1);
                _globalHits.GetData(_giPrimaryHitReadback, 0, probePixelIndex, 1);
                AppendReSTIRGIProbeJson(
                    probeIdx,
                    probeX,
                    probeY,
                    initialReservoirSlot,
                    _lastIndirectReservoirOutputIdx,
                    _secondarySurfaceProbeReadback[0],
                    _initialGIProbeReadback[0],
                    _activeGIProbeReadback[0],
                    _giPrimaryHitReadback[0]);
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[ReSTIR GI] Probe readback failed: {ex.Message}");
                break;
            }
        }
    }

    private int GetCenterProbePixelIndex()
    {
        if (_currentRenderWidth <= 0 || _currentRenderHeight <= 0)
            return -1;

        int probeX = Mathf.Clamp(_currentRenderWidth / 2, 0, Mathf.Max(0, _currentRenderWidth - 1));
        int probeY = Mathf.Clamp(_currentRenderHeight / 2, 0, Mathf.Max(0, _currentRenderHeight - 1));
        return probeY * _currentRenderWidth + probeX;
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

    private void AppendReSTIRGIProbeJson(
        int probeId,
        int pixelX,
        int pixelY,
        int initialReservoirIndex,
        int activeReservoirIndex,
        SecondarySurfaceDataCPU secondarySurface,
        IndirectReservoirDataCPU initialReservoir,
        IndirectReservoirDataCPU activeReservoir,
        HitDataCPU primaryHit)
    {
        string outputPath = GetDiagnosticOutputPath("restir_gi_probe.jsonl");
        CultureInfo culture = CultureInfo.InvariantCulture;
        bool initialValid = IsReservoirNumericallyValid(initialReservoir);
        bool activeValid = IsReservoirNumericallyValid(activeReservoir);
        string probeClass = ClassifyReSTIRGIProbe(secondarySurface, initialValid, activeValid);
        string stage1InvalidReason = DescribeReSTIRGIStage1InvalidReason(secondarySurface);
        Vector3 stage1DirectionToSecondary = Vector3.zero;
        float stage1DistanceToSecondary = 0.0f;
        float stage1PrimaryNormalDotSecondary = 0.0f;
        string emptyReservoirReason = DiagnoseReSTIRGIEmptyReservoirReason(
            primaryHit,
            secondarySurface,
            initialReservoir,
            out stage1DirectionToSecondary,
            out stage1DistanceToSecondary,
            out stage1PrimaryNormalDotSecondary);

        _giDiagnosticBuilder.Clear();
        _giDiagnosticBuilder.Append("{\"frameIndex\":").Append(frameId);
        _giDiagnosticBuilder.Append(",\"sampleCount\":").Append(sampleCount);
        _giDiagnosticBuilder.Append(",\"sceneName\":\"").Append(SceneManager.GetActiveScene().name).Append('"');
        _giDiagnosticBuilder.Append(",\"renderWidth\":").Append(_currentRenderWidth);
        _giDiagnosticBuilder.Append(",\"renderHeight\":").Append(_currentRenderHeight);
        _giDiagnosticBuilder.Append(",\"probeId\":").Append(probeId);
        _giDiagnosticBuilder.Append(",\"pixelX\":").Append(pixelX);
        _giDiagnosticBuilder.Append(",\"pixelY\":").Append(pixelY);
        _giDiagnosticBuilder.Append(",\"initialReservoirIndex\":").Append(initialReservoirIndex);
        _giDiagnosticBuilder.Append(",\"activeReservoirIndex\":").Append(activeReservoirIndex);
        _giDiagnosticBuilder.Append(",\"primaryHit\":").Append(IsPrimaryHitValid(primaryHit) ? "true" : "false");
        _giDiagnosticBuilder.Append(",\"primaryDistance\":").Append(primaryHit.distance.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"primaryMode\":").Append(primaryHit.mode.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"primaryPosition\":[").Append(FormatFloat3(primaryHit.position, culture)).Append(']');
        _giDiagnosticBuilder.Append(",\"primarySurfaceNormal\":[").Append(FormatFloat3(primaryHit.normal, culture)).Append(']');
        _giDiagnosticBuilder.Append(",\"primaryAlbedo\":[").Append(FormatFloat3(primaryHit.albedo, culture)).Append(']');
        _giDiagnosticBuilder.Append(",\"primaryEmission\":[").Append(FormatFloat3(primaryHit.emission, culture)).Append(']');
        _giDiagnosticBuilder.Append(",\"primaryEmissionIntensity\":").Append(primaryHit.emissionIntensity.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"primaryRoughness\":").Append(primaryHit.roughness.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"primaryMetallic\":").Append(primaryHit.metallic.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"primaryAlpha\":").Append(primaryHit.alpha.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"primaryIor\":").Append(primaryHit.ior.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"secondaryProposalPdf\":").Append(secondarySurface.proposalPdf.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"secondaryPrimaryDistance\":").Append(secondarySurface.primaryDistance.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"secondaryFlags\":").Append(secondarySurface.flags.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"secondaryFlagsHex\":\"0x").Append(Mathf.RoundToInt(secondarySurface.flags).ToString("X")).Append('"');
        _giDiagnosticBuilder.Append(",\"secondaryPosition\":[").Append(FormatFloat3(secondarySurface.position, culture)).Append(']');
        _giDiagnosticBuilder.Append(",\"secondaryNormal\":[").Append(FormatFloat3(secondarySurface.normal, culture)).Append(']');
        _giDiagnosticBuilder.Append(",\"secondaryThroughput\":[").Append(FormatFloat3(secondarySurface.throughput, culture)).Append(']');
        _giDiagnosticBuilder.Append(",\"secondaryAlbedo\":[").Append(FormatFloat3(secondarySurface.albedo, culture)).Append(']');
        _giDiagnosticBuilder.Append(",\"secondaryRoughness\":").Append(secondarySurface.roughness.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"secondaryEmissionRadiance\":[").Append(FormatFloat3(secondarySurface.emissionRadiance, culture)).Append(']');
        _giDiagnosticBuilder.Append(",\"secondaryMetallic\":").Append(secondarySurface.metallic.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"secondaryAlpha\":").Append(secondarySurface.alpha.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"secondaryIor\":").Append(secondarySurface.ior.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"secondaryMode\":").Append(secondarySurface.mode.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"secondaryReserved\":").Append(secondarySurface.reserved.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"stage1DistanceToSecondary\":").Append(stage1DistanceToSecondary.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"stage1DirectionToSecondary\":[").Append(FormatFloat3(stage1DirectionToSecondary, culture)).Append(']');
        _giDiagnosticBuilder.Append(",\"stage1PrimaryNormalDotSecondary\":").Append(stage1PrimaryNormalDotSecondary.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"probeClass\":\"").Append(probeClass).Append('"');
        _giDiagnosticBuilder.Append(",\"stage1InvalidReason\":\"").Append(stage1InvalidReason).Append('"');
        _giDiagnosticBuilder.Append(",\"emptyReservoirReason\":\"").Append(emptyReservoirReason).Append('"');
        _giDiagnosticBuilder.Append(",\"initialValid\":").Append(initialValid ? "true" : "false");
        _giDiagnosticBuilder.Append(",\"initialProposalPdf\":").Append(initialReservoir.proposalPdf.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"initialTargetLum\":").Append(initialReservoir.targetLum.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"initialWeightSum\":").Append(initialReservoir.weightSum.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"initialSelectedWeight\":").Append(initialReservoir.selectedWeight.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"initialSampleCountM\":").Append(initialReservoir.sampleCount.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"initialSecondaryPosition\":[").Append(FormatFloat3(initialReservoir.secondaryPosition, culture)).Append(']');
        _giDiagnosticBuilder.Append(",\"initialSecondaryNormal\":[").Append(FormatFloat3(initialReservoir.secondaryNormal, culture)).Append(']');
        _giDiagnosticBuilder.Append(",\"initialRadiance\":[").Append(FormatFloat3(initialReservoir.radiance, culture)).Append(']');
        _giDiagnosticBuilder.Append(",\"initialContribution\":[").Append(FormatFloat3(initialReservoir.contribution, culture)).Append(']');
        _giDiagnosticBuilder.Append(",\"initialPrimaryNormal\":[").Append(FormatFloat3(initialReservoir.primaryNormal, culture)).Append(']');
        _giDiagnosticBuilder.Append(",\"activeValid\":").Append(activeValid ? "true" : "false");
        _giDiagnosticBuilder.Append(",\"activeProposalPdf\":").Append(activeReservoir.proposalPdf.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"activeTargetLum\":").Append(activeReservoir.targetLum.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"activeWeightSum\":").Append(activeReservoir.weightSum.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"activeSelectedWeight\":").Append(activeReservoir.selectedWeight.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"activeSampleCountM\":").Append(activeReservoir.sampleCount.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"activeSecondaryPosition\":[").Append(FormatFloat3(activeReservoir.secondaryPosition, culture)).Append(']');
        _giDiagnosticBuilder.Append(",\"activeSecondaryNormal\":[").Append(FormatFloat3(activeReservoir.secondaryNormal, culture)).Append(']');
        _giDiagnosticBuilder.Append(",\"activeRadiance\":[").Append(FormatFloat3(activeReservoir.radiance, culture)).Append(']');
        _giDiagnosticBuilder.Append(",\"activeContribution\":[").Append(FormatFloat3(activeReservoir.contribution, culture)).Append(']');
        _giDiagnosticBuilder.Append(",\"activePrimaryNormal\":[").Append(FormatFloat3(activeReservoir.primaryNormal, culture)).Append("]}");
        _giDiagnosticBuilder.AppendLine();

        File.AppendAllText(outputPath, _giDiagnosticBuilder.ToString());
    }

    private void AppendReSTIRDISummaryJson(
        int pixelX,
        int pixelY,
        DirectLightReservoirDataCPU reservoir,
        PathContributionCPU globalColorBefore,
        PathContributionCPU globalColorAfter)
    {
        string outputPath = GetDiagnosticOutputPath("restir_di_stats.jsonl");
        CultureInfo culture = CultureInfo.InvariantCulture;
        bool reservoirValid = IsDirectReservoirNumericallyValid(reservoir);
        Vector3 directLightContribution = reservoir.contribution * reservoir.selectedWeight;
        Vector3 globalLightDelta = globalColorAfter.L - globalColorBefore.L;
        float globalLightDeltaLum = Mathf.Max(globalLightDelta.x, Mathf.Max(globalLightDelta.y, globalLightDelta.z));

        _giDiagnosticBuilder.Clear();
        _giDiagnosticBuilder.Append("{\"frameIndex\":").Append(frameId);
        _giDiagnosticBuilder.Append(",\"sampleCount\":").Append(sampleCount);
        _giDiagnosticBuilder.Append(",\"sceneName\":\"").Append(SceneManager.GetActiveScene().name).Append('"');
        _giDiagnosticBuilder.Append(",\"pixelX\":").Append(pixelX);
        _giDiagnosticBuilder.Append(",\"pixelY\":").Append(pixelY);
        _giDiagnosticBuilder.Append(",\"reservoirValid\":").Append(reservoirValid ? "true" : "false");
        _giDiagnosticBuilder.Append(",\"lightType\":").Append(reservoir.lightType);
        _giDiagnosticBuilder.Append(",\"lightIndex\":").Append(reservoir.lightIndex);
        _giDiagnosticBuilder.Append(",\"proposalPdf\":").Append(reservoir.proposalPdf.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"targetLum\":").Append(reservoir.targetLum.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"weightSum\":").Append(reservoir.weightSum.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"selectedWeight\":").Append(reservoir.selectedWeight.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"sampleCountM\":").Append(reservoir.sampleCount);
        _giDiagnosticBuilder.Append(",\"maxDist\":").Append(reservoir.maxDist.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"origin\":[").Append(FormatFloat3(reservoir.origin, culture)).Append(']');
        _giDiagnosticBuilder.Append(",\"direction\":[").Append(FormatFloat3(reservoir.direction, culture)).Append(']');
        _giDiagnosticBuilder.Append(",\"surfaceNormal\":[").Append(FormatFloat3(reservoir.surfaceNormal, culture)).Append(']');
        _giDiagnosticBuilder.Append(",\"diContribution\":[").Append(FormatFloat3(directLightContribution, culture)).Append(']');
        _giDiagnosticBuilder.Append(",\"globalLightBefore\":[").Append(FormatFloat3(globalColorBefore.L, culture)).Append(']');
        _giDiagnosticBuilder.Append(",\"globalLightAfter\":[").Append(FormatFloat3(globalColorAfter.L, culture)).Append(']');
        _giDiagnosticBuilder.Append(",\"globalLightDelta\":[").Append(FormatFloat3(globalLightDelta, culture)).Append(']');
        _giDiagnosticBuilder.Append(",\"globalLightDeltaLum\":").Append(globalLightDeltaLum.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"globalLightDeltaPositive\":").Append((IsFiniteVector(globalLightDelta) && globalLightDeltaLum > 0.0f) ? "true" : "false");
        _giDiagnosticBuilder.Append("}");
        _giDiagnosticBuilder.AppendLine();

        File.AppendAllText(outputPath, _giDiagnosticBuilder.ToString());
    }

    private void AppendReSTIRGISpatialSummaryJson(
        int selectedProbeId,
        int selectedProbeX,
        int selectedProbeY,
        int validSurfaceNeighbors,
        int compatibleNeighbors,
        int activeReservoirNeighbors,
        Vector4 spatialShaderStats,
        Vector4 spatialShaderFailStats0,
        Vector4 spatialShaderFailStats1,
        Vector4 spatialShaderSelectionStats,
        Vector4 spatialShaderNormalizationStats)
    {
        string outputPath = GetDiagnosticOutputPath("restir_gi_spatial_stats.jsonl");
        CultureInfo culture = CultureInfo.InvariantCulture;

        _giDiagnosticBuilder.Clear();
        _giDiagnosticBuilder.Append("{\"frameIndex\":").Append(frameId);
        _giDiagnosticBuilder.Append(",\"sampleCount\":").Append(sampleCount);
        _giDiagnosticBuilder.Append(",\"sceneName\":\"").Append(SceneManager.GetActiveScene().name).Append('"');
        _giDiagnosticBuilder.Append(",\"selectedProbeId\":").Append(selectedProbeId);
        _giDiagnosticBuilder.Append(",\"selectedProbeX\":").Append(selectedProbeX);
        _giDiagnosticBuilder.Append(",\"selectedProbeY\":").Append(selectedProbeY);
        _giDiagnosticBuilder.Append(",\"validSurfaceNeighbors\":").Append(validSurfaceNeighbors);
        _giDiagnosticBuilder.Append(",\"compatibleNeighbors\":").Append(compatibleNeighbors);
        _giDiagnosticBuilder.Append(",\"activeReservoirNeighbors\":").Append(activeReservoirNeighbors);
        _giDiagnosticBuilder.Append(",\"spatialShaderCompatibleNeighbors\":").Append(spatialShaderStats.x.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"spatialShaderReevaluateNeighbors\":").Append(spatialShaderStats.y.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"spatialShaderJacobianNeighbors\":").Append(spatialShaderStats.z.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"spatialShaderCombinedNeighbors\":").Append(spatialShaderStats.w.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"spatialShaderReevaluateFailInvalidSample\":").Append(spatialShaderFailStats0.x.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"spatialShaderReevaluateFailInvalidSurface\":").Append(spatialShaderFailStats0.y.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"spatialShaderReevaluateFailDistance\":").Append(spatialShaderFailStats0.z.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"spatialShaderReevaluateFailBrdf\":").Append(spatialShaderFailStats0.w.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"spatialShaderReevaluateFailBackfacing\":").Append(spatialShaderFailStats1.x.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"spatialShaderReevaluateFailZeroTarget\":").Append(spatialShaderFailStats1.y.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"spatialSelectedNeighborIndex\":").Append(spatialShaderFailStats1.z.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"spatialSelectedTargetPdf\":").Append(spatialShaderFailStats1.w.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"spatialSelectedNeighborOriginalProposalPdf\":").Append(spatialShaderSelectionStats.x.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"spatialSelectedNeighborReuseProposalPdf\":").Append(spatialShaderSelectionStats.y.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"spatialSelectedNeighborJacobian\":").Append(spatialShaderSelectionStats.z.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"spatialSelectedNeighborTargetPdf\":").Append(spatialShaderSelectionStats.w.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"spatialPi\":").Append(spatialShaderNormalizationStats.x.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"spatialPiSum\":").Append(spatialShaderNormalizationStats.y.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"spatialNormalizationDenominator\":").Append(spatialShaderNormalizationStats.z.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"spatialCurrentTargetPdf\":").Append(spatialShaderNormalizationStats.w.ToString("R", culture));
        _giDiagnosticBuilder.Append("}");
        _giDiagnosticBuilder.AppendLine();

        File.AppendAllText(outputPath, _giDiagnosticBuilder.ToString());
    }

    private void AppendReSTIRGITemporalSummaryJson(
        int selectedProbeId,
        int selectedProbeX,
        int selectedProbeY,
        IndirectReservoirDataCPU temporalProbe,
        Vector4 temporalDebug0,
        Vector4 temporalDebug1,
        Vector4 temporalDebug2)
    {
        string outputPath = GetDiagnosticOutputPath("restir_gi_temporal_stats.jsonl");
        CultureInfo culture = CultureInfo.InvariantCulture;

        _giDiagnosticBuilder.Clear();
        _giDiagnosticBuilder.Append("{\"frameIndex\":").Append(frameId);
        _giDiagnosticBuilder.Append(",\"sampleCount\":").Append(sampleCount);
        _giDiagnosticBuilder.Append(",\"sceneName\":\"").Append(SceneManager.GetActiveScene().name).Append('"');
        _giDiagnosticBuilder.Append(",\"selectedProbeId\":").Append(selectedProbeId);
        _giDiagnosticBuilder.Append(",\"selectedProbeX\":").Append(selectedProbeX);
        _giDiagnosticBuilder.Append(",\"selectedProbeY\":").Append(selectedProbeY);
        _giDiagnosticBuilder.Append(",\"proposalPdf\":").Append(temporalProbe.proposalPdf.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"targetLum\":").Append(temporalProbe.targetLum.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"weightSum\":").Append(temporalProbe.weightSum.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"selectedWeight\":").Append(temporalProbe.selectedWeight.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"sampleCountM\":").Append(temporalProbe.sampleCount.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"selectedPrevious\":").Append(temporalDebug0.x > 0.5f ? "true" : "false");
        _giDiagnosticBuilder.Append(",\"selectedTemporalOffsetIndex\":").Append(temporalDebug0.y.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"selectedFallbackSample\":").Append(temporalDebug0.z > 0.5f ? "true" : "false");
        _giDiagnosticBuilder.Append(",\"currentTargetPdf\":").Append(temporalDebug0.w.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"selectedTargetPdf\":").Append(temporalDebug1.x.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"selectedPrevTargetPdf\":").Append(temporalDebug1.y.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"selectedPrevOriginalProposalPdf\":").Append(temporalDebug1.z.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"selectedPrevReuseProposalPdf\":").Append(temporalDebug1.w.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"selectedPrevJacobian\":").Append(temporalDebug2.x.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"temporalPi\":").Append(temporalDebug2.y.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"temporalPiSum\":").Append(temporalDebug2.z.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"temporalNormalizationDenominator\":").Append(temporalDebug2.w.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"radiance\":[").Append(FormatFloat3(temporalProbe.radiance, culture)).Append(']');
        _giDiagnosticBuilder.Append(",\"contribution\":[").Append(FormatFloat3(temporalProbe.contribution, culture)).Append("]}");
        _giDiagnosticBuilder.AppendLine();

        File.AppendAllText(outputPath, _giDiagnosticBuilder.ToString());
    }

    private void AppendReSTIRGIFinalSummaryJson(
        int selectedProbeId,
        int selectedProbeX,
        int selectedProbeY,
        IndirectReservoirDataCPU initialProbe,
        IndirectReservoirDataCPU finalProbe,
        PathContributionCPU globalColorBeforeFinal,
        PathContributionCPU globalColorAfterFinal,
        Vector4 finalDebug0,
        Vector4 finalDebug1,
        Vector4 finalDebug2)
    {
        string outputPath = GetDiagnosticOutputPath("restir_gi_final_stats.jsonl");
        CultureInfo culture = CultureInfo.InvariantCulture;
        Vector3 finalContribution = new Vector3(finalDebug1.x, finalDebug1.y, finalDebug1.z);
        Vector3 globalLightDelta = globalColorAfterFinal.L - globalColorBeforeFinal.L;
        float finalContributionLum = Mathf.Max(finalContribution.x, Mathf.Max(finalContribution.y, finalContribution.z));
        float globalLightDeltaLum = Mathf.Max(globalLightDelta.x, Mathf.Max(globalLightDelta.y, globalLightDelta.z));
        bool finalContributionPositive = IsFiniteVector(finalContribution) && finalContributionLum > 0.0f;

        _giDiagnosticBuilder.Clear();
        _giDiagnosticBuilder.Append("{\"frameIndex\":").Append(frameId);
        _giDiagnosticBuilder.Append(",\"sampleCount\":").Append(sampleCount);
        _giDiagnosticBuilder.Append(",\"sceneName\":\"").Append(SceneManager.GetActiveScene().name).Append('"');
        _giDiagnosticBuilder.Append(",\"selectedProbeId\":").Append(selectedProbeId);
        _giDiagnosticBuilder.Append(",\"selectedProbeX\":").Append(selectedProbeX);
        _giDiagnosticBuilder.Append(",\"selectedProbeY\":").Append(selectedProbeY);
        _giDiagnosticBuilder.Append(",\"initialValid\":").Append(IsReservoirNumericallyValid(initialProbe) ? "true" : "false");
        _giDiagnosticBuilder.Append(",\"finalValid\":").Append(IsReservoirNumericallyValid(finalProbe) ? "true" : "false");
        _giDiagnosticBuilder.Append(",\"hasFinalSample\":").Append(finalDebug0.x > 0.5f ? "true" : "false");
        _giDiagnosticBuilder.Append(",\"hasInitialSample\":").Append(finalDebug0.y > 0.5f ? "true" : "false");
        _giDiagnosticBuilder.Append(",\"misFinalWeight\":").Append(finalDebug0.z.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"misInitialWeight\":").Append(finalDebug0.w.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"finalContribution\":[").Append(FormatFloat3(finalContribution, culture)).Append(']');
        _giDiagnosticBuilder.Append(",\"finalContributionLum\":").Append(finalContributionLum.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"finalContributionPositive\":").Append(finalContributionPositive ? "true" : "false");
        _giDiagnosticBuilder.Append(",\"finalContributionFinite\":").Append(IsFiniteVector(finalContribution) ? "true" : "false");
        _giDiagnosticBuilder.Append(",\"globalLightBefore\":[").Append(FormatFloat3(globalColorBeforeFinal.L, culture)).Append(']');
        _giDiagnosticBuilder.Append(",\"globalLightAfter\":[").Append(FormatFloat3(globalColorAfterFinal.L, culture)).Append(']');
        _giDiagnosticBuilder.Append(",\"globalLightDelta\":[").Append(FormatFloat3(globalLightDelta, culture)).Append(']');
        _giDiagnosticBuilder.Append(",\"globalLightDeltaLum\":").Append(globalLightDeltaLum.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"globalLightDeltaPositive\":").Append((IsFiniteVector(globalLightDelta) && globalLightDeltaLum > 0.0f) ? "true" : "false");
        _giDiagnosticBuilder.Append(",\"initialTargetLum\":").Append(initialProbe.targetLum.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"finalTargetLum\":").Append(finalProbe.targetLum.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"initialSelectedWeight\":").Append(initialProbe.selectedWeight.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"finalSelectedWeight\":").Append(finalProbe.selectedWeight.ToString("R", culture));
        _giDiagnosticBuilder.Append(",\"debugFlags\":[").Append(finalDebug2.x.ToString("R", culture)).Append(',')
            .Append(finalDebug2.y.ToString("R", culture)).Append(',')
            .Append(finalDebug2.z.ToString("R", culture)).Append(']');
        _giDiagnosticBuilder.Append("}");
        _giDiagnosticBuilder.AppendLine();

        File.AppendAllText(outputPath, _giDiagnosticBuilder.ToString());
    }

    private string GetDiagnosticOutputPath(string fileName)
    {
        if (string.IsNullOrEmpty(_currentDiagnosticOutputDir))
        {
            string rootOutputDir = Path.Combine(Application.dataPath, "..", "Tools", "Output");
            Directory.CreateDirectory(rootOutputDir);

            string timestamp = DateTime.Now.ToString("yyyy-MM-dd_HHmmss", CultureInfo.InvariantCulture);
            string candidateDir = Path.Combine(rootOutputDir, timestamp);
            int collisionSuffix = 1;
            while (Directory.Exists(candidateDir))
            {
                candidateDir = Path.Combine(rootOutputDir, timestamp + "_" + collisionSuffix.ToString(CultureInfo.InvariantCulture));
                collisionSuffix++;
            }

            Directory.CreateDirectory(candidateDir);
            _currentDiagnosticOutputDir = candidateDir;
            Debug.Log($"[ReSTIR Diagnostics] Writing logs to {_currentDiagnosticOutputDir}");
        }

        return Path.Combine(_currentDiagnosticOutputDir, fileName);
    }

    private static string ClassifyReSTIRGIProbe(SecondarySurfaceDataCPU secondarySurface, bool initialValid, bool activeValid)
    {
        if (activeValid)
            return "reservoir_reusable";
        if (initialValid)
            return "initial_only";
        if (secondarySurface.flags >= 8.0f)
            return "bypass";
        if (secondarySurface.flags < 0.0f)
            return "invalid_stage1";
        return "nonreusable_empty";
    }

    private static string DescribeReSTIRGIStage1InvalidReason(SecondarySurfaceDataCPU secondarySurface)
    {
        if (secondarySurface.flags >= 0.0f)
            return "none";
        if (secondarySurface.flags <= -11.0f)
            return "primary_miss";
        if (secondarySurface.flags <= -10.0f)
            return "stage1_uninitialized";
        if (secondarySurface.flags <= -3.0f)
            return "proposal_pdf_zero";
        if (secondarySurface.flags <= -2.0f)
        {
            if (secondarySurface.reserved >= 4.5f)
                return "perfect_metal_specular_selected";
            if (secondarySurface.reserved >= 3.5f)
                return "perfect_metal_specular_not_selected";
            if (secondarySurface.reserved >= 2.5f)
                return "perfect_metal_zero_response";
            if (secondarySurface.reserved >= 1.5f)
                return "ggx_specular_backfacing";
            if (secondarySurface.reserved >= 0.5f)
                return "delta_specular_backfacing";
            return "throughput_zero";
        }
        return "unknown_negative_flag";
    }

    private static string DiagnoseReSTIRGIEmptyReservoirReason(
        HitDataCPU primaryHit,
        SecondarySurfaceDataCPU secondarySurface,
        IndirectReservoirDataCPU initialReservoir,
        out Vector3 directionToSecondary,
        out float distanceToSecondary,
        out float primaryNormalDotSecondary)
    {
        directionToSecondary = Vector3.zero;
        distanceToSecondary = 0.0f;
        primaryNormalDotSecondary = 0.0f;

        if (IsReservoirNumericallyValid(initialReservoir))
            return "has_initial_reservoir";

        if (secondarySurface.flags < 0.0f)
            return "invalid_stage1_" + DescribeReSTIRGIStage1InvalidReason(secondarySurface);

        if (!IsPrimaryHitValid(primaryHit))
            return "invalid_primary_hit";

        if (!IsFinitePositive(secondarySurface.proposalPdf))
            return "secondary_proposal_invalid";

        Vector3 toSecondary = secondarySurface.position - primaryHit.position;
        distanceToSecondary = toSecondary.magnitude;
        if (!(distanceToSecondary > 1e-5f) || float.IsNaN(distanceToSecondary) || float.IsInfinity(distanceToSecondary))
            return "secondary_distance_invalid";

        directionToSecondary = toSecondary / distanceToSecondary;
        if (!IsFiniteVector(directionToSecondary))
            return "secondary_direction_invalid";

        Vector3 primaryNormal = primaryHit.normal.normalized;
        if (!IsFiniteVector(primaryNormal) || primaryNormal.sqrMagnitude <= 1e-8f)
            return "primary_normal_invalid";

        primaryNormalDotSecondary = Vector3.Dot(primaryNormal, directionToSecondary);
        if (primaryNormalDotSecondary <= 0.0f)
            return "primary_backfacing_secondary";

        if (secondarySurface.emissionRadiance.sqrMagnitude <= 1e-10f &&
            secondarySurface.albedo.sqrMagnitude <= 1e-10f &&
            secondarySurface.flags < 0.5f)
            return "secondary_radiance_likely_zero";

        return "target_luminance_rejected";
    }

    private static bool IsSpatialProbeCompatible(HitDataCPU current, HitDataCPU neighbor)
    {
        float normalDot = Vector3.Dot(current.normal.normalized, neighbor.normal.normalized);
        if (normalDot < 0.9f)
            return false;

        Vector3 delta = neighbor.position - current.position;
        if (Mathf.Abs(Vector3.Dot(delta, current.normal.normalized)) > 0.05f)
            return false;
        if (Mathf.Abs(Vector3.Dot(delta, neighbor.normal.normalized)) > 0.05f)
            return false;

        return Mathf.Abs(current.mode - neighbor.mode) <= 0.25f;
    }

    private static bool IsIndirectReservoirValid(IndirectReservoirDataCPU reservoir)
    {
        return reservoir.targetLum > 0f &&
               reservoir.weightSum > 0f &&
               reservoir.sampleCount > 0f &&
               reservoir.proposalPdf > 0f;
    }

    private static bool IsDirectReservoirNumericallyValid(DirectLightReservoirDataCPU reservoir)
    {
        return IsFiniteVector(reservoir.origin) &&
               IsFinitePositiveOrZero(reservoir.maxDist) &&
               IsFiniteVector(reservoir.direction) &&
               IsFinitePositiveOrZero(reservoir.targetLum) &&
               IsFiniteVector(reservoir.contribution) &&
               IsFinitePositiveOrZero(reservoir.weightSum) &&
               IsFiniteVector(reservoir.surfaceNormal) &&
               IsFinitePositiveOrZero(reservoir.proposalPdf) &&
               reservoir.targetLum > 0.0f &&
               reservoir.weightSum > 0.0f &&
               reservoir.selectedWeight > 0.0f &&
               reservoir.sampleCount > 0u;
    }

    private static string FormatFloat3(Vector3 value, CultureInfo culture)
    {
        return value.x.ToString("R", culture) + "," +
               value.y.ToString("R", culture) + "," +
               value.z.ToString("R", culture);
    }

    private static bool IsReservoirNumericallyValid(IndirectReservoirDataCPU reservoir)
    {
        return IsFinitePositiveOrZero(reservoir.proposalPdf) &&
               IsFinitePositiveOrZero(reservoir.targetLum) &&
               IsFinitePositiveOrZero(reservoir.weightSum) &&
               IsFinitePositiveOrZero(reservoir.selectedWeight) &&
               IsFinitePositiveOrZero(reservoir.sampleCount) &&
               IsFiniteVector(reservoir.secondaryPosition) &&
               IsFiniteVector(reservoir.secondaryNormal) &&
               IsFiniteVector(reservoir.radiance) &&
               IsFiniteVector(reservoir.contribution) &&
               IsFiniteVector(reservoir.primaryNormal) &&
                reservoir.targetLum > 0.0f &&
                reservoir.weightSum > 0.0f;
    }

    private static bool IsPrimaryHitValid(HitDataCPU primaryHit)
    {
        return IsFinitePositive(primaryHit.distance) &&
               primaryHit.distance < 1e19f &&
               IsFiniteVector(primaryHit.position) &&
               IsFiniteVector(primaryHit.normal) &&
               primaryHit.normal.sqrMagnitude > 1e-8f;
    }

    private static bool IsFinitePositiveOrZero(float value)
    {
        return !(float.IsNaN(value) || float.IsInfinity(value)) && value >= 0.0f;
    }

    private static bool IsFinitePositive(float value)
    {
        return !(float.IsNaN(value) || float.IsInfinity(value)) && value > 0.0f;
    }

    private static bool IsFiniteVector(Vector3 value)
    {
        return IsFiniteComponent(value.x) && IsFiniteComponent(value.y) && IsFiniteComponent(value.z);
    }

    private static bool IsFiniteComponent(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }

    private void Update()
    {
        bool resetRequired = false;

        if (_OldDenoise != Denoise)
        {
            _OldDenoise = Denoise;
            resetRequired = true;
        }

        LightManager.Instance.UpdateLights();
        int lightStateHash = LightManager.Instance.ComputeLightStateHash();
        if (lightStateHash != _lastLightStateHash)
        {
            _lastLightStateHash = lightStateHash;
            resetRequired = true;
        }

        bool materialChanged = BVHBuilder.ReloadMaterials();
        resetRequired |= materialChanged;

        if (HaveRuntimeSettingsChanged())
        {
            CacheRuntimeSettings();
            resetRequired = true;
        }

        if (resetRequired)
            ResetSampleCount();

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
        RestoreFrameRateLimit();
        ReleaseRenderTargets();
        ReleaseBuffers();
        ReleaseCommandBuffer();
        ReleaseMaterials();
        BVHBuilder.Destroy();
    }

    private void OnApplicationQuit()
    {
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

    void ResetSampleCount()
    {
        sampleCount = 0;
        _currentDiagnosticOutputDir = null;
        _hasPrimarySurfaceHistory = false;
        _hasDirectRestirHistory = false;
        _hasIndirectRestirHistory = false;
        _lastDirectReservoirOutputIdx = 0;
        _lastIndirectReservoirOutputIdx = 0;
    }

    void ResetAccumulationOnly()
    {
        sampleCount = 0;
        _currentDiagnosticOutputDir = null;
        _hasDirectRestirHistory = false;
        _hasIndirectRestirHistory = false;
        _lastDirectReservoirOutputIdx = 0;
        _lastIndirectReservoirOutputIdx = 0;
    }

}
