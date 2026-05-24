using System;
using System.Collections.Generic;
using System.Reflection;
using System.Runtime.InteropServices;
using UnityEngine;
using UnityEngine.Rendering;
#if UNITY_EDITOR
using UnityEditor;
#endif

public enum DirectLightDebugView
{
    Off = 0,
    ResolvedDirect = 1,
    RISEstimate = 2,
    RISError = 3,
    ReservoirStatus = 4,
    NeighborReplayStages = 5,
}

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
    [SerializeField] bool UseDirectLightReservoirRIS = false;
    [SerializeField] bool UseDirectLightReservoirNeighborReuse = false;
    [SerializeField, Range(1, 8)] int DirectLightNeighborReuseCount = 4;
    [SerializeField] DirectLightDebugView DirectLightDebugViewMode = DirectLightDebugView.Off;
    [SerializeField] Vector2Int DirectLightDiagnosticPixelOffset = Vector2Int.zero;
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
    private int kernelCopyDirectLightReservoirHistory;
    private int kernelCopyPrimarySurfaceHistory;
    private int kernelTransfer;

    // Multi-pass buffers
    private ComputeBuffer _globalRaysA;
    private ComputeBuffer _globalRaysB;
    private ComputeBuffer _globalHits;
    private ComputeBuffer _primarySurfaceHistory;
    private ComputeBuffer _primarySurfaceHistoryPrev;
    private ComputeBuffer _shadowRays;
    private ComputeBuffer _directLightReservoirs;
    private ComputeBuffer _directLightReservoirsPrev;
    private ComputeBuffer _directLightDebugOutput;
    private ComputeBuffer _directLightDiagnostics;
    private ComputeBuffer _globalColors;
    private ComputeBuffer _bufferSizes;
    private ComputeBuffer _indirectArgs;

    // Struct sizes (must match HLSL layout)
    private const int RayDataStride = 48;         // float3+float3+uint+uint+float3+float = 12+12+4+4+12+4
    private const int HitDataStride = 76;          // 4×(float3+scalar) + 2×float + float = 4×16+8+4
    private const int ShadowRayDataStride = 48;    // 3×(float3+scalar)
    private const int DirectLightReservoirStride = 80; // matches DirectLightReservoirData (5 float4 rows)
    private const int DirectLightDebugOutputStride = 12; // float3
    private const int DirectLightDiagnosticsStride = 16; // float4
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
    private bool _oldUseDirectLightReservoirRIS = false;
    private bool _oldUseDirectLightReservoirNeighborReuse = false;
    private int _oldDirectLightNeighborReuseCount = 4;
    private bool _hasPrimarySurfaceHistory = false;
    private DirectLightDebugView _oldDirectLightDebugViewMode = DirectLightDebugView.Off;
    private bool _hasDirectLightReservoirHistory = false;
    private Matrix4x4 _previousCameraViewProjection = Matrix4x4.identity;
    private readonly Vector4[] _directLightDiagnosticsData = new Vector4[22];
    private readonly Vector4[] _directLightDiagnosticsSnapshot = new Vector4[22];
    private bool _hasDirectLightDiagnosticsSnapshot = false;
    private static readonly Vector3 LuminanceWeights = new Vector3(0.2126f, 0.7152f, 0.0722f);
    private int _previousTargetFrameRate = -1;
    private int _previousVSyncCount = -1;
    private int _previousRenderFrameInterval = -1;
    private bool _hasCapturedFrameRateState = false;
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
        kernelCopyDirectLightReservoirHistory = tracingShader.FindKernel("kernel_copy_direct_light_reservoir_history");
        kernelCopyPrimarySurfaceHistory = tracingShader.FindKernel("kernel_copy_primary_surface_history");
        kernelTransfer = tracingShader.FindKernel("TransferKernel");

        // Pre-allocate reusable arrays and names
        bvhKernels = new int[] { kernelGenerate, kernelTrace, kernelShade, kernelShadow };
        lightKernels = new int[] { kernelGenerate, kernelTrace, kernelShade };
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
        _hasDirectLightReservoirHistory = false;

        int pixelCount = width * height;

        _globalRaysA = new ComputeBuffer(pixelCount, RayDataStride);
        _globalRaysB = new ComputeBuffer(pixelCount, RayDataStride);
        _globalHits = new ComputeBuffer(pixelCount, HitDataStride);
        _primarySurfaceHistory = new ComputeBuffer(pixelCount, HitDataStride);
        _primarySurfaceHistoryPrev = new ComputeBuffer(pixelCount, HitDataStride);
        // Keep headroom for future direct-light candidates/reservoir experiments.
        _shadowRays = new ComputeBuffer(pixelCount * 2, ShadowRayDataStride);
        _directLightReservoirs = new ComputeBuffer(pixelCount, DirectLightReservoirStride);
        _directLightReservoirsPrev = new ComputeBuffer(pixelCount, DirectLightReservoirStride);
        _directLightDebugOutput = new ComputeBuffer(pixelCount, DirectLightDebugOutputStride);
        _directLightDiagnostics = new ComputeBuffer(_directLightDiagnosticsData.Length, DirectLightDiagnosticsStride);
        _globalColors = new ComputeBuffer(pixelCount, PathContributionStride);
        _bufferSizes = new ComputeBuffer(TraceDepth + 1, BufferSizeDataStride);
        _indirectArgs = new ComputeBuffer(3, IndirectArgsStride, ComputeBufferType.IndirectArguments);

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
        _directLightReservoirsPrev?.Release(); _directLightReservoirsPrev = null;
        _directLightDebugOutput?.Release(); _directLightDebugOutput = null;
        _directLightDiagnostics?.Release(); _directLightDiagnostics = null;
        _globalColors?.Release(); _globalColors = null;
        _bufferSizes?.Release(); _bufferSizes = null;
        _indirectArgs?.Release(); _indirectArgs = null;
        _hasDirectLightDiagnosticsSnapshot = false;
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
            if (IsDirectLightDebugViewActive())
                Graphics.Blit(target, destination);
            else if (Denoise)
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

        Array.Clear(_directLightDiagnosticsData, 0, _directLightDiagnosticsData.Length);
        _directLightDiagnostics.SetData(_directLightDiagnosticsData);

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
            bool readA = true;
            for (int bounce = 0; bounce < TraceDepth; bounce++)
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
        tracingShader.Dispatch(kernelCopyDirectLightReservoirHistory, (pixelCount + 63) / 64, 1, 1);
        CaptureDirectLightDiagnosticsSnapshot();

        if (IsDirectLightDebugViewActive())
        {
            Graphics.Blit(target, destination);
        }
        else if (Denoise)
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
        _hasDirectLightReservoirHistory = true;
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

        // Set multi-pass buffers on all relevant kernels
        tracingShader.SetBuffer(kernelGenerate, "GlobalRays", _globalRaysA);
        tracingShader.SetBuffer(kernelGenerate, "GlobalColors", _globalColors);
        tracingShader.SetBuffer(kernelGenerate, "GlobalHits", _globalHits);
        tracingShader.SetBuffer(kernelGenerate, "PrimarySurfaceHistory", _primarySurfaceHistory);
        tracingShader.SetBuffer(kernelGenerate, "PrimarySurfaceHistoryPrev", _primarySurfaceHistoryPrev);
        tracingShader.SetBuffer(kernelGenerate, "DirectLightReservoirs", _directLightReservoirs);
        tracingShader.SetBuffer(kernelGenerate, "DirectLightReservoirsPrev", _directLightReservoirsPrev);
        tracingShader.SetBuffer(kernelGenerate, "DirectLightDebugOutput", _directLightDebugOutput);
        tracingShader.SetBuffer(kernelTrace, "BufferSizes", _bufferSizes);
        tracingShader.SetBuffer(kernelShade, "GlobalColors", _globalColors);
        tracingShader.SetBuffer(kernelShade, "PrimarySurfaceHistory", _primarySurfaceHistory);
        tracingShader.SetBuffer(kernelShade, "PrimarySurfaceHistoryPrev", _primarySurfaceHistoryPrev);
        tracingShader.SetBuffer(kernelShade, "ShadowRaysBuffer", _shadowRays);
        tracingShader.SetBuffer(kernelShade, "DirectLightReservoirs", _directLightReservoirs);
        tracingShader.SetBuffer(kernelShade, "DirectLightReservoirsPrev", _directLightReservoirsPrev);
        tracingShader.SetBuffer(kernelShade, "DirectLightDebugOutput", _directLightDebugOutput);
        tracingShader.SetBuffer(kernelShade, "DirectLightDiagnostics", _directLightDiagnostics);
        tracingShader.SetBuffer(kernelShade, "BufferSizes", _bufferSizes);
        tracingShader.SetBuffer(kernelShadow, "ShadowRaysBuffer", _shadowRays);
        tracingShader.SetBuffer(kernelShadow, "GlobalColors", _globalColors);
        tracingShader.SetBuffer(kernelShadow, "DirectLightReservoirs", _directLightReservoirs);
        tracingShader.SetBuffer(kernelShadow, "DirectLightReservoirsPrev", _directLightReservoirsPrev);
        tracingShader.SetBuffer(kernelShadow, "DirectLightDebugOutput", _directLightDebugOutput);
        tracingShader.SetBuffer(kernelShadow, "BufferSizes", _bufferSizes);
        tracingShader.SetBuffer(kernelCopyPrimarySurfaceHistory, "PrimarySurfaceHistory", _primarySurfaceHistory);
        tracingShader.SetBuffer(kernelCopyPrimarySurfaceHistory, "PrimarySurfaceHistoryPrevRW", _primarySurfaceHistoryPrev);
        tracingShader.SetBuffer(kernelCopyDirectLightReservoirHistory, "DirectLightReservoirs", _directLightReservoirs);
        tracingShader.SetBuffer(kernelCopyDirectLightReservoirHistory, "DirectLightReservoirsPrevRW", _directLightReservoirsPrev);
        tracingShader.SetBuffer(kernelTransfer, "BufferSizes", _bufferSizes);
        tracingShader.SetBuffer(kernelTransfer, "IndirectArgs", _indirectArgs);
        tracingShader.SetBuffer(kernelFinalize, "GlobalColors", _globalColors);
        tracingShader.SetBuffer(kernelFinalize, "DirectLightDebugOutput", _directLightDebugOutput);

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
        tracingShader.SetBool("_UseDirectLightReservoirRIS", UseDirectLightReservoirRIS);
        tracingShader.SetBool("_UseDirectLightReservoirNeighborReuse", UseDirectLightReservoirNeighborReuse);
        tracingShader.SetInt("_DirectLightNeighborReuseCount", DirectLightNeighborReuseCount);
        tracingShader.SetBool("_HasDirectLightReservoirHistory", _hasDirectLightReservoirHistory);
        tracingShader.SetInt("_DirectLightDebugView", (int)DirectLightDebugViewMode);
        tracingShader.SetInt("_DirectLightDebugPixelIndex", GetDirectLightDebugPixelIndex());
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

    private void CaptureDirectLightDiagnosticsSnapshot()
    {
        if (_directLightDiagnostics == null)
        {
            _hasDirectLightDiagnosticsSnapshot = false;
            return;
        }

        _directLightDiagnostics.GetData(_directLightDiagnosticsData);
        Array.Copy(_directLightDiagnosticsData, _directLightDiagnosticsSnapshot, _directLightDiagnosticsData.Length);
        _hasDirectLightDiagnosticsSnapshot = true;
    }

    private int GetDirectLightDebugPixelIndex()
    {
        if (_currentRenderWidth <= 0 || _currentRenderHeight <= 0)
            return 0;

        int x = GetDirectLightDebugPixelX();
        int y = GetDirectLightDebugPixelY();
        return y * _currentRenderWidth + x;
    }

    public string[] GetDirectLightDiagnosticsSummaryLines()
    {
        if (_directLightDiagnostics == null || !_hasDirectLightDiagnosticsSnapshot)
            return new[] { "Center pixel diagnostics unavailable." };

        Vector4 flags = _directLightDiagnosticsSnapshot[0];
        Vector4 current = _directLightDiagnosticsSnapshot[1];
        Vector4 previous = _directLightDiagnosticsSnapshot[2];
        Vector4 weights = _directLightDiagnosticsSnapshot[3];
        Vector4 meta = _directLightDiagnosticsSnapshot[4];
        Vector4 candidate = _directLightDiagnosticsSnapshot[5];
        Vector4 firstDraw = _directLightDiagnosticsSnapshot[6];
        Vector4 firstMeta = _directLightDiagnosticsSnapshot[7];
        Vector4 risLocal = _directLightDiagnosticsSnapshot[8];
        Vector4 risFinal = _directLightDiagnosticsSnapshot[9];
        Vector4 material = _directLightDiagnosticsSnapshot[10];
        Vector4 shading = _directLightDiagnosticsSnapshot[11];
        Vector4 rawMaterial = _directLightDiagnosticsSnapshot[12];
        Vector4 rawThroughput = _directLightDiagnosticsSnapshot[13];
        Vector4 rawLight = _directLightDiagnosticsSnapshot[14];
        Vector4 sampleScalars = _directLightDiagnosticsSnapshot[15];
        Vector4 rawContribution = _directLightDiagnosticsSnapshot[16];
        Vector4 finalSampleScalars = _directLightDiagnosticsSnapshot[17];
        Vector4 finalRawContribution = _directLightDiagnosticsSnapshot[18];
        Vector4 neighborCounts = _directLightDiagnosticsSnapshot[19];
        Vector4 neighborState = _directLightDiagnosticsSnapshot[20];
        Vector4 temporalReplayIndex = _directLightDiagnosticsSnapshot[21];
        float rawAlbedoLum = ComputeDiagnosticLuminance(new Vector3(rawMaterial.x, rawMaterial.y, rawMaterial.z));
        float rawThroughputLum = ComputeDiagnosticLuminance(new Vector3(rawThroughput.x, rawThroughput.y, rawThroughput.z));
        float rawLightLum = ComputeDiagnosticLuminance(new Vector3(rawLight.x, rawLight.y, rawLight.z) * rawLight.w);
        float materialAlbedoLum = rawMaterial.w > 0.0f ? rawAlbedoLum : material.w;
        float shadingThroughputLum = rawThroughput.w > 0.0f ? rawThroughputLum : shading.w;
        float shadingDirLightLum = rawLight.w > 0.0f ? rawLightLum : shading.z;
        string shadingOrCompatibilityLine = UseDirectLightReservoirNeighborReuse
            ? $"Temporal Compatibility: normalDot={shading.x:G6} planeCurrent={shading.y:G6} planePrevious={shading.z:G6} modeDelta={shading.w:G6}"
            : $"Shading: NdotV={shading.x:G6} dirFirstBrdfLum={shading.y:G6} dirLightLum={shadingDirLightLum:G6} throughputLum={shadingThroughputLum:G6}";

        return new[]
        {
            $"Direct Light Diagnostics ({GetDirectLightDebugPixelX()}, {GetDirectLightDebugPixelY()})",
            $"PrimaryHit={flags.x:0} CurrentStored={flags.y:0} PrevStored={flags.z:0} PrevSourceValid={flags.w:0}",
            $"History Current: targetLum={current.x:G6} weightSum={current.y:G6} sampleCount={current.z:G6} maxDist={current.w:G6}",
            $"History Prev: targetLum={previous.x:G6} weightSum={previous.y:G6} sampleCount={previous.z:G6} maxDist={previous.w:G6}",
            $"PDF/Weight: currentPdf={weights.x:G6} currentSelectedWeight={weights.y:G6} prevPdf={weights.z:G6} prevSelectedWeight={weights.w:G6}",
            $"Meta: currentDirLen={meta.x:G6} prevDirLen={meta.y:G6} currentLightType={meta.z:G6} prevLightType={meta.w:G6}",
            $"Light Candidates: total={candidate.x:G6} hasSun={candidate.y:0} pointLights={candidate.z:G6} firstIndex={candidate.w:G6}",
            $"RIS Config: enabled={(UseDirectLightReservoirRIS ? 1 : 0)} drawCount={DirectLightRISCandidateCount:G6} neighborEnabled={(UseDirectLightReservoirNeighborReuse ? 1 : 0)} neighborCount={DirectLightNeighborReuseCount:G6}",
            $"First Draw: accepted={firstDraw.x:0} valid={firstDraw.y:0} proposalPdf={firstDraw.z:G6} targetLum={firstDraw.w:G6}",
            $"First Meta: reservoirWeight={firstMeta.x:G6} maxDist={firstMeta.y:G6} lightType={firstMeta.z:G6} sunNdotL={firstMeta.w:G6}",
            $"RIS Local: selected={risLocal.x:0} localSelected={risLocal.y:0} weightSum={risLocal.z:G6} localWeightSum={risLocal.w:G6}",
            $"RIS Final: represented={risFinal.x:G6} localRepresented={risFinal.y:G6} selectedWeight={risFinal.z:G6} payloadValid={risFinal.w:0}",
            $"Material: mode={material.x:G6} roughness={material.y:G6} metallic={material.z:G6} albedoLum={materialAlbedoLum:G6}",
            shadingOrCompatibilityLine,
            $"Raw Material: albedo=({rawMaterial.x:G6}, {rawMaterial.y:G6}, {rawMaterial.z:G6}) finite={rawMaterial.w:0}",
            $"Raw Throughput: value=({rawThroughput.x:G6}, {rawThroughput.y:G6}, {rawThroughput.z:G6}) finite={rawThroughput.w:0}",
            $"Raw Light: rgba=({rawLight.x:G6}, {rawLight.y:G6}, {rawLight.z:G6}, {rawLight.w:G6})",
            $"First Sample Scalars: targetFromContribution={sampleScalars.x:G6} storedTarget={sampleScalars.y:G6} storedReservoirWeight={sampleScalars.z:G6} finite={sampleScalars.w:0}",
            $"First Raw Contribution: value=({rawContribution.x:G6}, {rawContribution.y:G6}, {rawContribution.z:G6}) finite={rawContribution.w:0}",
            $"Final Selected Scalars: targetFromContribution={finalSampleScalars.x:G6} storedTarget={finalSampleScalars.y:G6} storedReservoirWeight={finalSampleScalars.z:G6} finite={finalSampleScalars.w:0}",
            $"Final Raw Contribution: value=({finalRawContribution.x:G6}, {finalRawContribution.y:G6}, {finalRawContribution.z:G6}) finite={finalRawContribution.w:0}",
            $"Neighbor Replay Counts: stored={neighborCounts.x:G6} source={neighborCounts.y:G6} compatible={neighborCounts.z:G6} reevaluated={neighborCounts.w:G6}",
            $"Neighbor Replay State: requested={neighborState.x:0} active={neighborState.y:0} temporalPixel={neighborState.z:0} temporalStage={neighborState.w:G6}({DescribeTemporalReplayStage(neighborState.w)})",
            $"Temporal Replay Index: current={temporalReplayIndex.x:G6} reprojected={temporalReplayIndex.y:G6} matchesCurrent={temporalReplayIndex.z:0} usedSamePixelFallback={temporalReplayIndex.w:0}",
            $"Raw/Derived Check: albedoLumCPU={rawAlbedoLum:G6} delta={Mathf.Abs(rawAlbedoLum - materialAlbedoLum):G6} throughputLumCPU={rawThroughputLum:G6} delta={Mathf.Abs(rawThroughputLum - shadingThroughputLum):G6} dirLightLumCPU={rawLightLum:G6} delta={Mathf.Abs(rawLightLum - shadingDirLightLum):G6}"
        };
    }

    private int GetDirectLightDebugPixelX()
    {
        if (_currentRenderWidth <= 0)
            return 0;

        return Mathf.Clamp(_currentRenderWidth / 2 + DirectLightDiagnosticPixelOffset.x, 0, Mathf.Max(_currentRenderWidth - 1, 0));
    }

    private int GetDirectLightDebugPixelY()
    {
        if (_currentRenderHeight <= 0)
            return 0;

        return Mathf.Clamp(_currentRenderHeight / 2 + DirectLightDiagnosticPixelOffset.y, 0, Mathf.Max(_currentRenderHeight - 1, 0));
    }

    // Keep CPU-side diagnostic reconstruction aligned with the shader luminance convention.
    private static float ComputeDiagnosticLuminance(Vector3 value)
    {
        return Vector3.Dot(Vector3.Max(value, Vector3.zero), LuminanceWeights);
    }

    private static string DescribeTemporalReplayStage(float stage)
    {
        int stageIndex = Mathf.RoundToInt(stage);
        switch (stageIndex)
        {
            case 0: return "none";
            case 1: return "reprojected_stored";
            case 2: return "reprojected_source_valid";
            case 3: return "reprojected_compatible";
            case 4: return "reprojected_accepted";
            case 5: return "fallback_stored";
            case 6: return "fallback_source_valid";
            case 7: return "fallback_compatible";
            case 8: return "fallback_accepted";
            default: return $"stage_{stageIndex}";
        }
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
               _oldUseDirectLightReservoirRIS != UseDirectLightReservoirRIS ||
               _oldUseDirectLightReservoirNeighborReuse != UseDirectLightReservoirNeighborReuse ||
               _oldDirectLightNeighborReuseCount != DirectLightNeighborReuseCount ||
               _oldDirectLightDebugViewMode != DirectLightDebugViewMode;
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
        _oldUseDirectLightReservoirRIS = UseDirectLightReservoirRIS;
        _oldUseDirectLightReservoirNeighborReuse = UseDirectLightReservoirNeighborReuse;
        _oldDirectLightNeighborReuseCount = DirectLightNeighborReuseCount;
        _oldDirectLightDebugViewMode = DirectLightDebugViewMode;
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
        _hasPrimarySurfaceHistory = false;
        _hasDirectLightReservoirHistory = false;
        _hasDirectLightDiagnosticsSnapshot = false;
    }

    void ResetAccumulationOnly()
    {
        sampleCount = 0;
        _hasDirectLightDiagnosticsSnapshot = false;
    }

    private bool IsDirectLightDebugViewActive()
    {
        return DirectLightDebugViewMode != DirectLightDebugView.Off;
    }
}
