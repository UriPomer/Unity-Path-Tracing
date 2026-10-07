using UnityEngine;
using UnityEngine.Rendering;

// One renderer owns these resources; partial files group its pipeline responsibilities.
public partial class Tracing
{
    // Multi-pass kernel indices
    private int kernelGenerate;
    private int kernelTrace;
    private int kernelShade;
    private int kernelShadow;
    private int kernelFinalize;
    private int kernelTransfer;
    private int kernelGenerateInitial;
    private int kernelTemporalResampling;
    private int kernelShadeDISamples;
    private int kernelGenerateGISecondarySurfaces;
    private int kernelShadeGISecondarySurfaces;
    private int kernelTemporalGIResampling;
    private int kernelSpatialGIResampling;
    private int kernelClearReSTIRTelemetry;
    private int kernelCaptureReSTIRFrame;

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
    private int _lastDirectReservoirOutputIdx = 0;
    private int _lastIndirectReservoirOutputIdx = 0;
    private bool _hasDirectRestirHistory = false;
    private bool _hasIndirectRestirHistory = false;
    private ComputeBuffer _globalColors;
    private ComputeBuffer _bufferSizes;
    private ComputeBuffer _indirectArgs;
    private RenderTexture _denoisePlaceholder;
    private int[] denoiseKernels;

    // Struct sizes (must match HLSL layout)
    private const int RayDataStride = 44;         // float3+float3+uint+uint+float3
    private const int HitDataStride = 92;          // geometric/shading normals and material identity
    private const int ShadowRayDataStride = 48;    // 3×(float3+scalar)
    private const int DirectLightReservoirStride = 80; // matches DirectLightReservoirData (5 float4 rows)
    private const int IndirectReservoirStride = 88; // one finalized weight; distinct geometric/shading normals
    private const int SecondarySurfaceStride = 124; // 7 float4 rows plus geometric normal
    private const int RestirDebugDataStride = 16; // float4
    private const int RestirDebugDataCount = 5;
    private const int ReSTIRTelemetrySampleStride = 64;
    private const int PathContributionStride = 16; // radiance float3 and explicit HLSL padding
    private const int BufferSizeDataStride = 8;    // int+int = 4+4
    private const int IndirectArgsStride = 4;      // uint x3 = 3 elements x 4 bytes each

    private void Start()
    {
        cam = GetComponent<Camera>();
        _lights.UpdateLights();

        // Find kernel indices
        kernelGenerate = tracingShader.FindKernel("kernel_generate");
        kernelTrace = tracingShader.FindKernel("kernel_trace");
        kernelShade = tracingShader.FindKernel("kernel_shade");
        kernelShadow = tracingShader.FindKernel("kernel_shadow");
        kernelFinalize = tracingShader.FindKernel("kernel_finalize");
        kernelTransfer = tracingShader.FindKernel("TransferKernel");
        kernelGenerateInitial = tracingShader.FindKernel("kernel_generate_initial");
        kernelTemporalResampling = tracingShader.FindKernel("kernel_temporal_resampling");
        kernelShadeDISamples = tracingShader.FindKernel("kernel_shade_di_samples");
        kernelGenerateGISecondarySurfaces = tracingShader.FindKernel("kernel_generate_gi_secondary_surfaces");
        kernelShadeGISecondarySurfaces = tracingShader.FindKernel("kernel_shade_gi_secondary_surfaces");
        kernelTemporalGIResampling = tracingShader.FindKernel("kernel_temporal_gi_resampling");
        kernelSpatialGIResampling = tracingShader.FindKernel("kernel_spatial_gi_resampling");
        kernelClearReSTIRTelemetry = tracingShader.FindKernel("kernel_clear_restir_telemetry");
        kernelCaptureReSTIRFrame = tracingShader.FindKernel("kernel_capture_restir_frame");

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
            kernelSpatialGIResampling
        };
        denoiseKernels = new[] { kernelGenerate, kernelShade, kernelShadow, kernelFinalize, kernelGenerateInitial, kernelShadeDISamples, kernelSpatialGIResampling };
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
            kernelCaptureReSTIRFrame
        };
        cmdBuffer = new CommandBuffer();
        CacheRuntimeSettings();
        _lastLightStateHash = _lights.ComputeLightStateHash();
        for (int i = 0; i < bounceNames.Length; i++)
        {
            int bounce = i / 3;
            int phase = i % 3;
            bounceNames[i] = "PT_B" + bounce + "_" + (phase == 0 ? "Trace" : phase == 1 ? "Shade" : "Shadow");
        }
        _runtimeStarted = true;
        StartReSTIRDiagnostics();
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
        // One shadow candidate per live path and bounce.
        _shadowRays = new ComputeBuffer(pixelCount, ShadowRayDataStride);
        _restirDebugData = new ComputeBuffer(RestirDebugDataCount, RestirDebugDataStride);
        _globalColors = new ComputeBuffer(pixelCount, PathContributionStride);
        _bufferSizes = new ComputeBuffer(TraceDepth + 1, BufferSizeDataStride);
        _indirectArgs = new ComputeBuffer(3, IndirectArgsStride, ComputeBufferType.IndirectArguments);

        _hasDirectRestirHistory = false;
        _hasIndirectRestirHistory = false;
        _lastDirectReservoirOutputIdx = 0;
        _lastIndirectReservoirOutputIdx = 0;

        prevWidth = width;
        prevHeight = height;
        prevTraceDepth = TraceDepth;
    }

    private void CreateReSTIRBuffersIfNeeded(int pixelCount)
    {
        int historyCount = UseReSTIRDI || IsReSTIRGIActive || TemporalDenoisingActive ? pixelCount : 1;
        EnsureBuffer(ref _primarySurfaceHistory, historyCount, HitDataStride);
        EnsureBuffer(ref _primarySurfaceHistoryPrev, historyCount, HitDataStride);
        EnsureBuffer(ref _directLightReservoirs, UseReSTIRDI ? pixelCount * 3 : 1, DirectLightReservoirStride);
        EnsureBuffer(ref _indirectReservoirs, IsReSTIRGIActive ? pixelCount * 3 : 1, IndirectReservoirStride);
        EnsureBuffer(ref _secondarySurfaces, IsReSTIRGIActive ? pixelCount : 1, SecondarySurfaceStride);
    }

    private static void EnsureBuffer(ref ComputeBuffer buffer, int count, int stride)
    {
        if (buffer != null && buffer.count == count) return;
        buffer?.Release();
        buffer = new ComputeBuffer(count, stride);
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
        _hasDirectRestirHistory = false;
        _hasIndirectRestirHistory = false;
        _lastDirectReservoirOutputIdx = 0;
        _lastIndirectReservoirOutputIdx = 0;
        prevWidth = 0;
        prevHeight = 0;
        prevTraceDepth = 0;
    }

    private void SetShaderParameters()
    {
        tracingShader.SetInt("_FrameCount", (int)++frameId);

        tracingShader.SetInt("_TraceDepth", TraceDepth);
        tracingShader.SetMatrix("_CameraToWorld", cam.cameraToWorldMatrix);
        Matrix4x4 gpuProjection = GL.GetGPUProjectionMatrix(cam.projectionMatrix, false);
        tracingShader.SetMatrix("_CameraInverseProjection", cam.projectionMatrix.inverse);
        tracingShader.SetMatrix("_RestirPreviousViewProjection", _previousCameraViewProjection);
        tracingShader.SetVector("_RestirPreviousCameraPosition", _previousRestirCameraPosition);
        tracingShader.SetFloat("_RayPixelSpread", 2f * Mathf.Tan(cam.fieldOfView * Mathf.Deg2Rad * 0.5f) / _currentRenderHeight);
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

        // Set multi-pass buffers on all relevant kernels
        tracingShader.SetBuffer(kernelGenerate, "GlobalRays", _globalRaysA);
        tracingShader.SetBuffer(kernelGenerate, "BufferSizes", _bufferSizes);
        tracingShader.SetBuffer(kernelGenerate, "GlobalColors", _globalColors);
        tracingShader.SetBuffer(kernelGenerate, "GlobalHits", _globalHits);
        tracingShader.SetBuffer(kernelGenerate, "PrimarySurfaceHistory", _primarySurfaceHistory);
        tracingShader.SetBuffer(kernelGenerate, "PrimarySurfaceHistoryPrev", _primarySurfaceHistoryPrev);
        tracingShader.SetBuffer(kernelGenerate, "DirectLightReservoirs", _directLightReservoirs);
        tracingShader.SetBuffer(kernelGenerate, "IndirectReservoirs", _indirectReservoirs);
        tracingShader.SetBuffer(kernelGenerate, "SecondarySurfaces", _secondarySurfaces);
        tracingShader.SetBuffer(kernelTrace, "BufferSizes", _bufferSizes);
        tracingShader.SetBuffer(kernelTrace, "SecondarySurfaces", _secondarySurfaces);
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
        tracingShader.SetBuffer(kernelShadow, "PrimarySurfaceHistory", _primarySurfaceHistory);
        tracingShader.SetBuffer(kernelTransfer, "BufferSizes", _bufferSizes);
        tracingShader.SetBuffer(kernelTransfer, "IndirectArgs", _indirectArgs);
        tracingShader.SetBuffer(kernelFinalize, "GlobalColors", _globalColors);

        // Rebind scene buffers/textures every frame. Shader recompiles during play can
        // invalidate texture bindings even when the BVH itself did not change.
        tracingShader.SetInt("_TLASNodesCount", BVHBuilder.GetTLASNodes().Count);
        foreach (int kernel in bvhKernels)
            BindSceneBuffersToKernel(kernel);

        tracingShader.SetBool("_OnlyDrawDepth", OnlyDrawDepth);
        tracingShader.SetBool("_OnlyDrawNormals", OnlyDrawNormals);
        tracingShader.SetBool("_OnlyDrawAlbedo", OnlyDrawAlbedo);
        tracingShader.SetBool("_HasPrimarySurfaceHistory", _hasPrimarySurfaceHistory);
        tracingShader.SetBool("_UseReSTIRDI", UseReSTIRDI);
        tracingShader.SetBool("_UseReSTIRGI", IsReSTIRGIActive);
        tracingShader.SetBool("_DenoiseEnabled", TemporalDenoisingActive);
        if (_denoisePlaceholder == null)
        {
            _denoisePlaceholder = new RenderTexture(1, 1, 0, RenderTextureFormat.ARGBFloat) { enableRandomWrite = true };
            _denoisePlaceholder.Create();
        }
        foreach (int kernel in denoiseKernels)
        {
            tracingShader.SetTexture(kernel, "_DenoiseDirect", _denoiser?.DirectInput ?? _denoisePlaceholder);
            tracingShader.SetTexture(kernel, "_DenoiseDiffuse", _denoiser?.DiffuseInput ?? _denoisePlaceholder);
            tracingShader.SetTexture(kernel, "_DenoisePathDiffuse", _denoiser?.PathDiffuseWeights ?? _denoisePlaceholder);
        }
        if (!TemporalDenoisingActive)
        {
            tracingShader.SetTexture(kernelTrace, "_DenoiseMotion", _denoisePlaceholder);
            tracingShader.SetBuffer(kernelTrace, "_DenoisePreviousTransforms", BVHBuilder.TransformBuffer);
        }
        tracingShader.SetFloat("_CameraFar", cam.farClipPlane);

        // Bind current light data immediately before dispatch so rendering does not depend on Update() timing.
        _lights.UpdateBuffer(tracingShader);
        _previousCameraViewProjection = gpuProjection * cam.worldToCameraMatrix;
        _previousRestirCameraPosition = cam.transform.position;
    }

    private void BindSceneBuffersToKernel(int kernel)
    {
        if (BVHBuilder.VertexBuffer != null) tracingShader.SetBuffer(kernel, "_Vertices", BVHBuilder.VertexBuffer);
        if (BVHBuilder.TriangleBuffer != null) tracingShader.SetBuffer(kernel, "_Triangles", BVHBuilder.TriangleBuffer);
        if (BVHBuilder.IndexBuffer != null) tracingShader.SetBuffer(kernel, "_Indices", BVHBuilder.IndexBuffer);
        if (BVHBuilder.NormalBuffer != null) tracingShader.SetBuffer(kernel, "_Normals", BVHBuilder.NormalBuffer);
        if (BVHBuilder.TangentBuffer != null) tracingShader.SetBuffer(kernel, "_Tangents", BVHBuilder.TangentBuffer);
        if (BVHBuilder.UVBuffer != null) tracingShader.SetBuffer(kernel, "_UVs", BVHBuilder.UVBuffer);
        if (BVHBuilder.MaterialBuffer != null) tracingShader.SetBuffer(kernel, "_Materials", BVHBuilder.MaterialBuffer);
        if (BVHBuilder.MeshNodeBuffer != null)
        {
            tracingShader.SetBuffer(kernel, "_TLASNodes", BVHBuilder.MeshNodeBuffer);
        }
        if (BVHBuilder.BLASBuffer != null)
        {
            tracingShader.SetBuffer(kernel, "_BNodes", BVHBuilder.BLASBuffer);
        }
        if (BVHBuilder.TransformBuffer != null) tracingShader.SetBuffer(kernel, "_Transforms", BVHBuilder.TransformBuffer);
        tracingShader.SetBuffer(kernel, "_PointLights", _lights.pointLightsBuffer);
        if (BVHBuilder.AlbedoTextures != null) tracingShader.SetTexture(kernel, "_AlbedoTextures", BVHBuilder.AlbedoTextures);
        if (BVHBuilder.EmissionTextures != null) tracingShader.SetTexture(kernel, "_EmitTextures", BVHBuilder.EmissionTextures);
        if (BVHBuilder.MetallicTextures != null) tracingShader.SetTexture(kernel, "_MetallicTextures", BVHBuilder.MetallicTextures);
        if (BVHBuilder.NormalTextures != null) tracingShader.SetTexture(kernel, "_NormalTextures", BVHBuilder.NormalTextures);
        if (BVHBuilder.RoughnessTextures != null) tracingShader.SetTexture(kernel, "_RoughnessTextures", BVHBuilder.RoughnessTextures);
    }

    private void ReleaseRenderTargets()
    {
        ReleaseRenderTexture(ref target);
        ReleaseRenderTexture(ref frameConverged);
        ReleaseRenderTexture(ref _denoisePlaceholder);
        _currentRenderWidth = 0;
        _currentRenderHeight = 0;
    }

    private static void ReleaseRenderTexture(ref RenderTexture texture)
    {
        if (texture == null) return;
        texture.Release();
        Destroy(texture);
        texture = null;
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

    private void ReleaseCommandBuffer()
    {
        if (cmdBuffer == null) return;

        cmdBuffer.Release();
        cmdBuffer = null;
    }

    private void DispatchReSTIRDI(int pixelCount)
    {
        if (!UseReSTIRDI) return;

        int diagnosticPixelIndex = GetReSTIRDiagnosticPixelIndex();
        int initialIdx = (_lastDirectReservoirOutputIdx + 1) % 3;
        int temporalIdx = (_lastDirectReservoirOutputIdx + 2) % 3;
        int prevIdx = _lastDirectReservoirOutputIdx;

        tracingShader.SetBuffer(kernelGenerateInitial, "DirectLightReservoirs", _directLightReservoirs);
        tracingShader.SetBuffer(kernelGenerateInitial, "_RestirGbuffer", _globalHits);
        tracingShader.SetInt("_RestirInitialReservoirOffset", initialIdx * pixelCount);
        tracingShader.SetInt("_RestirCandidateCount", DirectLightRISCandidateCount);
        tracingShader.Dispatch(kernelGenerateInitial, (pixelCount + 63) / 64, 1, 1);

        bool useTemporal = _hasDirectRestirHistory;
        if (useTemporal)
        {
            tracingShader.SetBuffer(kernelTemporalResampling, "DirectLightReservoirs", _directLightReservoirs);
            tracingShader.SetBuffer(kernelTemporalResampling, "ReSTIRDebugData", _restirDebugData);
            tracingShader.SetInt("_RestirInitialReservoirOffset", initialIdx * pixelCount);
            tracingShader.SetInt("_RestirTemporalReservoirOffset", temporalIdx * pixelCount);
            tracingShader.SetInt("_RestirPrevReservoirOffset", prevIdx * pixelCount);
            tracingShader.SetBuffer(kernelTemporalResampling, "_RestirGbuffer", _globalHits);
            tracingShader.SetBuffer(kernelTemporalResampling, "_RestirGbufferPrevious", _primarySurfaceHistoryPrev);
            tracingShader.SetInt("_RestirDebugPixelIndex", diagnosticPixelIndex);
            tracingShader.Dispatch(kernelTemporalResampling, (pixelCount + 63) / 64, 1, 1);
        }

        int shadingReservoirIdx = useTemporal ? temporalIdx : initialIdx;

        tracingShader.SetBuffer(kernelShadeDISamples, "DirectLightReservoirs", _directLightReservoirs);
        tracingShader.SetBuffer(kernelShadeDISamples, "_RestirGbuffer", _globalHits);
        tracingShader.SetBuffer(kernelShadeDISamples, "GlobalColors", _globalColors);
        tracingShader.SetInt("_RestirShadingReservoirOffset", shadingReservoirIdx * pixelCount);
        tracingShader.Dispatch(kernelShadeDISamples, (pixelCount + 63) / 64, 1, 1);

        _lastDirectReservoirOutputIdx = shadingReservoirIdx;
        _hasDirectRestirHistory = true;
    }

    private void DispatchReSTIRGI(int pixelCount)
    {
        int diagnosticPixelIndex = GetReSTIRDiagnosticPixelIndex();
        int initialIdx = (_lastIndirectReservoirOutputIdx + 1) % 3;
        int temporalIdx = (_lastIndirectReservoirOutputIdx + 2) % 3;
        int prevIdx = _lastIndirectReservoirOutputIdx;
        int spatialIdx = prevIdx;

        tracingShader.SetBuffer(kernelGenerateGISecondarySurfaces, "_RestirGbuffer", _globalHits);
        tracingShader.SetBuffer(kernelGenerateGISecondarySurfaces, "SecondarySurfaces", _secondarySurfaces);
        tracingShader.SetBuffer(kernelGenerateGISecondarySurfaces, "SecondarySurfacesRead", _secondarySurfaces);
        tracingShader.Dispatch(kernelGenerateGISecondarySurfaces, (_currentRenderWidth + 7) / 8, (_currentRenderHeight + 7) / 8, 1);

        tracingShader.SetBuffer(kernelShadeGISecondarySurfaces, "IndirectReservoirs", _indirectReservoirs);
        tracingShader.SetBuffer(kernelShadeGISecondarySurfaces, "SecondarySurfaces", _secondarySurfaces);
        tracingShader.SetBuffer(kernelShadeGISecondarySurfaces, "IndirectReservoirsRead", _indirectReservoirs);
        tracingShader.SetBuffer(kernelShadeGISecondarySurfaces, "SecondarySurfacesRead", _secondarySurfaces);
        tracingShader.SetBuffer(kernelShadeGISecondarySurfaces, "GlobalColors", _globalColors);
        tracingShader.SetBuffer(kernelShadeGISecondarySurfaces, "ReSTIRDebugData", _restirDebugData);
        tracingShader.SetBuffer(kernelShadeGISecondarySurfaces, "_RestirGbuffer", _globalHits);
        tracingShader.SetInt("_RestirInitialReservoirOffset", initialIdx * pixelCount);
        tracingShader.SetInt("_RestirDebugPixelIndex", diagnosticPixelIndex);
        tracingShader.Dispatch(kernelShadeGISecondarySurfaces, (_currentRenderWidth + 7) / 8, (_currentRenderHeight + 7) / 8, 1);

        bool useTemporal = _hasIndirectRestirHistory;
        if (useTemporal)
        {
            tracingShader.SetBuffer(kernelTemporalGIResampling, "IndirectReservoirs", _indirectReservoirs);
            tracingShader.SetBuffer(kernelTemporalGIResampling, "IndirectReservoirsRead", _indirectReservoirs);
            tracingShader.SetBuffer(kernelTemporalGIResampling, "SecondarySurfaces", _secondarySurfaces);
            tracingShader.SetBuffer(kernelTemporalGIResampling, "ReSTIRDebugData", _restirDebugData);
            tracingShader.SetBuffer(kernelTemporalGIResampling, "_RestirGbuffer", _globalHits);
            tracingShader.SetBuffer(kernelTemporalGIResampling, "_RestirGbufferPrevious", _primarySurfaceHistoryPrev);
            tracingShader.SetInt("_RestirInitialReservoirOffset", initialIdx * pixelCount);
            tracingShader.SetInt("_RestirTemporalReservoirOffset", temporalIdx * pixelCount);
            tracingShader.SetInt("_RestirPrevReservoirOffset", prevIdx * pixelCount);
            tracingShader.SetInt("_RestirDebugPixelIndex", diagnosticPixelIndex);
            tracingShader.Dispatch(kernelTemporalGIResampling, (_currentRenderWidth + 7) / 8, (_currentRenderHeight + 7) / 8, 1);
        }

        int shadingReservoirIdx = useTemporal ? temporalIdx : initialIdx;

        tracingShader.SetBuffer(kernelSpatialGIResampling, "GlobalColors", _globalColors);
        tracingShader.SetBuffer(kernelSpatialGIResampling, "IndirectReservoirs", _indirectReservoirs);
        tracingShader.SetBuffer(kernelSpatialGIResampling, "SecondarySurfaces", _secondarySurfaces);
        tracingShader.SetBuffer(kernelSpatialGIResampling, "ReSTIRDebugData", _restirDebugData);
        tracingShader.SetBuffer(kernelSpatialGIResampling, "_RestirGbuffer", _globalHits);
        tracingShader.SetInt("_RestirShadingReservoirOffset", shadingReservoirIdx * pixelCount);
        tracingShader.SetInt("_RestirSpatialReservoirOffset", spatialIdx * pixelCount);
        tracingShader.SetInt("_RestirDebugPixelIndex", diagnosticPixelIndex);
        tracingShader.Dispatch(kernelSpatialGIResampling, (_currentRenderWidth + 7) / 8, (_currentRenderHeight + 7) / 8, 1);

        shadingReservoirIdx = spatialIdx;

        _lastIndirectReservoirOutputIdx = shadingReservoirIdx;
        _hasIndirectRestirHistory = true;
    }

    private int GetReSTIRDiagnosticPixelIndex()
    {
        int width = Mathf.Max(_currentRenderWidth, 1);
        int height = Mathf.Max(_currentRenderHeight, 1);
        return (height / 2) * width + width / 2;
    }
}
