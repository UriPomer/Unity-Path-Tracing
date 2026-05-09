using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;
using UnityEngine.Rendering;

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

    [Header("Debug")]
    [SerializeField] bool OnlyDrawAlbedo = false;
    [SerializeField] bool OnlyDrawNormals = false;
    [SerializeField] bool OnlyDrawDepth = false;
    [SerializeField, Range(1, 16)] int DirectLightRISCandidateCount = 1;
    [SerializeField] bool UseDirectLightReservoirForPrimaryDirect = false;
    [SerializeField] bool ShowDirectLightReservoirDifference = false;
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
    private int kernelTransfer;

    // Multi-pass buffers
    private ComputeBuffer _globalRaysA;
    private ComputeBuffer _globalRaysB;
    private ComputeBuffer _globalHits;
    private ComputeBuffer _shadowRays;
    private ComputeBuffer _directLightReservoirs;
    private ComputeBuffer _directLightReservoirDifference;
    private ComputeBuffer _globalColors;
    private ComputeBuffer _bufferSizes;
    private ComputeBuffer _indirectArgs;

    // Struct sizes (must match HLSL layout)
    private const int RayDataStride = 48;         // float3+float3+uint+uint+float3+float = 12+12+4+4+12+4
    private const int HitDataStride = 76;          // 4×(float3+scalar) + 2×float + float = 4×16+8+4
    private const int ShadowRayDataStride = 48;    // 3×(float3+scalar)
    private const int DirectLightReservoirStride = 64; // 4 packed float4-sized rows
    private const int DirectLightReservoirDifferenceStride = 12; // float3
    private const int PathContributionStride = 24; // float3+float3 = 12+12
    private const int BufferSizeDataStride = 8;    // int+int = 4+4
    private const int IndirectArgsStride = 4;      // uint x3 = 3 elements x 4 bytes each

    private int prevWidth, prevHeight;

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
    private int _oldDirectLightRISCandidateCount = 1;
    private bool _oldUseDirectLightReservoirForPrimaryDirect = false;
    private bool _oldShowDirectLightReservoirDifference = false;

    private void Awake()
    {
        EnsureMaterials();
    }

    private void Start()
    {
        cam = GetComponent<Camera>();
        lightCullingManager = GetComponent<LightCullingManager>();
        LightManager.Instance.UpdateLights();
        _OldDenoise = Denoise;
        QualitySettings.vSyncCount = 0;
        Application.targetFrameRate = targetFrameRate;

        // Find kernel indices
        kernelGenerate = tracingShader.FindKernel("kernel_generate");
        kernelTrace = tracingShader.FindKernel("kernel_trace");
        kernelShade = tracingShader.FindKernel("kernel_shade");
        kernelShadow = tracingShader.FindKernel("kernel_shadow");
        kernelFinalize = tracingShader.FindKernel("kernel_finalize");
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
        Render(destination);
    }

    private bool NeedUpdate = true;

    private void CreateBuffersIfNeeded()
    {
        int w = Screen.width;
        int h = Screen.height;
        if (w == prevWidth && h == prevHeight) return;

        ReleaseBuffers();

        int pixelCount = w * h;

        _globalRaysA = new ComputeBuffer(pixelCount, RayDataStride);
        _globalRaysB = new ComputeBuffer(pixelCount, RayDataStride);
        _globalHits = new ComputeBuffer(pixelCount, HitDataStride);
        // Keep headroom for future direct-light candidates/reservoir experiments.
        _shadowRays = new ComputeBuffer(pixelCount * 2, ShadowRayDataStride);
        _directLightReservoirs = new ComputeBuffer(pixelCount, DirectLightReservoirStride);
        _directLightReservoirDifference = new ComputeBuffer(pixelCount, DirectLightReservoirDifferenceStride);
        _globalColors = new ComputeBuffer(pixelCount, PathContributionStride);
        _bufferSizes = new ComputeBuffer(TraceDepth + 1, BufferSizeDataStride);
        _indirectArgs = new ComputeBuffer(3, IndirectArgsStride, ComputeBufferType.IndirectArguments);

        prevWidth = w;
        prevHeight = h;
    }

    private void ReleaseBuffers()
    {
        _globalRaysA?.Release(); _globalRaysA = null;
        _globalRaysB?.Release(); _globalRaysB = null;
        _globalHits?.Release(); _globalHits = null;
        _shadowRays?.Release(); _shadowRays = null;
        _directLightReservoirs?.Release(); _directLightReservoirs = null;
        _directLightReservoirDifference?.Release(); _directLightReservoirDifference = null;
        _globalColors?.Release(); _globalColors = null;
        _bufferSizes?.Release(); _bufferSizes = null;
        _indirectArgs?.Release(); _indirectArgs = null;
    }

    private void Render(RenderTexture destination)
    {
        EnsureMaterials();

        if (target == null || target.width != Screen.width || target.height != Screen.height)
        {
            if (target != null) target.Release();
            target = new RenderTexture(Screen.width, Screen.height, 0, RenderTextureFormat.ARGBFloat,
                RenderTextureReadWrite.Linear);
            target.filterMode = FilterMode.Point;
            target.wrapMode   = TextureWrapMode.Clamp;
            target.enableRandomWrite = true;
            target.Create();
        }
        if (frameConverged == null ||
            frameConverged.width != Screen.width ||
            frameConverged.height != Screen.height)
        {
            if (frameConverged != null)
                frameConverged.Release();
            frameConverged = new RenderTexture(Screen.width, Screen.height, 0, RenderTextureFormat.ARGBFloat, RenderTextureReadWrite.Linear);
            frameConverged.filterMode = FilterMode.Point;
            frameConverged.wrapMode   = TextureWrapMode.Clamp;
            frameConverged.enableRandomWrite = true;
            frameConverged.Create();
        }

        CreateBuffersIfNeeded();

        if(BVHBuilder.Validate() || (Camera.main != null && Camera.main.transform.hasChanged))
        {
            ResetSampleCount();
            Camera.main.transform.hasChanged = false;
            NeedUpdate = true;
        }

        // 执行光源剔除
        if (lightCullingManager != null)
        {
            lightCullingManager.PerformLightCulling();
        }

        SetShaderParameters();
        NeedUpdate = false;
        sampleCount++;

        int pixelCount = Screen.width * Screen.height;

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
                tracingShader.SetBuffer(kernelShade, "GlobalRays", readBuf);
                tracingShader.SetBuffer(kernelShade, "GlobalRays2", writeBuf);
                tracingShader.SetBuffer(kernelShade, "GlobalHits", _globalHits);

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
            Mathf.CeilToInt(Screen.width / 8.0f),
            Mathf.CeilToInt(Screen.height / 8.0f), 1);

        if (Denoise && !ShowDirectLightReservoirDifference)
        {
            _addMaterial.SetFloat("_Sample", sampleCount);
            Graphics.Blit(target, frameConverged, _addMaterial);
            BlitToDisplay(frameConverged, destination);
        }
        else
        {
            BlitToDisplay(target, destination);
        }
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
        QualitySettings.vSyncCount = 0;
        Application.targetFrameRate = targetFrameRate;
    }

    private uint frameId = 0;

    private void SetShaderParameters()
    {
        tracingShader.SetInt("_FrameCount", (int)frameId++);

        // Per-pixel jitter for temporal AA (used by GenRayByID)
        float jx = UnityEngine.Random.value - 0.5f;
        float jy = UnityEngine.Random.value - 0.5f;
        tracingShader.SetVector("_PixelOffset", new Vector2(jx, jy));

        tracingShader.SetVector("_Resolution", new Vector2(Screen.width, Screen.height));
        tracingShader.SetInt("_TraceDepth", TraceDepth);
        tracingShader.SetMatrix("_CameraToWorld", cam.cameraToWorldMatrix);
        tracingShader.SetMatrix("_CameraInverseProjection", cam.projectionMatrix.inverse);
        tracingShader.SetFloat("_SunFocus", SunFocus);
        tracingShader.SetFloat("_SunAngularRadius", SunAngularRadius);
        tracingShader.SetFloat("_SkyboxIntensity", SkyboxIntensity);

        // Screen dimensions for multi-pass
        tracingShader.SetInt("_ScreenWidth", Screen.width);
        tracingShader.SetInt("_ScreenHeight", Screen.height);

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
        tracingShader.SetBuffer(kernelGenerate, "DirectLightReservoirs", _directLightReservoirs);
        tracingShader.SetBuffer(kernelGenerate, "DirectLightReservoirDifference", _directLightReservoirDifference);
        tracingShader.SetBuffer(kernelTrace, "BufferSizes", _bufferSizes);
        tracingShader.SetBuffer(kernelShade, "GlobalColors", _globalColors);
        tracingShader.SetBuffer(kernelShade, "ShadowRaysBuffer", _shadowRays);
        tracingShader.SetBuffer(kernelShade, "DirectLightReservoirs", _directLightReservoirs);
        tracingShader.SetBuffer(kernelShade, "DirectLightReservoirDifference", _directLightReservoirDifference);
        tracingShader.SetBuffer(kernelShade, "BufferSizes", _bufferSizes);
        tracingShader.SetBuffer(kernelShadow, "ShadowRaysBuffer", _shadowRays);
        tracingShader.SetBuffer(kernelShadow, "GlobalColors", _globalColors);
        tracingShader.SetBuffer(kernelShadow, "DirectLightReservoirs", _directLightReservoirs);
        tracingShader.SetBuffer(kernelShadow, "DirectLightReservoirDifference", _directLightReservoirDifference);
        tracingShader.SetBuffer(kernelShadow, "BufferSizes", _bufferSizes);
        tracingShader.SetBuffer(kernelTransfer, "BufferSizes", _bufferSizes);
        tracingShader.SetBuffer(kernelTransfer, "IndirectArgs", _indirectArgs);
        tracingShader.SetBuffer(kernelFinalize, "GlobalColors", _globalColors);
        tracingShader.SetBuffer(kernelFinalize, "DirectLightReservoirDifference", _directLightReservoirDifference);

        // Set BVH and geometry buffers on all kernels that need them
        if (NeedUpdate)
        {
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
        }

        tracingShader.SetBool("_OnlyDrawDepth", OnlyDrawDepth);
        tracingShader.SetBool("_OnlyDrawNormals", OnlyDrawNormals);
        tracingShader.SetBool("_OnlyDrawAlbedo", OnlyDrawAlbedo);
        tracingShader.SetInt("_DirectLightRISCandidateCount", DirectLightRISCandidateCount);
        tracingShader.SetBool("_UseDirectLightReservoirForPrimaryDirect", UseDirectLightReservoirForPrimaryDirect);
        tracingShader.SetBool("_ShowDirectLightReservoirDifference", ShowDirectLightReservoirDifference);
        tracingShader.SetFloat("_CameraFar", cam.farClipPlane);

        // 设置光源缓冲区（绑定到所有需要的kernel）
        if (lightCullingManager != null)
        {
            lightCullingManager.SetTracingShaderBuffers(tracingShader, lightKernels);
        }

        // Bind current light data immediately before dispatch so rendering does not depend on Update() timing.
        LightManager.Instance.UpdateBuffer(tracingShader, lightKernels);
    }

    private void OnDisable()
    {
        ReleaseRenderTargets();
        ReleaseBuffers();
        ReleaseCommandBuffer();
        ReleaseMaterials();
        BVHBuilder.Destroy();
    }

    private void OnApplicationQuit()
    {
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
               _oldDirectLightRISCandidateCount != DirectLightRISCandidateCount ||
               _oldUseDirectLightReservoirForPrimaryDirect != UseDirectLightReservoirForPrimaryDirect ||
               _oldShowDirectLightReservoirDifference != ShowDirectLightReservoirDifference;
    }

    private void CacheRuntimeSettings()
    {
        _oldToneMap = ToneMap;
        _oldExposure = Exposure;
        _oldSkyboxIntensity = SkyboxIntensity;
        _oldSunFocus = SunFocus;
        _oldSunAngularRadius = SunAngularRadius;
        _oldDirectLightRISCandidateCount = DirectLightRISCandidateCount;
        _oldUseDirectLightReservoirForPrimaryDirect = UseDirectLightReservoirForPrimaryDirect;
        _oldShowDirectLightReservoirDifference = ShowDirectLightReservoirDifference;
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
    }
}
