using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEditor;
using UnityEngine;
using UnityEngine.Profiling;

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
    
    
    [SerializeField, Range(2, 20)]
    int TraceDepth = 5;
    
    [SerializeField]
    private bool drawGizmos = false;
    [SerializeField]
    private bool DrawTLAS = true;
    [SerializeField] private bool DrawBLAS = true;
    [SerializeField] private bool DrawMeshNode = true;
    [SerializeField] private bool DrawTLASBVH = true;
    
    private int sampleCount = 0;
    
    private Material collectMaterial;
    private RenderTexture frameConverged;
    
    private readonly int dispatchGroupX = 32;
    private readonly int dispatchGroupY = 32;
    private int dispatchGroupXFull, dispatchGroupYFull;
    private Vector2 dispatchOffsetLimit;
    private Vector4 dispatchCount;

    private void Awake()
    {
        if (collectMaterial == null)
            collectMaterial = new Material(Shader.Find("Hidden/Collect"));
    }

    private void Start()
    {
        cam = GetComponent<Camera>();
        LightManager.Instance.UpdateLights();
    }

    private void OnRenderImage(RenderTexture source, RenderTexture destination)
    {
#if UNITY_EDITOR
        if (drawGizmos)
        {
            Graphics.Blit(source, destination);
            return;
        }
#endif
        Render(destination);
    }

    private void Render(RenderTexture destination)
    {
        if (target == null || target.width != Screen.width || target.height != Screen.height)
        {
            if (target != null) target.Release();
            target = new RenderTexture(Screen.width, Screen.height, 0, RenderTextureFormat.ARGBFloat,
                RenderTextureReadWrite.Linear);
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
            frameConverged.enableRandomWrite = true;
            frameConverged.Create();
        }
        
        SetShaderParameters(100);
        sampleCount++;
        dispatchGroupXFull = Mathf.CeilToInt(Screen.width / 8.0f);
        dispatchGroupYFull = Mathf.CeilToInt(Screen.height / 8.0f);
        tracingShader.SetTexture(0, "_Result", target);
        tracingShader.Dispatch(0, dispatchGroupXFull, dispatchGroupYFull, 1);
        // Graphics.Blit(target, frameConverged, collectMaterial);
        // Graphics.Blit(frameConverged, destination);
        Graphics.Blit(target, destination);
    }

    private void Update()
    {
        // LightManager.Instance.UpdateLights();
        BVHBuilder.Validate();
        if(Camera.main.transform.hasChanged)
        {
            ResetSampleCount();
            Camera.main.transform.hasChanged = false;
        }
    }
    
    private uint frameId = 0;

    private void SetShaderParameters(int refreshRate)
    {
        // if(sampleCount % refreshRate == 0 || true)
        // {
        tracingShader.SetInt("_FrameCount", (int)frameId++);
            var DirectionalLight = LightManager.Instance.DirectionalLight;
        
            Vector3 dir = DirectionalLight.transform.forward;
            Vector3 directionalLightInfo = new Vector3(-dir.x, -dir.y, -dir.z);
            Vector3 directionalLightColorInfo = new Vector4(
                DirectionalLight.color.r,
                DirectionalLight.color.g,
                DirectionalLight.color.b,
                DirectionalLight.intensity
            );
            
            var pointLightsBuffer = LightManager.Instance.pointLightsBuffer;
            
            tracingShader.SetVector("_PixelOffset", GeneratePixelOffset());
            tracingShader.SetVector("_DirectionalLight", directionalLightInfo);
            tracingShader.SetVector("_DirectionalLightColor", directionalLightColorInfo);
            // tracingShader.SetFloat("_Seed", UnityEngine.Random.value);
            tracingShader.SetVector("_Resolution", new Vector2(Screen.width, Screen.height));
            tracingShader.SetMatrix("_CameraToWorld", cam.cameraToWorldMatrix);
            tracingShader.SetMatrix("_CameraInverseProjection", cam.projectionMatrix.inverse);
            tracingShader.SetTexture(0, "_SkyboxTexture", skyboxTexture);
            tracingShader.SetInt("_PointLightsCount", LightManager.Instance.GetPointLightsCount());
            tracingShader.SetBuffer(0,"_PointLights",pointLightsBuffer);

		    if (BVHBuilder.VertexBuffer != null) tracingShader.SetBuffer(0, "_Vertices", BVHBuilder.VertexBuffer);
            if (BVHBuilder.IndexBuffer != null) tracingShader.SetBuffer(0, "_Indices", BVHBuilder.IndexBuffer);
            if (BVHBuilder.NormalBuffer != null) tracingShader.SetBuffer(0, "_Normals", BVHBuilder.NormalBuffer);
            if (BVHBuilder.TangentBuffer != null) tracingShader.SetBuffer(0, "_Tangents", BVHBuilder.TangentBuffer);
            if (BVHBuilder.UVBuffer != null) tracingShader.SetBuffer(0, "_UVs", BVHBuilder.UVBuffer);
            if (BVHBuilder.MaterialBuffer != null) tracingShader.SetBuffer(0, "_Materials", BVHBuilder.MaterialBuffer);
            if (BVHBuilder.MeshNodeBuffer != null)
            {
                tracingShader.SetInt("_TLASNodesCount", BVHBuilder.GetTLASNodes().Count);
                tracingShader.SetBuffer(0, "_TLASNodes", BVHBuilder.MeshNodeBuffer);
            }

            if (BVHBuilder.BLASBuffer != null)
            {
                tracingShader.SetBuffer(0, "_BNodes", BVHBuilder.BLASBuffer);
                tracingShader.SetInt("_BNodesCount", BVHBuilder.GetBLASNodes().Count);
            }
            if (BVHBuilder.TransformBuffer != null) tracingShader.SetBuffer(0, "_Transforms", BVHBuilder.TransformBuffer);
            if (BVHBuilder.AlbedoTextures != null) tracingShader.SetTexture(0, "_AlbedoTextures", BVHBuilder.AlbedoTextures);
            if (BVHBuilder.EmissionTextures != null) tracingShader.SetTexture(0, "_EmitTextures", BVHBuilder.EmissionTextures);
            if (BVHBuilder.MetallicTextures != null) tracingShader.SetTexture(0, "_MetallicTextures", BVHBuilder.MetallicTextures);
            if (BVHBuilder.NormalTextures != null) tracingShader.SetTexture(0, "_NormalTextures", BVHBuilder.NormalTextures);
            if (BVHBuilder.RoughnessTextures != null) tracingShader.SetTexture(0, "_RoughnessTextures", BVHBuilder.RoughnessTextures);
        // }
    }
    
    private Vector2 GeneratePixelOffset()
    {
        Vector2 offset = new Vector2(UnityEngine.Random.value, UnityEngine.Random.value);
        offset.x += dispatchCount.x * dispatchGroupX * 8;
        offset.y += dispatchCount.y * dispatchGroupY * 8;
        return offset;
    }
    
#if UNITY_EDITOR
    [MenuItem("Tools/Debug/Force GC & UnloadAssets")]
    static void ForceGC()
    {
        System.GC.Collect();
        Resources.UnloadUnusedAssets();
        Debug.Log($"After GC - Total Alloc: {Profiler.GetTotalAllocatedMemoryLong()/1048576f:F1} MB");
    }
#endif
    
    private void OnDisable()
    {
        if (target != null)
        {
            target.Release();
        }
        BVHBuilder.Destroy();
    }
    
    private void OnApplicationQuit()
    {
        BVHBuilder.Destroy();
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

            Color colLeaf  = new(0, 1,   0);         // 叶子 = 绿
            Color colInner = new(0, 0.4f, 1);        // 内部 = 蓝

            while (q.Count > 0)
            {
                BVH.BVHNode n = q.Dequeue();

                Vector3 min  = n.Bounds.min;
                Vector3 max  = n.Bounds.max;
                Gizmos.color = n.IsLeaf() ? colLeaf : colInner;
                if (n.IsLeaf())
                {
                    for (int i = n.OriginTriOrMeshStartIndex; i < n.OriginTriOrMeshEndIndex; ++i)
                    {
                        var mesh = meshNodes[orderedInfos[i]];
                        var transform = transforms[mesh.TransformIdx * 2]; // local to world
                        Vector3 LocalCenter  = (mesh.BoundMin + mesh.BoundMax) * 0.5f;
                        Vector3 LocalSize    = (mesh.BoundMax - mesh.BoundMin);
                
                        var WorldCenter = transform.MultiplyPoint3x4(LocalCenter);
                        var WorldSize = transform.MultiplyVector(LocalSize);
                
                        WorldSize.x = Mathf.Abs(WorldSize.x);
                        WorldSize.y = Mathf.Abs(WorldSize.y);
                        WorldSize.z = Mathf.Abs(WorldSize.z);
                        
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
                    // if (n.RightChild != null) 
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
                Vector3 LocalCenter  = (n.BoundMin + n.BoundMax) * 0.5f;
                Vector3 LocalSize    = (n.BoundMax - n.BoundMin);
                
                var WorldCenter = l2w.MultiplyPoint3x4(LocalCenter);
                var WorldSize = l2w.MultiplyVector(LocalSize);
                
                WorldSize.x = Mathf.Abs(WorldSize.x);
                WorldSize.y = Mathf.Abs(WorldSize.y);
                WorldSize.z = Mathf.Abs(WorldSize.z);
            
                Gizmos.color = Color.yellow;
                Gizmos.DrawWireCube(
                    WorldCenter,
                    WorldSize);
            }
            
            
        }
        
        if (tlasNodes == null || tlasNodes.Count == 0) return;

        if (DrawTLAS)
        {
            // var sharedInfos = BVHBuilder.tlasTree.GetSharedPrimitiveInfo();
            // for (int i = 0; i < sharedInfos.Count; ++i)
            // {
            //     var info = sharedInfos[i];
            //     Gizmos.color = Color.green;
            //     var center = info.Center;
            //     var bounds = info.Bounds;
            //     Gizmos.DrawWireCube(center, bounds.extent);
            // }
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
                
                Gizmos.color = (n.ChildIdx < 0) // 叶子 / 父节点不同颜色
                    ? new Color(1.0f, 0.4f, 0.6f)
                    : new Color(0.0f, 1.0f, 0.0f);
                if(n.ChildIdx < 0)
                {
                    // size 变为正数
                    size.x = Mathf.Abs(size.x);
                    size.y = Mathf.Abs(size.y);
                    size.z = Mathf.Abs(size.z);
                    Gizmos.DrawWireCube(center, size);
                }
            
                if (n.ChildIdx >= 0)
                {
                    stackTLAS[++sp] = n.ChildIdx + 1; // 右
                    stackTLAS[++sp] = n.ChildIdx; // 左
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
                int[] stack = new int[32];
                stack[stackPtr] = meshNode.NodeRootIdx;
                
                while (stackPtr >= 0 && stackPtr < 32)
                {
                    var idx = stack[stackPtr--];
                    var bnode = bnodes[idx];
                    var min = localToWorld.MultiplyPoint3x4(bnode.BoundMin);
                    var max = localToWorld.MultiplyPoint3x4(bnode.BoundMax);
                    var center = (min + max) / 2;
                    var s = max - min;
                    Color color = Color.red;
                    color.a = 0.5f;
                    Gizmos.color = color;
                    Gizmos.DrawWireCube(center, s);
                    
                    if(bnode.PrimitiveStartIdx < 0)
                    {
                        stack[++stackPtr] = bnode.ChildIdx;
                        
                        stack[++stackPtr] = bnode.ChildIdx + 1;
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