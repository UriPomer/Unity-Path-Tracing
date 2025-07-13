using System;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(LightManager))]
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
    
    [Header("Debug")]
    [SerializeField] bool OnlyDrawAlbedo = false;
    [SerializeField] bool OnlyDrawNormals = false;
    [SerializeField] bool OnlyDrawDepth = false;
    [SerializeField] bool Denoise = true;
    private bool _OldDenoise = true;
    
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
    
    private RenderTexture frameConverged;
    
    private readonly int dispatchGroupX = 32;
    private readonly int dispatchGroupY = 32;
    private int dispatchGroupXFull, dispatchGroupYFull;
    private Vector2 dispatchOffsetLimit;
    private Vector4 dispatchCount;

    private void Awake()
    {
        if (_addMaterial == null)
            _addMaterial = new Material(Shader.Find("Hidden/AddShader"));
    }

    private void Start()
    {
        cam = GetComponent<Camera>();
        LightManager.Instance.UpdateLights();
        _OldDenoise = Denoise;
    }

    private void OnRenderImage(RenderTexture source, RenderTexture destination)
    {
        Render(destination);
    }
    
    private bool NeedUpdate = false;
    
    private void Render(RenderTexture destination)
    {
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
        
        if(BVHBuilder.Validate() || (Camera.main != null && Camera.main.transform.hasChanged))
        {
            ResetSampleCount();
            Camera.main.transform.hasChanged = false;
            NeedUpdate = true;
        }
        
        SetShaderParameters();
        sampleCount++;
        dispatchGroupXFull = Mathf.CeilToInt(Screen.width / 8.0f);
        dispatchGroupYFull = Mathf.CeilToInt(Screen.height / 8.0f);
        tracingShader.SetTexture(0, "_Result", target);
        tracingShader.Dispatch(0, dispatchGroupXFull, dispatchGroupYFull, 1);
        
        if (Denoise)
        {
            _addMaterial.SetFloat("_Sample", sampleCount);
            Graphics.Blit(target, frameConverged, _addMaterial);
            Graphics.Blit(frameConverged, destination);
        }
        else
        {
            Graphics.Blit(target, destination);
        }
    }

    private void Update()
    {
        if (_OldDenoise != Denoise)
        {
            _OldDenoise = Denoise;
            ResetSampleCount();
        }
        
        LightManager.Instance.UpdateLights();
        LightManager.Instance.UpdateBuffer(tracingShader);
    }
    
    private uint frameId = 0;

    private void SetShaderParameters()
    {
        tracingShader.SetInt("_FrameCount", (int)frameId++);
        
        tracingShader.SetVector("_PixelOffset", GeneratePixelOffset());
        // tracingShader.SetFloat("_Seed", UnityEngine.Random.value);
        tracingShader.SetVector("_Resolution", new Vector2(Screen.width, Screen.height));
        tracingShader.SetInt("_TraceDepth", TraceDepth);
        tracingShader.SetMatrix("_CameraToWorld", cam.cameraToWorldMatrix);
        tracingShader.SetMatrix("_CameraInverseProjection", cam.projectionMatrix.inverse);
        tracingShader.SetTexture(0, "_SkyboxTexture", skyboxTexture);
        tracingShader.SetFloat("_SunFocus", SunFocus);
        tracingShader.SetFloat("_SunAngularRadius", SunAngularRadius);

        if (NeedUpdate)
        {
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
        }
        
        tracingShader.SetBool("_OnlyDrawDepth", OnlyDrawDepth);
        tracingShader.SetBool("_OnlyDrawNormals", OnlyDrawNormals);
        tracingShader.SetBool("_OnlyDrawAlbedo", OnlyDrawAlbedo);
        tracingShader.SetFloat("_CameraFar", cam.farClipPlane);
    }
    
    private Vector2 GeneratePixelOffset()
    {
        Vector2 offset = new Vector2(UnityEngine.Random.value, UnityEngine.Random.value);
        offset.x += dispatchCount.x * dispatchGroupX * 8;
        offset.y += dispatchCount.y * dispatchGroupY * 8;
        return offset;
    }
    
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
                Gizmos.color = n.IsLeaf() ? colLeaf : colInner;
                if (n.IsLeaf())
                {
                    for (int i = n.OriginTriOrMeshStartIndex; i < n.OriginTriOrMeshEndIndex; ++i)
                    {
                        var mesh = meshNodes[orderedInfos[i]];
                        var transform_ = transforms[mesh.TransformIdx * 2]; // local to world
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
                
                Gizmos.color = (n.TransformIdx >= 0) // 叶子 / 父节点不同颜色
                    ? new Color(1.0f, 0.4f, 0.6f)
                    : new Color(0.0f, 1.0f, 0.0f);
                if(n.TransformIdx >= 0)
                {
                    Gizmos.DrawWireCube(center, size);
                }
            
                if (n.TransformIdx < 0)
                {
                    stackTLAS[++sp] = n.Index + 1; // 右
                    stackTLAS[++sp] = n.Index; // 左
                    
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
                int[] stack = new int[32];
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
                    
                    // TransformUtils.TransformBounds(localToWorld, bnode.BoundMin, bnode.BoundMax, out var worldMin, out var worldMax);
                    //
                    // color = Color.blue;
                    // color.a = 0.5f;
                    // Gizmos.color = color;
                    //
                    // Gizmos.DrawWireCube((worldMin + worldMax) / 2, (worldMax - worldMin));
                    
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