using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;

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
    private bool drawGizmos = true;
    
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
        LightManager.Instance.UpdateLights();
        BVHBuilder.Validate();
        if(Camera.main.transform.hasChanged)
        {
            ResetSampleCount();
            Camera.main.transform.hasChanged = false;
        }
    }

    private void SetShaderParameters(int refreshRate)
    {
        // if(sampleCount % refreshRate == 0 || true)
        // {
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
            tracingShader.SetFloat("_Seed", UnityEngine.Random.value);
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
            if (BVHBuilder.TLASBuffer != null) tracingShader.SetBuffer(0, "_TNodes", BVHBuilder.TLASBuffer);
            if (BVHBuilder.MeshNodeBuffer != null) tracingShader.SetBuffer(0, "_MeshNodes", BVHBuilder.MeshNodeBuffer);
            if (BVHBuilder.BLASBuffer != null) tracingShader.SetBuffer(0, "_BNodes", BVHBuilder.BLASBuffer);
            if (BVHBuilder.TransformBuffer != null) tracingShader.SetBuffer(0, "_Transforms", BVHBuilder.TransformBuffer);
            if (BVHBuilder.AlbedoTextures != null) tracingShader.SetTexture(0, "_AlbedoTextures", BVHBuilder.AlbedoTextures);
            if (BVHBuilder.EmissionTextures != null) tracingShader.SetTexture(0, "_EmitTextures", BVHBuilder.EmissionTextures);
            if (BVHBuilder.MetallicTextures != null) tracingShader.SetTexture(0, "_MetallicTextures", BVHBuilder.MetallicTextures);
            if (BVHBuilder.NormalTextures != null) tracingShader.SetTexture(0, "_NormalTextures", BVHBuilder.NormalTextures);
            if (BVHBuilder.RoughnessTextures != null) tracingShader.SetTexture(0, "_RoughnessTextures", BVHBuilder.RoughnessTextures);
        // }
    }
    
    private void EstimateGroups(int width, int height)
    {
        // target dispatch 32x32 groups
        // each group has 8x8 threads
        //int pixels = width * height;
        dispatchGroupXFull = Mathf.CeilToInt(Screen.width / 8.0f);
        dispatchGroupYFull = Mathf.CeilToInt(Screen.height / 8.0f);
        dispatchOffsetLimit = new Vector2(
            width - dispatchGroupX * 8,
            height - dispatchGroupY * 8
        );
        dispatchOffsetLimit = Vector2.Max(dispatchOffsetLimit, Vector2.zero);
        dispatchCount = new Vector4(
            0.0f, 0.0f,
            Mathf.Ceil(width / (float)(dispatchGroupX * 8)),
            Mathf.Ceil(height / (float)(dispatchGroupY * 8))
        );
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

    private void OnDrawGizmos()
    {
        if (!drawGizmos)
        {
            return;
        }
        var bnodes = BVHBuilder.GetBLASNodes();
        var meshNodes = BVHBuilder.GetMeshNodes();
        var transforms = BVHBuilder.GetTransforms();
        if (bnodes != null && meshNodes != null && transforms != null)
        {
            for (int i = 0; i < meshNodes.Count; i++)
            {
                var meshNode = meshNodes[i];
                var localToWorld = transforms[meshNode.TransformIdx * 2];
    
                // draw mesh bounds
                Gizmos.color = Color.green;
                var boundCenter = (meshNode.BoundMin + meshNode.BoundMax) / 2;
                var size = meshNode.BoundMax - meshNode.BoundMin;
                Gizmos.DrawWireCube(localToWorld.MultiplyPoint3x4(boundCenter), localToWorld.MultiplyVector(size));
                
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
                    Gizmos.color = Color.red;
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