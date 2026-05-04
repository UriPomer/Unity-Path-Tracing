using UnityEngine;
using UnityEngine.Rendering;

public class LightCullingManager : MonoBehaviour
{
    [Header("Light Culling Settings")]
    [SerializeField] private ComputeShader lightCullingShader;
    [SerializeField] private int tileSize = 16;
    [SerializeField] private bool enableLightCulling = false;
    
    private ComputeBuffer lightCullingDataBuffer;
    private ComputeBuffer tileDataBuffer;
    private ComputeBuffer dummyLightCullingData;
    private ComputeBuffer dummyTileData;
    private RenderTexture depthTexture;
    
    private int lightCullingKernel;
    private Vector2Int tileCount;
    private Camera targetCamera;
    private Camera depthRenderCamera;
    
    [Header("Debug Info")]
    [SerializeField] private bool showDebugInfo = false;
    private int totalTiles;
    private int maxLightsPerTile = 32;
    
    private void Awake()
    {
        targetCamera = GetComponent<Camera>();
        if (lightCullingShader != null)
        {
            lightCullingKernel = lightCullingShader.FindKernel("CSLightCulling");
        }
    }
    
    private void Start()
    {
        InitializeBuffers();
    }

    private void EnsureDepthRenderCamera()
    {
        if (depthRenderCamera != null) return;

        GameObject cameraObj = new GameObject("LightCulling Depth Camera");
        cameraObj.hideFlags = HideFlags.HideAndDontSave;
        depthRenderCamera = cameraObj.AddComponent<Camera>();
        depthRenderCamera.enabled = false;
    }
    
    private void InitializeBuffers()
    {
        UpdateTileCount();
        
        // 创建光源剔除数据缓冲区
        int totalElements = totalTiles * maxLightsPerTile;
        lightCullingDataBuffer?.Release();
        lightCullingDataBuffer = new ComputeBuffer(totalElements, sizeof(uint));
        
        // 创建tile数据缓冲区 (光源数量和偏移)
        tileDataBuffer?.Release();
        tileDataBuffer = new ComputeBuffer(totalTiles, sizeof(uint) * 2);
        
        // 创建深度纹理
        CreateDepthTexture();
    }
    
    private void UpdateTileCount()
    {
        int screenWidth = Screen.width;
        int screenHeight = Screen.height;
        
        tileCount.x = Mathf.CeilToInt((float)screenWidth / tileSize);
        tileCount.y = Mathf.CeilToInt((float)screenHeight / tileSize);
        totalTiles = tileCount.x * tileCount.y;
    }
    
    private void CreateDepthTexture()
    {
        if (depthTexture != null)
            depthTexture.Release();
            
        depthTexture = new RenderTexture(Screen.width, Screen.height, 24, RenderTextureFormat.Depth);
        depthTexture.Create();
    }
    
    public void PerformLightCulling()
    {
        if (!enableLightCulling || lightCullingShader == null || lightCullingDataBuffer == null)
            return;
            
        // 检查屏幕尺寸是否改变
        if (depthTexture.width != Screen.width || depthTexture.height != Screen.height)
        {
            InitializeBuffers();
        }
        
        // 渲染深度纹理
        RenderDepthTexture();
        
        // 设置ComputeShader参数
        SetComputeShaderParameters();
        
        // 执行光源剔除
        lightCullingShader.Dispatch(lightCullingKernel, tileCount.x, tileCount.y, 1);
    }
    
    private void RenderDepthTexture()
    {
        EnsureDepthRenderCamera();

        depthRenderCamera.CopyFrom(targetCamera);
        depthRenderCamera.transform.SetPositionAndRotation(
            targetCamera.transform.position,
            targetCamera.transform.rotation
        );
        depthRenderCamera.targetTexture = depthTexture;
        depthRenderCamera.enabled = false;

        // 使用深度渲染shader
        Shader depthShader = Shader.Find("Hidden/DepthOnly");
        if (depthShader != null)
        {
            depthRenderCamera.RenderWithShader(depthShader, "RenderType");
        }
        else
        {
            // 回退到内置深度shader
            depthRenderCamera.RenderWithShader(Shader.Find("Hidden/Internal-DepthNormalsTexture"), "RenderType");
        }
        
        depthRenderCamera.targetTexture = null;
    }
    
    private void SetComputeShaderParameters()
    {
        // 设置缓冲区
        lightCullingShader.SetBuffer(lightCullingKernel, "_PointLights", LightManager.Instance.pointLightsBuffer);
        lightCullingShader.SetBuffer(lightCullingKernel, "_LightCullingDataOut", lightCullingDataBuffer);
        lightCullingShader.SetBuffer(lightCullingKernel, "_TileDataOut", tileDataBuffer);
        
        // 设置纹理
        lightCullingShader.SetTexture(lightCullingKernel, "_DepthTexture", depthTexture);
        
        // 设置参数
        lightCullingShader.SetInt("_PointLightsCount", LightManager.Instance.GetPointLightsCount());
        lightCullingShader.SetInts("_TileCount", tileCount.x, tileCount.y);
        lightCullingShader.SetFloat("_CameraNear", targetCamera.nearClipPlane);
        lightCullingShader.SetFloat("_CameraFar", targetCamera.farClipPlane);
        lightCullingShader.SetVector("_Resolution", new Vector2(Screen.width, Screen.height));

        // 设置相机矩阵
        lightCullingShader.SetMatrix("unity_WorldToCamera", targetCamera.worldToCameraMatrix);
        lightCullingShader.SetMatrix("_CameraToWorld", targetCamera.cameraToWorldMatrix);
        lightCullingShader.SetMatrix("_CameraInverseProjection", targetCamera.projectionMatrix.inverse);
    }
    
    public void SetTracingShaderBuffers(ComputeShader tracingShader)
    {
        if (enableLightCulling && lightCullingDataBuffer != null && tileDataBuffer != null)
        {
            tracingShader.SetInts("_TileCount", tileCount.x, tileCount.y);
            tracingShader.SetFloat("_CameraNear", targetCamera.nearClipPlane);
            tracingShader.SetFloat("_CameraFar", targetCamera.farClipPlane);
        }
        else
        {
            // 禁用光源剔除时，设置TileCount为0
            tracingShader.SetInts("_TileCount", 0, 0);
        }
    }

    public void SetTracingShaderBuffers(ComputeShader tracingShader, int[] kernelIndices)
    {
        SetTracingShaderBuffers(tracingShader);
        if (enableLightCulling && lightCullingDataBuffer != null && tileDataBuffer != null)
        {
            foreach (int k in kernelIndices)
            {
                tracingShader.SetBuffer(k, "_LightCullingData", lightCullingDataBuffer);
                tracingShader.SetBuffer(k, "_TileData", tileDataBuffer);
            }
        }
        else
        {
            // 禁用光源剔除时，使用缓存的 dummy buffer 避免 "Property is not set" 错误
            if (dummyLightCullingData == null)
                dummyLightCullingData = new ComputeBuffer(1, sizeof(uint));
            if (dummyTileData == null)
                dummyTileData = new ComputeBuffer(1, sizeof(uint) * 2);
            foreach (int k in kernelIndices)
            {
                tracingShader.SetBuffer(k, "_LightCullingData", dummyLightCullingData);
                tracingShader.SetBuffer(k, "_TileData", dummyTileData);
            }
        }
    }
    
    private void OnDisable()
    {
        ReleaseBuffers();
    }
    
    private void OnDestroy()
    {
        ReleaseBuffers();
    }
    
    private void ReleaseBuffers()
    {
        lightCullingDataBuffer?.Release();
        tileDataBuffer?.Release();
        depthTexture?.Release();
        dummyLightCullingData?.Release();
        dummyTileData?.Release();
        
        lightCullingDataBuffer = null;
        tileDataBuffer = null;
        depthTexture = null;
        dummyLightCullingData = null;
        dummyTileData = null;

        if (depthRenderCamera != null)
        {
            if (Application.isPlaying)
                Destroy(depthRenderCamera.gameObject);
            else
                DestroyImmediate(depthRenderCamera.gameObject);
            depthRenderCamera = null;
        }
    }
    
    private void OnGUI()
    {
        if (!showDebugInfo) return;

        GUILayout.BeginArea(new Rect(10, 10, 350, 250));
        GUILayout.Label($"Light Culling Debug Info:");
        GUILayout.Label($"Tile Size: {tileSize}x{tileSize}");
        GUILayout.Label($"Tile Count: {tileCount.x}x{tileCount.y}");
        GUILayout.Label($"Total Tiles: {totalTiles}");
        GUILayout.Label($"Max Lights Per Tile: {maxLightsPerTile}");
        GUILayout.Label($"Point Lights Count: {LightManager.Instance?.GetPointLightsCount() ?? 0}");
        GUILayout.Label($"Screen Size: {Screen.width}x{Screen.height}");
        GUILayout.Label($"Depth Texture: {(depthTexture != null ? "OK" : "NULL")}");
        GUILayout.Label($"Light Culling Shader: {(lightCullingShader != null ? "OK" : "NULL")}");
        GUILayout.Label($"Buffers: Data={lightCullingDataBuffer != null}, Tile={tileDataBuffer != null}");

        if (GUILayout.Button("Force Disable Light Culling"))
        {
            enabled = false;
        }

        GUILayout.EndArea();
    }
}
