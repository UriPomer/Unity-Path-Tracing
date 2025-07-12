using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LightManager : MonoBehaviour
{
    private static LightManager _instance;
    
    [SerializeField, Range(0.0f, 32.0f)]
    float DirectionalLightIntensityMultiplier = 1f;

    public static LightManager Instance
    {
        get
        {
            if (_instance == null)
            {
                _instance = FindObjectOfType<LightManager>();
                if (_instance == null)
                {
                    GameObject singleton = new GameObject(typeof(LightManager).ToString());
                    _instance = singleton.AddComponent<LightManager>();
                }
            }
            return _instance;
        }
        private set => _instance = value;
    }

    private void Awake()
    {
        if (_instance == null)
        {
            _instance = this;
            DontDestroyOnLoad(gameObject); // Optional: Makes sure the instance persists across scenes.
        }
        else if (_instance != this)
        {
            Debug.LogWarning("Multiple instances of LightManager detected. Destroying the new one.");
            Destroy(gameObject);
        }
    }

    [Header("Light Settings")]
    [SerializeField]
    public Light DirectionalLight;
    [SerializeField]
    Light[] PointLights;
    
    // point lights
    private int pointLightsCount;
    public ComputeBuffer pointLightsBuffer;

    public void UpdateBuffer(ComputeShader tracingShader)
    {
        Vector3 dir = DirectionalLight.transform.forward;
        Vector3 directionalLightInfo = new Vector3(-dir.x, -dir.y, -dir.z);
        Vector4 directionalLightColorInfo = new Vector4(
            DirectionalLight.color.r,
            DirectionalLight.color.g,
            DirectionalLight.color.b,
            DirectionalLight.intensity * DirectionalLightIntensityMultiplier
        );
        
        tracingShader.SetVector("_InverseDirectionalLight", directionalLightInfo);
        tracingShader.SetVector("_DirectionalLightColor", directionalLightColorInfo);
        tracingShader.SetBuffer(0,"_PointLights", pointLightsBuffer);
        tracingShader.SetInt("_PointLightsCount", LightManager.Instance.GetPointLightsCount());
    }
    
    public void UpdateLights()
    {
        pointLightsCount = 0;
        List<Vector4> pointLightsPosColor = new List<Vector4>();

        foreach (Light light in PointLights)
        {
            if (light == null || light.gameObject.activeSelf == false || light.type != LightType.Point) continue;

            pointLightsPosColor.Add(new Vector4(
                light.transform.position.x,
                light.transform.position.y,
                light.transform.position.z,
                light.range));
            pointLightsPosColor.Add(new Vector4(
                light.color.r,
                light.color.g,
                light.color.b,
                light.intensity));

            ++pointLightsCount;
        }

        if (pointLightsCount == 0)
            pointLightsPosColor.Add(Vector4.zero);

        int neededCount = pointLightsPosColor.Count;

        if (pointLightsBuffer == null || pointLightsBuffer.count != neededCount)
        {
            pointLightsBuffer?.Release();
            pointLightsBuffer = new ComputeBuffer(neededCount, 4 * sizeof(float));
        }

        pointLightsBuffer.SetData(pointLightsPosColor);
    }

    private void OnDisable()
    {
        if (pointLightsBuffer != null)
            pointLightsBuffer.Release();
    }
    
    public int GetPointLightsCount()
    {
        return pointLightsCount;
    }
}
