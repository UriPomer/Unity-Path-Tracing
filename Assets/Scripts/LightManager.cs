using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LightManager : MonoBehaviour
{
    private static LightManager _instance;
    
    [SerializeField, Range(0.0f, 32.0f)]
    float DirectionalLightIntensityMultiplier = 1f;
    [SerializeField, Min(0.0f)]
    float DefaultPointLightSourceRadius = 3.0f;

    public static LightManager Instance
    {
        get
        {
            if (_instance == null)
            {
                _instance = FindAnyObjectByType<LightManager>();
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
    bool UseSceneRayTracingPointLights = true;
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
        tracingShader.SetInt("_PointLightsCount", GetPointLightsCount());
    }

    public void UpdateBuffer(ComputeShader tracingShader, int[] kernelIndices)
    {
        UpdateBuffer(tracingShader);
        if (pointLightsBuffer != null)
        {
            foreach (int k in kernelIndices)
                tracingShader.SetBuffer(k, "_PointLights", pointLightsBuffer);
        }
    }

    private List<Vector4> pointLightsPosColor = new List<Vector4>();

    public int ComputeLightStateHash()
    {
        unchecked
        {
            int hash = 17;
            hash = hash * 31 + DefaultPointLightSourceRadius.GetHashCode();
            hash = hash * 31 + UseSceneRayTracingPointLights.GetHashCode();

            if (DirectionalLight != null)
            {
                hash = hash * 31 + DirectionalLight.gameObject.activeSelf.GetHashCode();
                hash = hash * 31 + DirectionalLight.transform.forward.GetHashCode();
                hash = hash * 31 + DirectionalLight.color.GetHashCode();
                hash = hash * 31 + DirectionalLight.intensity.GetHashCode();
            }

            if (UseSceneRayTracingPointLights)
            {
                foreach (RayTracingPointLight pointLight in RayTracingPointLight.ActivePointLights)
                {
                    if (pointLight == null)
                    {
                        hash = hash * 31 + 0;
                        continue;
                    }

                    hash = hash * 31 + pointLight.ComputeStateHash();
                }
            }

            if (PointLights != null)
            {
                hash = hash * 31 + PointLights.Length;
                foreach (Light light in PointLights)
                {
                    if (light == null)
                    {
                        hash = hash * 31 + 0;
                        continue;
                    }

                    hash = hash * 31 + light.gameObject.activeSelf.GetHashCode();
                    hash = hash * 31 + light.enabled.GetHashCode();
                    hash = hash * 31 + light.transform.position.GetHashCode();
                    hash = hash * 31 + light.color.GetHashCode();
                    hash = hash * 31 + light.intensity.GetHashCode();
                    hash = hash * 31 + light.range.GetHashCode();
                }
            }

            return hash;
        }
    }
    
    public void UpdateLights()
    {
        pointLightsCount = 0;
        pointLightsPosColor.Clear();

        HashSet<EntityId> handledLights = new HashSet<EntityId>();

        if (UseSceneRayTracingPointLights)
        {
            foreach (RayTracingPointLight pointLight in RayTracingPointLight.ActivePointLights)
            {
                if (pointLight == null) continue;

                Light light = pointLight.AttachedLight;
                if (light == null || !light.enabled || light.gameObject.activeSelf == false || light.type != LightType.Point)
                    continue;

                AddPointLight(light, pointLight.SourceRadius);
                handledLights.Add(light.GetEntityId());
            }
        }

        foreach (Light light in PointLights)
        {
            if (light == null || !light.enabled || light.gameObject.activeSelf == false || light.type != LightType.Point) continue;
            if (handledLights.Contains(light.GetEntityId())) continue;

            float sourceRadius = DefaultPointLightSourceRadius;
            AddPointLight(light, sourceRadius);
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

    private void AddPointLight(Light light, float sourceRadius)
    {
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
        pointLightsPosColor.Add(new Vector4(
            Mathf.Max(0.0f, sourceRadius),
            0.0f,
            0.0f,
            0.0f));

        ++pointLightsCount;
    }

    private void OnDisable()
    {
        if (pointLightsBuffer != null)
        {
            pointLightsBuffer.Release();
            pointLightsBuffer = null;
        }
    }
    
    public int GetPointLightsCount()
    {
        return pointLightsCount;
    }
}
