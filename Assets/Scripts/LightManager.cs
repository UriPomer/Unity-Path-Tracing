using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LightManager : MonoBehaviour
{
    private static LightManager _instance;

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
    
    
    private Vector3 directionalLightInfo;
    private Vector4 directionalLightColorInfo;
    // angles in radians
    private float directionalLightYaw = 0.0f;
    private float directionalLightPitch = 0.0f;
    // point lights
    private int pointLightsCount;
    public ComputeBuffer pointLightsBuffer;
    
    public void UpdateLights()
    {
        // record directional light info
        Vector3 dir = DirectionalLight.transform.forward;
        directionalLightInfo = new Vector3(
            -dir.x, -dir.y, -dir.z
        );
        directionalLightInfo = Vector3.Normalize(directionalLightInfo);
        directionalLightColorInfo = new Vector4(
            DirectionalLight.color.r,
            DirectionalLight.color.g,
            DirectionalLight.color.b,
            DirectionalLight.intensity
        );
        // prepare point lights
        if (pointLightsBuffer != null)
            pointLightsBuffer.Release();
        List<Vector4> pointLightsPosColor = new List<Vector4>();
        foreach(Light light in PointLights)
        {
            if (light.type != LightType.Point) continue;
            pointLightsCount++;
            pointLightsPosColor.Add(new Vector4(
                light.transform.position.x,
                light.transform.position.y,
                light.transform.position.z,
                light.range
            ));
            pointLightsPosColor.Add(new Vector4(
                light.color.r,
                light.color.g,
                light.color.b,
                light.intensity
            ));
        }
        // if no point light, insert empty vector to make buffer happy
        if (pointLightsCount == 0)
            pointLightsPosColor.Add(Vector4.zero);
        pointLightsBuffer = new ComputeBuffer(pointLightsPosColor.Count, 4 * sizeof(float));
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
