using System.Collections.Generic;
using UnityEngine;

[ExecuteAlways]
[DisallowMultipleComponent]
[RequireComponent(typeof(Light))]
public class RayTracingPointLight : MonoBehaviour
{
    private static readonly List<RayTracingPointLight> ActivePointLightsInternal = new List<RayTracingPointLight>();
    public static IReadOnlyList<RayTracingPointLight> ActivePointLights => ActivePointLightsInternal;

    [SerializeField, Min(0.0f)]
    private float sourceRadius = 3.0f;

    private Light cachedLight;

    public Light AttachedLight
    {
        get
        {
            if (cachedLight == null)
                cachedLight = GetComponent<Light>();
            return cachedLight;
        }
    }

    public float SourceRadius
    {
        get => sourceRadius;
        set => sourceRadius = Mathf.Max(0.0f, value);
    }

    public int ComputeStateHash()
    {
        unchecked
        {
            int hash = 17;
            hash = hash * 31 + gameObject.activeSelf.GetHashCode();
            hash = hash * 31 + transform.position.GetHashCode();
            hash = hash * 31 + SourceRadius.GetHashCode();

            Light light = AttachedLight;
            if (light != null)
            {
                hash = hash * 31 + light.enabled.GetHashCode();
                hash = hash * 31 + light.color.GetHashCode();
                hash = hash * 31 + light.intensity.GetHashCode();
                hash = hash * 31 + light.range.GetHashCode();
            }

            return hash;
        }
    }

    private void OnEnable()
    {
        CacheLight();
        Register();
    }

    private void OnDisable()
    {
        ActivePointLightsInternal.Remove(this);
    }

    private void OnValidate()
    {
        sourceRadius = Mathf.Max(0.0f, sourceRadius);
        CacheLight();
    }

    private void CacheLight()
    {
        cachedLight = GetComponent<Light>();
    }

    private void Register()
    {
        if (!ActivePointLightsInternal.Contains(this))
            ActivePointLightsInternal.Add(this);
    }
}
