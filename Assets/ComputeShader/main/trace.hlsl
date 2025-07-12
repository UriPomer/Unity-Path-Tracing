#pragma once

#include "global.hlsl"


float3 SampleSkybox(Ray ray)
{
    float3 dir = normalize(ray.dir);
    float theta = acos(dir.y) / -PI;
    float phi   = atan2(dir.x, -dir.z) / -PI * 0.5;
    float3 envColor = _SkyboxTexture.SampleLevel(
        sampler_SkyboxTexture,
        float2(phi, theta),
        0
    ).xyz;
    //
    // float3 sunDir = normalize(_InverseDirectionalLight);
    // float cosA = saturate(dot(dir, sunDir));
    // float disk = pow(cosA, 50);
    // float3 sunColor = disk
    //                 * _DirectionalLightColor.rgb
    //                 * _DirectionalLightColor.a;

    return envColor;
}

// trace a ray and returns hit immediately (for shadow rays)
bool TraceHit(Ray ray, float targetDist)
{
    return IntersectTlasFast(ray, targetDist);
}

// trace a ray and detect nearest hit
RayHit Trace(Ray ray)
{
    RayHit bestHit = GenRayHit();
    IntersectTlas(ray, bestHit);
    return bestHit;
}

float3 GetLightContribution(RayHit hit, float3 dir)
{
    float3 lightContribution = 0.0;
    {
        Ray shadowRay = GenRay(hit.position + hit.normal * 1e-5, _InverseDirectionalLight);
        if (_DirectionalLightColor.a > 0.0 && !TraceHit(shadowRay, 1.#INF))
        {
            // lightContribution += _DirectionalLightColor.rgb * _DirectionalLightColor.a;
            float3 sunDir = normalize(_InverseDirectionalLight);
            float cosA = saturate(dot(dir, sunDir));
            float disk = pow(cosA, _SunFocus);
            float3 sunColor = disk
                            * _DirectionalLightColor.rgb
                            * _DirectionalLightColor.a;
            lightContribution += sunColor;
        }
    }

    float u = RNG_Next(rng);
    uint idx = min(uint(u * _PointLightsCount), _PointLightsCount - 1);
    
    // sample point lights
    for (int i = 0; i < _PointLightsCount; i++)
    {
        float4 lightPos = _PointLights[i * 2];
        float4 lightColor = _PointLights[i * 2 + 1];
        if (lightColor.a <= 0.0)
            continue;

        float3 toLight = lightPos.xyz - hit.position;
        float  dist    = length(toLight);
        float3 L       = toLight / dist;

        Ray shadowRay = GenRay(hit.position + hit.normal * 1e-5, L);
        if (TraceHit(shadowRay, dist))
            continue;

        float radius = lightPos.w;
        if (radius <= 0.0) continue;
        float x = saturate(1.0 - dist / radius);
        float attenuation = x * x;
        lightContribution += lightColor.rgb * lightColor.a * attenuation;
    }

    return lightContribution;
}