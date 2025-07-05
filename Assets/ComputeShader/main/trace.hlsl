#pragma once

#include "global.hlsl"


float3 SampleSkybox(Ray ray)
{
    float theta = acos(ray.dir.y) / -PI;
    float phi = atan2(ray.dir.x, - ray.dir.z) / - PI * 0.5f;
    return _SkyboxTexture.SampleLevel(sampler_SkyboxTexture, float2(phi, theta), 0).xyz;
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

float3 GetLightContribution(RayHit hit)
{
    float3 lightContribution = 0.0;
    {
        Ray shadowRay = GenRay(hit.position + hit.normal * 1e-5, _InverseDirectionalLight);
        if (_DirectionalLightColor.a > 0.0 && !TraceHit(shadowRay, 1.#INF))
        {
            lightContribution += hit.material.albedo * saturate(dot(hit.normal, _InverseDirectionalLight)) *
                _DirectionalLightColor.rgb * _DirectionalLightColor.a;
        }
    }
    
    // sample point lights
    for (int i = 0; i < _PointLightsCount; i++)
    {
        float4 lightPos = _PointLights[i * 2];
        float4 lightColor = _PointLights[i * 2 + 1];
        if (lightColor.a <= 0.0)
            continue;
        float3 rayDir = lightPos.xyz - hit.position;
        float rayDist = length(rayDir);
        float distDecay = max(0.0, rayDist - lightPos.w);
        distDecay = pow(0.2, distDecay * distDecay);
        rayDir /= rayDist;
        Ray shadowRay = GenRay(hit.position + hit.normal * 1e-5, rayDir);
        if (!TraceHit(shadowRay, rayDist))
        {
            lightContribution += hit.material.albedo * saturate(dot(hit.normal, rayDir)) *
                lightColor.rgb * lightColor.a * distDecay;
        }
    }

    return lightContribution;
}