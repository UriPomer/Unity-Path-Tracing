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

#define DIRECTIONAL_LIGHT_SAMPLE 1
#define POINT_LIGHT_SAMPLES 1

float2 SampleDisk(float u1, float u2)
{
    float r     = sqrt(u1);
    float theta = 2.0 * PI * u2;
    return float2(r * cos(theta), r * sin(theta));
}

float3 AccumulateSunLight(
    RayHit hit,
    float3 viewDir
)
{
    if (_DirectionalLightColor.a <= 0.0)
        return 0;

    float3 L0     = normalize(_InverseDirectionalLight);
    float3 color  = _DirectionalLightColor.rgb * _DirectionalLightColor.a;

    float3 up     = abs(L0.y) < 0.99 ? float3(0,1,0) : float3(1,0,0);
    float3 right  = normalize(cross(up, L0));
    float3 up2    = cross(L0, right);

    float3 accum = float3(0,0,0);
    [unroll]
    for (int i = 0; i < DIRECTIONAL_LIGHT_SAMPLE; ++i)
    {
        float u1 = RNG_Next(rng);
        float u2 = RNG_Next(rng);

        float2 d = SampleDisk(u1, u2) * _SunAngularRadius;

        float3 sampleDir = normalize(L0 + d.x * right + d.y * up2);

        Ray shadowRay = GenRay(hit.position + hit.normal * 1e-5, sampleDir);
        if (TraceHit(shadowRay, 1e20))
            continue;

        float cosA  = saturate(dot(viewDir, sampleDir));
        float diskV = pow(cosA, _SunFocus);

        accum += diskV * color;
    }
    return accum / float(DIRECTIONAL_LIGHT_SAMPLE);
}

float3 GetLightContribution(RayHit hit, float3 dir)
{
    float3 lightContribution = AccumulateSunLight(hit, dir);

    // float3 lightContribution = 0.f;
    // Ray shadowRay = GenRay(hit.position + hit.normal * 1e-5, _InverseDirectionalLight);
    // if (_DirectionalLightColor.a > 0.0 && !TraceHit(shadowRay, 1.#INF))
    // {
    //     // lightContribution += _DirectionalLightColor.rgb * _DirectionalLightColor.a;
    //     float3 sunDir = normalize(_InverseDirectionalLight);
    //     float cosA = saturate(dot(dir, sunDir));
    //     float disk = pow(cosA, _SunFocus);
    //     float3 sunColor = disk
    //                     * _DirectionalLightColor.rgb
    //                     * _DirectionalLightColor.a;
    //     lightContribution += sunColor;
    // }
    
    if (_PointLightsCount == 0)
    {
        return lightContribution;
    }

    int samples = min(POINT_LIGHT_SAMPLES, _PointLightsCount);
    float inv_pdf = _PointLightsCount / float(samples);
    [unroll]
    for (int i = 0; i < POINT_LIGHT_SAMPLES; ++i)
    {
        float u = RNG_Next(rng);
        uint idx = min(uint(u * _PointLightsCount), _PointLightsCount - 1);

        float4 lightPos = _PointLights[idx * 2];
        float4 lightColor = _PointLights[idx * 2 + 1];
        if (lightColor.a <= 0.0)
        {
            continue;
        }
        float3 toLight = lightPos.xyz - hit.position;
        float  dist    = length(toLight);
        float3 L       = toLight / dist;

        Ray shadowRay = GenRay(hit.position + hit.normal * 1e-5, L);
        if (!TraceHit(shadowRay, dist))
        {
            float radius = lightPos.w;
            if (radius > 0.0)
            {
                float x = saturate(1.0 - dist / radius);
                float attenuation = x * x;
                lightContribution += lightColor.rgb * lightColor.a * attenuation * inv_pdf;
            }
        }
    }
    return lightContribution;
}