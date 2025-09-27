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

float3 AccumulatePointLightSoft(
    RayHit hit, 
    uint lightIdx,
    float inv_pdf
)
{
    float4 lightPosRad   = _PointLights[lightIdx * 2];
    float4 lightColorA   = _PointLights[lightIdx * 2 + 1];
    float radius         = lightPosRad.w;
    float3 baseColor     = lightColorA.rgb * lightColorA.a;
    if (lightColorA.a <= 0.0 || radius <= 0.0)
        return 0;

    float3 toCenter = lightPosRad.xyz - hit.position;
    float  distC    = length(toCenter);
    float3 L0       = toCenter / distC;

    float3 upRef  = abs(L0.y) < 0.99 ? float3(0,1,0) : float3(1,0,0);
    float3 right  = normalize(cross(upRef, L0));
    float3 up2    = cross(L0, right);

    float3 accum = float3(0,0,0);

    [unroll]
    for (int s = 0; s < POINT_LIGHT_SAMPLES; ++s)
    {
        float u1 = RNG_Next(rng);
        float u2 = RNG_Next(rng);
        float2 d  = SampleDisk(u1, u2) * radius;

        float3 samplePos = lightPosRad.xyz 
                         + d.x * right 
                         + d.y * up2;

        float3 toSample = samplePos - hit.position;
        float  distS    = length(toSample);
        float3 Ls       = toSample / distS;

        Ray shadowRay = GenRay(hit.position + hit.normal * 1e-5, Ls);
        if (TraceHit(shadowRay, distS))
            continue;

        float atten = saturate(1.0 - distS / (radius * 2.0));
        accum += baseColor * atten * inv_pdf;
    }

    return accum / float(POINT_LIGHT_SAMPLES);
}

uint2 GetTileIndex(uint2 pixelCoord)
{
    return pixelCoord / TILE_SIZE;
}

float3 GetLightContribution(RayHit hit, float3 viewDir)
{
    float3 lightContribution = AccumulateSunLight(hit, viewDir);

    if (_PointLightsCount > 0)
    {
        bool useLightCulling = _TileCount.x > 0 && _TileCount.y > 0;

        if (useLightCulling)
        {
            uint2 tileIndex = GetTileIndex((uint2)_Pixel);
            if (tileIndex.x < _TileCount.x && tileIndex.y < _TileCount.y)
            {
                uint tileId = tileIndex.y * _TileCount.x + tileIndex.x;

                // 获取该tile的光源信息
                uint2 tileData = _TileData[tileId];
                uint lightCount = tileData.x;
                uint lightOffset = tileData.y;

                if (lightCount > 0)
                {
                    int samples = min(POINT_LIGHT_SAMPLES, (int)lightCount);
                    float inv_pdf = lightCount / float(samples);
                    for (int i = 0; i < POINT_LIGHT_SAMPLES && i < (int)lightCount; ++i)
                    {
                        float u = RNG_Next(rng);
                        uint sampleIndex = min(uint(u * lightCount), lightCount - 1);
                        uint lightIndex = _LightCullingData[lightOffset + sampleIndex];
                        lightContribution += AccumulatePointLightSoft(hit, lightIndex, inv_pdf);
                    }
                    return lightContribution;
                }
            }
        }

        int samples = min(POINT_LIGHT_SAMPLES, _PointLightsCount);
        float inv_pdf = _PointLightsCount / float(samples);

        for (int i = 0; i < POINT_LIGHT_SAMPLES; ++i)
        {
            float u = RNG_Next(rng);
            uint idx = min(uint(u * _PointLightsCount), _PointLightsCount - 1);
            lightContribution += AccumulatePointLightSoft(hit, idx, inv_pdf);
        }
    }

    return lightContribution;
}