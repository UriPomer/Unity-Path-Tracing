#pragma once

#include "global.hlsl"
#include "bxdf.hlsl"

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

    return envColor * _SkyboxIntensity;
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

struct PointLightData
{
    float3 position;
    float  range;
    float3 color;
    float  intensity;
    float  sourceRadius;
};

PointLightData LoadPointLight(uint lightIdx)
{
    float4 lightPosRange = _PointLights[lightIdx * 3];
    float4 lightColorIntensity = _PointLights[lightIdx * 3 + 1];
    float4 lightMeta = _PointLights[lightIdx * 3 + 2];

    PointLightData light;
    light.position = lightPosRange.xyz;
    light.range = lightPosRange.w;
    light.color = lightColorIntensity.rgb;
    light.intensity = lightColorIntensity.a;
    light.sourceRadius = lightMeta.x;
    return light;
}

float GetPointLightRangeAttenuation(float distanceToLight, float lightRange)
{
    if (lightRange <= 0.0 || distanceToLight >= lightRange)
        return 0.0;

    float x = saturate(distanceToLight / lightRange);
    float fade = 1.0 - x * x * x * x;
    return fade * fade;
}

uint2 GetTileIndex(uint2 pixelCoord)
{
    return pixelCoord / TILE_SIZE;
}

void EnqueueShadowRay(float3 origin, float3 direction, float maxDist, float3 illumination, uint pixelIndex)
{
    uint idx;
    InterlockedAdd(BufferSizes[CurBounce].shadowRays, 1, idx);

    ShadowRayData sr;
    sr.origin = origin;
    sr.direction = direction;
    sr.maxDist = maxDist;
    sr.illumination = illumination;
    sr.pixelIndex = pixelIndex;
    ShadowRaysBuffer[idx] = sr;
}

bool GetPointLightCandidateRange(out uint lightCount, out uint lightOffset, out bool useCulledList)
{
    lightCount = 0;
    lightOffset = 0;
    useCulledList = false;

    if (_PointLightsCount <= 0)
        return false;

    bool useLightCulling = _TileCount.x > 0 && _TileCount.y > 0;
    if (!useLightCulling)
    {
        lightCount = (uint)_PointLightsCount;
        return lightCount > 0;
    }

    uint2 tileIndex = GetTileIndex((uint2)_Pixel);
    if (tileIndex.x >= _TileCount.x || tileIndex.y >= _TileCount.y)
        return false;

    uint tileId = tileIndex.y * _TileCount.x + tileIndex.x;
    uint2 tileData = _TileData[tileId];
    if (tileData.x == 0)
        return false;

    lightCount = tileData.x;
    lightOffset = tileData.y;
    useCulledList = true;
    return true;
}

void EnqueueSunShadowRay(RayHit hit, float3 V, float3 throughput, uint pixelIndex, float invSelectPdf)
{
    float3 L0 = normalize(_InverseDirectionalLight);
    float3 up = abs(L0.y) < 0.99 ? float3(0,1,0) : float3(1,0,0);
    float3 right = normalize(cross(up, L0));
    float3 up2 = cross(L0, right);
    float2 d = SampleDisk(RNG_Next(rng), RNG_Next(rng)) * _SunAngularRadius;
    float3 sampleDir = normalize(L0 + d.x * right + d.y * up2);

    float NdotL = saturate(dot(hit.normal, sampleDir));
    if (NdotL <= 0.0)
        return;

    float3 f_brdf;
    float dummyPdf;
    EvaluateBXDF_GivenDir(hit, V, sampleDir, /*out*/ f_brdf, /*out*/ dummyPdf);

    float3 color = _DirectionalLightColor.rgb * _DirectionalLightColor.a;
    float diskV = pow(saturate(dot(V, sampleDir)), _SunFocus);
    float3 illum = throughput * diskV * color * f_brdf * NdotL * invSelectPdf;
    EnqueueShadowRay(hit.position + hit.normal * 1e-5, sampleDir, 1e20, illum, pixelIndex);
}

void EnqueuePointLightShadowRay(RayHit hit, float3 V, float3 throughput, uint pixelIndex, uint lightIdx, float invSelectPdf)
{
    PointLightData light = LoadPointLight(lightIdx);
    if (light.intensity <= 0.0 || light.range <= 0.0)
        return;

    float3 toCenter = light.position - hit.position;
    float distC = length(toCenter);
    float rangeAtten = GetPointLightRangeAttenuation(distC, light.range);
    if (rangeAtten <= 0.0)
        return;

    float3 L0 = toCenter / max(distC, 1e-6);
    float3 upRef = abs(L0.y) < 0.99 ? float3(0,1,0) : float3(1,0,0);
    float3 right = normalize(cross(upRef, L0));
    float3 up2 = cross(L0, right);

    float radius = max(light.sourceRadius, 0.0);
    float3 samplePos = light.position;
    if (radius > 1e-4)
    {
        float2 d = SampleDisk(RNG_Next(rng), RNG_Next(rng)) * radius;
        samplePos = light.position + d.x * right + d.y * up2;
    }

    float3 toSample = samplePos - hit.position;
    float distS = length(toSample);
    float3 Ls = toSample / max(distS, 1e-6);
    float NdotL = saturate(dot(hit.normal, Ls));
    if (NdotL <= 0.0)
        return;

    float3 f_brdf;
    float dummyPdf;
    EvaluateBXDF_GivenDir(hit, V, Ls, /*out*/ f_brdf, /*out*/ dummyPdf);

    float3 Le = light.color * light.intensity;
    float3 illum;
    if (radius <= 1e-4)
    {
        illum = throughput * Le * (f_brdf * NdotL) / max(distS * distS, 1e-6);
    }
    else
    {
        float pdf_area = 1.0 / (PI * radius * radius);
        float3 nLight = normalize(hit.position - light.position);
        float cosThetaPrime = saturate(dot(nLight, -Ls));
        if (cosThetaPrime <= 1e-6)
            return;

        float geom = cosThetaPrime / max(distS * distS, 1e-6);
        illum = throughput * Le * (f_brdf * NdotL) * (geom / pdf_area);
    }

    illum *= rangeAtten * invSelectPdf;
    EnqueueShadowRay(hit.position + hit.normal * 1e-5, Ls, distS, illum, pixelIndex);
}

void GenerateShadowRays(RayHit hit, float3 V, float3 throughput, uint pixelIndex)
{
    bool hasSun = _DirectionalLightColor.a > 0.0;

    uint pointLightCount;
    uint pointLightOffset;
    bool useCulledList;
    bool hasPointLights = GetPointLightCandidateRange(pointLightCount, pointLightOffset, useCulledList);

    uint candidateCount = (hasSun ? 1u : 0u) + (hasPointLights ? pointLightCount : 0u);
    if (candidateCount == 0u)
        return;

    // Emit one shadow candidate per path/bounce. D3D11 cannot atomically add float3
    // radiance, so avoiding multiple same-pixel shadow writes prevents lost light.
    float u = RNG_Next(rng);
    uint candidateIndex = min(uint(u * candidateCount), candidateCount - 1u);
    float invSelectPdf = (float)candidateCount;

    if (hasSun && candidateIndex == 0u)
    {
        EnqueueSunShadowRay(hit, V, throughput, pixelIndex, invSelectPdf);
        return;
    }

    uint pointCandidateIndex = candidateIndex - (hasSun ? 1u : 0u);
    uint lightIdx = useCulledList
        ? _LightCullingData[pointLightOffset + pointCandidateIndex]
        : pointCandidateIndex;

    EnqueuePointLightShadowRay(hit, V, throughput, pixelIndex, lightIdx, invSelectPdf);
}
