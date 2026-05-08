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

struct DirectLightSample
{
    float3 origin;
    float3 direction;
    float  maxDist;
    float3 contribution;
    float3 illumination;
    float  selectPdf;
    float  targetLum;
    float  reservoirWeight;
    uint   lightType; // 1 = sun, 2 = point
    uint   lightIndex;
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

void ClearDirectLightReservoir(uint pixelIndex)
{
    DirectLightReservoirData reservoir;
    reservoir.origin = float3(0.0, 0.0, 0.0);
    reservoir.maxDist = 0.0;
    reservoir.direction = float3(0.0, 0.0, 0.0);
    reservoir.targetLum = 0.0;
    reservoir.contribution = float3(0.0, 0.0, 0.0);
    reservoir.weightSum = 0.0;
    reservoir.lightType = 0u;
    reservoir.lightIndex = 0u;
    reservoir.sampleCount = 0u;
    reservoir.selectedWeight = 0.0;
    DirectLightReservoirs[pixelIndex] = reservoir;
}

float GetDirectLightTarget(float3 contribution)
{
    return max(0.0, dot(max(contribution, float3(0.0, 0.0, 0.0)), LUM));
}

void CompleteDirectLightSample(inout DirectLightSample sample, float3 contribution)
{
    sample.contribution = contribution;
    sample.targetLum = GetDirectLightTarget(contribution);
    sample.reservoirWeight = sample.targetLum / max(sample.selectPdf, 1e-6);
    sample.illumination = contribution / max(sample.selectPdf, 1e-6);
}

void StoreInitialDirectLightReservoir(DirectLightSample sample, uint pixelIndex)
{
    if (CurBounce != 0 || sample.targetLum <= 0.0)
        return;

    DirectLightReservoirData reservoir;
    reservoir.origin = sample.origin;
    reservoir.maxDist = sample.maxDist;
    reservoir.direction = sample.direction;
    reservoir.targetLum = sample.targetLum;
    reservoir.contribution = sample.contribution;
    reservoir.weightSum = sample.reservoirWeight;
    reservoir.lightType = sample.lightType;
    reservoir.lightIndex = sample.lightIndex;
    reservoir.sampleCount = 1u;
    // For one candidate: W = (target / sourcePdf) / target = 1 / sourcePdf.
    // Final ReSTIR DI can reconstruct the current estimator as contribution * W.
    reservoir.selectedWeight = reservoir.weightSum / max(sample.targetLum, 1e-6);
    DirectLightReservoirs[pixelIndex] = reservoir;
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

uint ResolvePointLightIndex(uint candidateIndex, uint lightOffset, bool useCulledList)
{
    return useCulledList
        ? _LightCullingData[lightOffset + candidateIndex]
        : candidateIndex;
}

void EnqueueDirectLightSample(DirectLightSample sample, uint pixelIndex)
{
    StoreInitialDirectLightReservoir(sample, pixelIndex);
    EnqueueShadowRay(sample.origin, sample.direction, sample.maxDist, sample.illumination, pixelIndex);
}

bool BuildSunDirectLightSample(
    RayHit hit,
    float3 V,
    float3 throughput,
    float selectPdf,
    out DirectLightSample sample)
{
    sample.origin = hit.position + hit.normal * 1e-5;
    sample.direction = float3(0.0, 0.0, 0.0);
    sample.maxDist = 1e20;
    sample.contribution = float3(0.0, 0.0, 0.0);
    sample.illumination = float3(0.0, 0.0, 0.0);
    sample.selectPdf = selectPdf;
    sample.targetLum = 0.0;
    sample.reservoirWeight = 0.0;
    sample.lightType = 1u;
    sample.lightIndex = 0u;

    float3 L0 = normalize(_InverseDirectionalLight);
    float3 up = abs(L0.y) < 0.99 ? float3(0,1,0) : float3(1,0,0);
    float3 right = normalize(cross(up, L0));
    float3 up2 = cross(L0, right);
    float2 d = SampleDisk(RNG_Next(rng), RNG_Next(rng)) * _SunAngularRadius;
    float3 sampleDir = normalize(L0 + d.x * right + d.y * up2);

    float NdotL = saturate(dot(hit.normal, sampleDir));
    if (NdotL <= 0.0)
        return false;

    float3 f_brdf;
    float dummyPdf;
    EvaluateBXDF_GivenDir(hit, V, sampleDir, /*out*/ f_brdf, /*out*/ dummyPdf);

    float3 color = _DirectionalLightColor.rgb * _DirectionalLightColor.a;
    float diskV = pow(saturate(dot(V, sampleDir)), _SunFocus);
    sample.direction = sampleDir;
    CompleteDirectLightSample(sample, throughput * diskV * color * f_brdf * NdotL);
    return true;
}

bool BuildPointLightDirectSample(
    RayHit hit,
    float3 V,
    float3 throughput,
    uint lightIdx,
    float selectPdf,
    out DirectLightSample sample)
{
    sample.origin = hit.position + hit.normal * 1e-5;
    sample.direction = float3(0.0, 0.0, 0.0);
    sample.maxDist = 0.0;
    sample.contribution = float3(0.0, 0.0, 0.0);
    sample.illumination = float3(0.0, 0.0, 0.0);
    sample.selectPdf = selectPdf;
    sample.targetLum = 0.0;
    sample.reservoirWeight = 0.0;
    sample.lightType = 2u;
    sample.lightIndex = lightIdx;

    PointLightData light = LoadPointLight(lightIdx);
    if (light.intensity <= 0.0 || light.range <= 0.0)
        return false;

    float3 toCenter = light.position - hit.position;
    float distC = length(toCenter);
    float rangeAtten = GetPointLightRangeAttenuation(distC, light.range);
    if (rangeAtten <= 0.0)
        return false;

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
        return false;

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
            return false;

        float geom = cosThetaPrime / max(distS * distS, 1e-6);
        illum = throughput * Le * (f_brdf * NdotL) * (geom / pdf_area);
    }

    sample.direction = Ls;
    sample.maxDist = distS;
    CompleteDirectLightSample(sample, illum * rangeAtten);
    return true;
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
    float selectPdf = 1.0 / (float)candidateCount;
    DirectLightSample sample;

    if (hasSun && candidateIndex == 0u)
    {
        if (BuildSunDirectLightSample(hit, V, throughput, selectPdf, sample))
            EnqueueDirectLightSample(sample, pixelIndex);
        return;
    }

    uint pointCandidateIndex = candidateIndex - (hasSun ? 1u : 0u);
    uint lightIdx = ResolvePointLightIndex(pointCandidateIndex, pointLightOffset, useCulledList);

    if (BuildPointLightDirectSample(hit, V, throughput, lightIdx, selectPdf, sample))
        EnqueueDirectLightSample(sample, pixelIndex);
}
