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

int _DirectLightRISCandidateCount;

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

void EnqueueShadowRay(float3 origin, float3 direction, float maxDist, float3 illumination, float selectPdf, uint pixelIndex)
{
    uint idx;
    InterlockedAdd(BufferSizes[CurBounce].shadowRays, 1, idx);

    ShadowRayData sr;
    sr.origin = origin;
    sr.direction = direction;
    sr.maxDist = maxDist;
    sr.illumination = illumination;
    sr.selectPdf = selectPdf;
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
    reservoir.surfaceNormal = float3(0.0, 0.0, 0.0);
    reservoir.selectPdf = 0.0;
    reservoir.lightType = 0u;
    reservoir.lightIndex = 0u;
    reservoir.sampleCount = 0u;
    reservoir.selectedWeight = 0.0;
    DirectLightReservoirs[pixelIndex] = reservoir;
    DirectLightReservoirDifference[pixelIndex] = _ShowDirectLightTemporalReuseDebug
        ? float3(0.03, 0.03, 0.03)
        : float3(0.0, 0.0, 0.0);
}

void MarkDirectLightTemporalReuseDebug(
    uint pixelIndex,
    bool temporalReuseEnabled,
    bool hasHistoryFlag,
    bool hasTemporalHistory,
    bool hadPrevReservoir,
    bool accepted,
    bool selected,
    uint sampleCount)
{
    float sampleSignal = lerp(0.08, 0.28, saturate((float)sampleCount / 8.0));
    float3 color = float3(0.0, 0.0, sampleSignal);

    if (!temporalReuseEnabled)
        color = float3(0.85, 0.0, 0.0);
    else if (!hasHistoryFlag)
        color = float3(0.85, 0.85, 0.0);
    else if (!hasTemporalHistory)
        color = float3(0.85, 0.45, 0.0);
    else if (!hadPrevReservoir)
        color = float3(0.0, 0.85, 0.85);
    else if (!accepted)
        color = float3(1.0, 0.35, 0.0);
    else if (selected)
        color = float3(1.0, 1.0, 1.0);
    else
        color = float3(0.0, 1.0, 0.0);

    DirectLightReservoirDifference[pixelIndex] = color;
}

float GetDirectLightTarget(float3 contribution)
{
    return max(0.0, dot(max(contribution, float3(0.0, 0.0, 0.0)), LUM));
}

bool IsValidDirectLightSample(DirectLightSample sample)
{
    return sample.targetLum > 0.0 && sample.selectPdf > 0.0;
}

void CompleteDirectLightSample(inout DirectLightSample sample, float3 contribution)
{
    sample.contribution = contribution;
    sample.targetLum = GetDirectLightTarget(contribution);
    sample.reservoirWeight = sample.targetLum / max(sample.selectPdf, 1e-6);
    sample.illumination = contribution / max(sample.selectPdf, 1e-6);
}

DirectLightReservoirData MakeInitialDirectLightReservoir(DirectLightSample sample, float3 surfaceNormal)
{
    DirectLightReservoirData reservoir;
    reservoir.origin = sample.origin;
    reservoir.maxDist = sample.maxDist;
    reservoir.direction = sample.direction;
    reservoir.targetLum = sample.targetLum;
    reservoir.contribution = sample.contribution;
    reservoir.weightSum = sample.reservoirWeight;
    reservoir.surfaceNormal = surfaceNormal;
    reservoir.selectPdf = sample.selectPdf;
    reservoir.lightType = sample.lightType;
    reservoir.lightIndex = sample.lightIndex;
    reservoir.sampleCount = 1u;
    // For one candidate: W = (target / sourcePdf) / target = 1 / sourcePdf.
    // Final ReSTIR DI can reconstruct the current estimator as contribution * W.
    reservoir.selectedWeight = reservoir.weightSum / max(sample.targetLum, 1e-6);
    return reservoir;
}

bool BuildSunDirectLightSample(
    RayHit hit,
    float3 V,
    float3 throughput,
    float selectPdf,
    out DirectLightSample sample);

bool BuildPointLightDirectSample(
    RayHit hit,
    float3 V,
    float3 throughput,
    uint lightIdx,
    float selectPdf,
    out DirectLightSample sample);

void StoreInitialDirectLightReservoir(DirectLightSample sample, float3 surfaceNormal, uint pixelIndex)
{
    if (CurBounce != 0 || !IsValidDirectLightSample(sample))
        return;

    DirectLightReservoirs[pixelIndex] = MakeInitialDirectLightReservoir(sample, surfaceNormal);
}

void StoreInitialDirectLightReservoir(DirectLightSample sample, uint pixelIndex)
{
    StoreInitialDirectLightReservoir(sample, float3(0.0, 1.0, 0.0), pixelIndex);
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

void EnqueueDirectLightSample(DirectLightSample sample, float3 surfaceNormal, uint pixelIndex)
{
    StoreInitialDirectLightReservoir(sample, surfaceNormal, pixelIndex);
    EnqueueShadowRay(sample.origin, sample.direction, sample.maxDist, sample.illumination, sample.selectPdf, pixelIndex);
}

void EnqueueDirectLightSample(DirectLightSample sample, uint pixelIndex)
{
    EnqueueDirectLightSample(sample, float3(0.0, 1.0, 0.0), pixelIndex);
}

bool UpdateDirectLightReservoirCandidate(inout DirectLightReservoirData reservoir, DirectLightSample sample, float3 surfaceNormal)
{
    if (sample.targetLum <= 0.0 || sample.selectPdf <= 0.0)
        return false;

    float w = sample.reservoirWeight;
    reservoir.weightSum += w;
    reservoir.sampleCount += 1u;

    float r = RNG_Next(rng) * reservoir.weightSum;
    if (r < w || reservoir.sampleCount == 1u)
    {
        reservoir.origin = sample.origin;
        reservoir.maxDist = sample.maxDist;
        reservoir.direction = sample.direction;
        reservoir.targetLum = sample.targetLum;
        reservoir.contribution = sample.contribution;
        reservoir.surfaceNormal = surfaceNormal;
        reservoir.selectPdf = sample.selectPdf;
        reservoir.lightType = sample.lightType;
        reservoir.lightIndex = sample.lightIndex;
        reservoir.selectedWeight = reservoir.weightSum / max(reservoir.targetLum, 1e-6);
        return true;
    }

    reservoir.selectedWeight = reservoir.weightSum / max(reservoir.targetLum, 1e-6);
    return false;
}

bool BuildDirectLightReservoirCandidate(
    RayHit hit,
    float3 V,
    float3 throughput,
    DirectLightReservoirData reservoir,
    out DirectLightSample sample)
{
    sample = (DirectLightSample)0;
    if (reservoir.sampleCount == 0u || reservoir.selectPdf <= 0.0)
        return false;

    sample.origin = hit.position + hit.normal * 1e-5;
    sample.direction = float3(0.0, 0.0, 0.0);
    sample.maxDist = reservoir.maxDist;
    sample.contribution = float3(0.0, 0.0, 0.0);
    sample.illumination = float3(0.0, 0.0, 0.0);
    sample.selectPdf = reservoir.selectPdf;
    sample.targetLum = 0.0;
    sample.reservoirWeight = 0.0;
    sample.lightType = reservoir.lightType;
    sample.lightIndex = reservoir.lightIndex;
    float3 prevNormal = normalize(reservoir.surfaceNormal);
    if (dot(prevNormal, prevNormal) <= 0.0)
        return false;

    if (dot(hit.normal, prevNormal) < 0.7)
        return false;

    if (reservoir.lightType == 1u)
    {
        float3 sampleDir = normalize(reservoir.direction);
        if (dot(sampleDir, sampleDir) <= 0.0)
            return false;

        float NdotL = saturate(dot(hit.normal, sampleDir));
        if (NdotL <= 0.0)
            return false;

        float3 f_brdf;
        float dummyPdf;
        EvaluateBXDF_GivenDir(hit, V, sampleDir, /*out*/ f_brdf, /*out*/ dummyPdf);

        float3 color = _DirectionalLightColor.rgb * _DirectionalLightColor.a;
        sample.direction = sampleDir;
        sample.maxDist = reservoir.maxDist;
        CompleteDirectLightSample(sample, throughput * color * f_brdf * NdotL);
        return true;
    }

    if (reservoir.lightType == 2u)
    {
        PointLightData light = LoadPointLight(reservoir.lightIndex);
        if (light.intensity <= 0.0 || light.range <= 0.0)
            return false;

        float3 samplePos = reservoir.origin + reservoir.direction * reservoir.maxDist;
        float3 toSample = samplePos - hit.position;
        float distS = length(toSample);
        if (distS <= 0.0)
            return false;

        float3 Ls = toSample / distS;
        float NdotL = saturate(dot(hit.normal, Ls));
        if (NdotL <= 0.0)
            return false;

        float rangeAtten = GetPointLightRangeAttenuation(distS, light.range);
        if (rangeAtten <= 0.0)
            return false;

        float3 f_brdf;
        float dummyPdf;
        EvaluateBXDF_GivenDir(hit, V, Ls, /*out*/ f_brdf, /*out*/ dummyPdf);

        float3 Le = light.color * light.intensity;
        float3 illum = throughput * Le * (f_brdf * NdotL) / max(distS * distS, 1e-6);

        sample.direction = Ls;
        sample.maxDist = distS;
        CompleteDirectLightSample(sample, illum * rangeAtten);
        return true;
    }

    return false;
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
    sample.direction = sampleDir;
    CompleteDirectLightSample(sample, throughput * color * f_brdf * NdotL);
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
    bool usePrimaryReservoir = CurBounce == 0;
    bool useDirectLightRIS = CurBounce == 0 && _UseDirectLightReservoirRIS;

    if (usePrimaryReservoir && _ShowDirectLightTemporalReuseDebug)
        DirectLightReservoirDifference[pixelIndex] = float3(0.0, 0.0, 0.08);

    bool hasSun = _DirectionalLightColor.a > 0.0;

    uint pointLightCount;
    uint pointLightOffset;
    bool useCulledList;
    bool hasPointLights = GetPointLightCandidateRange(pointLightCount, pointLightOffset, useCulledList);

    uint candidateCount = (hasSun ? 1u : 0u) + (hasPointLights ? pointLightCount : 0u);
    if (candidateCount == 0u)
    {
        if (usePrimaryReservoir && _ShowDirectLightTemporalReuseDebug)
            DirectLightReservoirDifference[pixelIndex] = float3(0.08, 0.0, 0.0);
        return;
    }

    float selectPdf = 1.0 / (float)candidateCount;
    float u = RNG_Next(rng);
    uint candidateIndex = min(uint(u * candidateCount), candidateCount - 1u);
    DirectLightSample sample = (DirectLightSample)0;
    bool accepted = false;

    if (hasSun && candidateIndex == 0u)
    {
        accepted = BuildSunDirectLightSample(hit, V, throughput, selectPdf, sample);
    }
    else
    {
        uint pointCandidateIndex = candidateIndex - (hasSun ? 1u : 0u);
        uint lightIdx = ResolvePointLightIndex(pointCandidateIndex, pointLightOffset, useCulledList);
        accepted = BuildPointLightDirectSample(hit, V, throughput, lightIdx, selectPdf, sample);
    }

    if (!useDirectLightRIS)
    {
        if (!accepted)
        {
            if (usePrimaryReservoir && _ShowDirectLightTemporalReuseDebug)
                DirectLightReservoirDifference[pixelIndex] = float3(1.0, 0.0, 1.0);
            return;
        }

        if (usePrimaryReservoir)
        {
            StoreInitialDirectLightReservoir(sample, hit.normal, pixelIndex);
            if (_ShowDirectLightTemporalReuseDebug)
            {
                DirectLightReservoirData debugReservoir = DirectLightReservoirs[pixelIndex];
                MarkDirectLightTemporalReuseDebug(
                    pixelIndex,
                    _UseDirectLightReservoirTemporalReuse,
                    _HasDirectLightReservoirHistory,
                    false,
                    false,
                    false,
                    false,
                    debugReservoir.sampleCount);
            }
        }

        EnqueueDirectLightSample(sample, hit.normal, pixelIndex);
        return;
    }

    uint risCount = max((uint)_DirectLightRISCandidateCount, 1u);
    DirectLightReservoirData reservoir = (DirectLightReservoirData)0;
    bool hasSelectedSample = false;
    bool selectedLastCandidate = false;

    if (accepted)
    {
        reservoir = MakeInitialDirectLightReservoir(sample, hit.normal);
        hasSelectedSample = true;
        selectedLastCandidate = true;
    }

    for (uint i = 1u; i < risCount; ++i)
    {
        u = RNG_Next(rng);
        candidateIndex = min(uint(u * candidateCount), candidateCount - 1u);
        sample = (DirectLightSample)0;
        accepted = false;

        if (hasSun && candidateIndex == 0u)
        {
            accepted = BuildSunDirectLightSample(hit, V, throughput, selectPdf, sample);
        }
        else
        {
            uint pointCandidateIndex = candidateIndex - (hasSun ? 1u : 0u);
            uint lightIdx = ResolvePointLightIndex(pointCandidateIndex, pointLightOffset, useCulledList);
            accepted = BuildPointLightDirectSample(hit, V, throughput, lightIdx, selectPdf, sample);
        }

        if (!accepted)
            continue;

        if (!hasSelectedSample)
        {
            reservoir = MakeInitialDirectLightReservoir(sample, hit.normal);
            hasSelectedSample = true;
            selectedLastCandidate = true;
            continue;
        }

        selectedLastCandidate = UpdateDirectLightReservoirCandidate(reservoir, sample, hit.normal);
    }

    if (!hasSelectedSample)
    {
        if (usePrimaryReservoir && _ShowDirectLightTemporalReuseDebug)
            DirectLightReservoirDifference[pixelIndex] = float3(1.0, 0.0, 1.0);
        return;
    }

    reservoir.selectedWeight /= max((float)risCount, 1.0);
    DirectLightReservoirs[pixelIndex] = reservoir;
    if (usePrimaryReservoir && _ShowDirectLightTemporalReuseDebug)
    {
        MarkDirectLightTemporalReuseDebug(
            pixelIndex,
            _UseDirectLightReservoirTemporalReuse,
            _HasDirectLightReservoirHistory,
            false,
            false,
            true,
            selectedLastCandidate,
            reservoir.sampleCount);
    }

    float3 risIllumination = reservoir.contribution * reservoir.selectedWeight;
    EnqueueShadowRay(
        reservoir.origin,
        reservoir.direction,
        reservoir.maxDist,
        risIllumination,
        reservoir.selectPdf,
        pixelIndex);
}
