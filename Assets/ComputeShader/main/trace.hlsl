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

float2 SampleDisk(float u1, float u2)
{
    float r     = sqrt(u1);
    float theta = 2.0 * PI * u2;
    return float2(r * cos(theta), r * sin(theta));
}

float GetSphericalCapSolidAngle(float cosThetaMax)
{
    return 2.0 * PI * (1.0 - cosThetaMax);
}

float GetDiskArea(float radius)
{
    return PI * radius * radius;
}

void BuildOrthonormalBasis(float3 n, out float3 tangent, out float3 bitangent)
{
    float3 up = abs(n.y) < 0.99 ? float3(0.0, 1.0, 0.0) : float3(1.0, 0.0, 0.0);
    tangent = normalize(cross(up, n));
    bitangent = cross(n, tangent);
}

float3 SampleUniformSphericalCap(float3 axis, float cosThetaMax, float u1, float u2)
{
    float3 tangent;
    float3 bitangent;
    BuildOrthonormalBasis(axis, tangent, bitangent);

    float cosTheta = lerp(1.0, cosThetaMax, u1);
    float sinTheta = sqrt(saturate(1.0 - cosTheta * cosTheta));
    float phi = 2.0 * PI * u2;
    float sinPhi;
    float cosPhi;
    sincos(phi, sinPhi, cosPhi);

    return normalize(
        tangent * (cosPhi * sinTheta) +
        bitangent * (sinPhi * sinTheta) +
        axis * cosTheta);
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
    float  proposalPdf; // full light proposal pdf for this sample
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

void EnqueueShadowRay(float3 origin, float3 direction, float maxDist, float3 illumination, float proposalPdf, uint pixelIndex)
{
    uint idx;
    InterlockedAdd(BufferSizes[CurBounce].shadowRays, 1, idx);

    ShadowRayData sr;
    sr.origin = origin;
    sr.direction = direction;
    sr.maxDist = maxDist;
    sr.illumination = illumination;
    sr.proposalPdf = proposalPdf;
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
    reservoir.proposalPdf = 0.0;
    reservoir.lightType = 0u;
    reservoir.lightIndex = 0u;
    reservoir.sampleCount = 0u;
    reservoir.selectedWeight = 0.0;
    DirectLightReservoirs[pixelIndex] = reservoir;
    DirectLightDebugOutput[pixelIndex] = _DirectLightDebugView == 4
        ? float3(0.03, 0.03, 0.03)
        : float3(0.0, 0.0, 0.0);
}

bool IsDirectLightRISEstimateDebugView()
{
    return _DirectLightDebugView == 2;
}

bool IsDirectLightRISErrorDebugView()
{
    return _DirectLightDebugView == 3;
}

bool IsDirectLightReservoirStatusDebugView()
{
    return _DirectLightDebugView == 4;
}

void MarkDirectLightReservoirStatusDebug(
    uint pixelIndex,
    bool hasHistoryFlag,
    bool accepted,
    bool selected,
    uint sampleCount)
{
    float sampleSignal = lerp(0.08, 0.28, saturate((float)sampleCount / 8.0));
    float3 color = float3(0.0, 0.0, sampleSignal);

    if (sampleCount == 0u)
        color = float3(0.85, 0.0, 0.0);
    else if (!hasHistoryFlag)
        color = float3(0.85, 0.85, 0.0);
    else if (!accepted)
        color = float3(1.0, 0.35, 0.0);
    else if (selected)
        color = float3(1.0, 1.0, 1.0);
    else
        color = float3(0.0, 1.0, 0.0);

    DirectLightDebugOutput[pixelIndex] = color;
}

float GetDirectLightTarget(float3 contribution)
{
    return max(0.0, dot(max(contribution, float3(0.0, 0.0, 0.0)), LUM));
}

bool IsValidDirectLightSample(DirectLightSample sample)
{
    return sample.targetLum > 0.0 && sample.proposalPdf > 0.0;
}

float GetDirectLightResamplingWeight(float weightSum, float targetLum, uint sampleCount)
{
    if (targetLum <= 0.0 || sampleCount == 0u)
        return 0.0;

    return weightSum / targetLum / max((float)sampleCount, 1.0);
}

void CompleteDirectLightSample(inout DirectLightSample sample, float3 contribution)
{
    sample.contribution = contribution;
    sample.targetLum = GetDirectLightTarget(contribution);
    sample.reservoirWeight = sample.targetLum / max(sample.proposalPdf, 1e-6);
    sample.illumination = contribution / max(sample.proposalPdf, 1e-6);
}

float3 GetDirectLightSurfaceNormal(RayHit hit, float3 V)
{
    float3 N = hit.normal;
    if (hit.mode >= 3.0 && dot(V, N) < 0.0)
        N = -N;
    return N;
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
    reservoir.proposalPdf = sample.proposalPdf;
    reservoir.lightType = sample.lightType;
    reservoir.lightIndex = sample.lightIndex;
    reservoir.sampleCount = 1u;
    // For one candidate: W = (target / proposalPdf) / target = 1 / proposalPdf.
    // Final ReSTIR DI can reconstruct the current estimator as contribution * W.
    reservoir.selectedWeight = GetDirectLightResamplingWeight(reservoir.weightSum, sample.targetLum, 1u);
    return reservoir;
}

bool BuildSunDirectLightSample(
    RayHit hit,
    float3 V,
    float3 throughput,
    float proposalPdf,
    out DirectLightSample sample);

bool BuildPointLightDirectSample(
    RayHit hit,
    float3 V,
    float3 throughput,
    uint lightIdx,
    float proposalPdf,
    out DirectLightSample sample);

bool ReevaluateSunDirectLightSample(
    RayHit hit,
    float3 V,
    float3 throughput,
    float3 sampleDir,
    float proposalPdf,
    out DirectLightSample sample);

bool ReevaluatePointLightDirectSample(
    RayHit hit,
    float3 V,
    float3 throughput,
    uint lightIdx,
    float3 samplePos,
    float proposalPdf,
    out DirectLightSample sample);

void StoreInitialDirectLightReservoir(DirectLightSample sample, float3 surfaceNormal, uint pixelIndex)
{
    if (CurBounce != 0 || !IsValidDirectLightSample(sample))
        return;

    DirectLightReservoirs[pixelIndex] = MakeInitialDirectLightReservoir(sample, surfaceNormal);
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

bool ShouldRecordDirectLightReservoir()
{
    return CurBounce == 0 && (
        _UseDirectLightReservoirRIS ||
        IsDirectLightRISEstimateDebugView() ||
        IsDirectLightRISErrorDebugView() ||
        IsDirectLightReservoirStatusDebugView());
}

void QueueDirectLightSample(DirectLightSample sample, uint pixelIndex)
{
    EnqueueShadowRay(sample.origin, sample.direction, sample.maxDist, sample.illumination, sample.proposalPdf, pixelIndex);
}

bool SampleDirectLightCandidate(
    RayHit hit,
    float3 V,
    float3 throughput,
    bool hasSun,
    uint pointLightOffset,
    bool useCulledList,
    uint candidateIndex,
    float proposalPdf,
    out DirectLightSample sample)
{
    sample = (DirectLightSample)0;
    DirectLightSample branchSample = (DirectLightSample)0;
    bool accepted = false;
    if (hasSun && candidateIndex == 0u)
    {
        accepted = BuildSunDirectLightSample(hit, V, throughput, proposalPdf, branchSample);
    }
    else
    {
        uint pointCandidateIndex = candidateIndex - (hasSun ? 1u : 0u);
        uint lightIdx = ResolvePointLightIndex(pointCandidateIndex, pointLightOffset, useCulledList);
        accepted = BuildPointLightDirectSample(hit, V, throughput, lightIdx, proposalPdf, branchSample);
    }

    if (accepted)
        sample = branchSample;
    return accepted;
}

bool UpdateDirectLightRISSelection(
    inout DirectLightSample selectedSample,
    inout float weightSum,
    inout bool hasSelectedSample,
    DirectLightSample candidate)
{
    if (!IsValidDirectLightSample(candidate))
        return false;

    float w = candidate.reservoirWeight;
    weightSum += w;
    if (!hasSelectedSample)
    {
        selectedSample = candidate;
        hasSelectedSample = true;
        return true;
    }

    float r = RNG_Next(rng) * weightSum;
    if (r < w)
    {
        selectedSample = candidate;
        return true;
    }

    return false;
}

int2 GetDirectLightNeighborOffset(uint neighborIndex)
{
    switch (neighborIndex & 7u)
    {
        case 0u: return int2(-1, 0);
        case 1u: return int2(1, 0);
        case 2u: return int2(0, -1);
        case 3u: return int2(0, 1);
        case 4u: return int2(-1, -1);
        case 5u: return int2(1, -1);
        case 6u: return int2(-1, 1);
        default: return int2(1, 1);
    }
}

bool TryGetDirectLightNeighborPixelIndex(uint pixelIndex, uint neighborOrdinal, out uint neighborPixelIndex)
{
    int2 pixelCoord = int2((int)(pixelIndex % _ScreenWidth), (int)(pixelIndex / _ScreenWidth));
    int2 neighborCoord = pixelCoord + GetDirectLightNeighborOffset(neighborOrdinal);
    if (neighborCoord.x < 0 || neighborCoord.y < 0 || neighborCoord.x >= (int)_ScreenWidth || neighborCoord.y >= (int)_ScreenHeight)
    {
        neighborPixelIndex = 0u;
        return false;
    }

    neighborPixelIndex = (uint)(neighborCoord.y * (int)_ScreenWidth + neighborCoord.x);
    return true;
}

bool BuildSunDirectLightSample(
    RayHit hit,
    float3 V,
    float3 throughput,
    float proposalPdf,
    out DirectLightSample sample)
{
    float3 surfaceNormal = GetDirectLightSurfaceNormal(hit, V);
    sample.origin = hit.position + hit.normal * 1e-5;
    sample.direction = float3(0.0, 0.0, 0.0);
    sample.maxDist = 1e20;
    sample.contribution = float3(0.0, 0.0, 0.0);
    sample.illumination = float3(0.0, 0.0, 0.0);
    sample.proposalPdf = proposalPdf;
    sample.targetLum = 0.0;
    sample.reservoirWeight = 0.0;
    sample.lightType = 1u;
    sample.lightIndex = 0u;

    float3 L0 = normalize(_InverseDirectionalLight);
    float3 sampleDir = L0;
    float3 color = _DirectionalLightColor.rgb * _DirectionalLightColor.a;
    if (_SunAngularRadius > 0.0)
    {
        float cosThetaMax = cos(_SunAngularRadius);
        float solidAngle = max(GetSphericalCapSolidAngle(cosThetaMax), 1e-6);
        sampleDir = SampleUniformSphericalCap(L0, cosThetaMax, RNG_Next(rng), RNG_Next(rng));
        sample.proposalPdf *= rcp(solidAngle);
        // DirectionalLight intensity in this project is treated as the legacy
        // total directional contribution, not radiance per steradian. Once we
        // broaden the sun to a finite spherical cap, convert that strength into
        // a density over the cap so changing SunAngularRadius only changes the
        // sampled support, not the overall direct-light energy.
        color *= rcp(solidAngle);
    }

    sample.origin = hit.position + surfaceNormal * 1e-5;
    float NdotL = saturate(dot(surfaceNormal, sampleDir));
    if (NdotL <= 0.0)
        return false;

    float3 f_brdf;
    float dummyPdf;
    EvaluateBXDF_GivenDir(hit, V, sampleDir, /*out*/ f_brdf, /*out*/ dummyPdf);

    sample.direction = sampleDir;
    CompleteDirectLightSample(sample, throughput * color * f_brdf * NdotL);
    return true;
}

bool ReevaluateSunDirectLightSample(
    RayHit hit,
    float3 V,
    float3 throughput,
    float3 sampleDir,
    float proposalPdf,
    out DirectLightSample sample)
{
    float3 surfaceNormal = GetDirectLightSurfaceNormal(hit, V);
    sample.origin = hit.position + surfaceNormal * 1e-5;
    sample.direction = normalize(sampleDir);
    sample.maxDist = 1e20;
    sample.contribution = float3(0.0, 0.0, 0.0);
    sample.illumination = float3(0.0, 0.0, 0.0);
    sample.proposalPdf = proposalPdf;
    sample.targetLum = 0.0;
    sample.reservoirWeight = 0.0;
    sample.lightType = 1u;
    sample.lightIndex = 0u;

    float NdotL = saturate(dot(surfaceNormal, sample.direction));
    if (NdotL <= 0.0)
        return false;

    float3 color = _DirectionalLightColor.rgb * _DirectionalLightColor.a;
    if (_SunAngularRadius > 0.0)
    {
        float solidAngle = max(GetSphericalCapSolidAngle(cos(_SunAngularRadius)), 1e-6);
        color *= rcp(solidAngle);
    }

    float3 f_brdf;
    float dummyPdf;
    EvaluateBXDF_GivenDir(hit, V, sample.direction, /*out*/ f_brdf, /*out*/ dummyPdf);
    CompleteDirectLightSample(sample, throughput * color * f_brdf * NdotL);
    return true;
}

bool BuildPointLightDirectSample(
    RayHit hit,
    float3 V,
    float3 throughput,
    uint lightIdx,
    float proposalPdf,
    out DirectLightSample sample)
{
    float3 surfaceNormal = GetDirectLightSurfaceNormal(hit, V);
    sample.origin = hit.position + surfaceNormal * 1e-5;
    sample.direction = float3(0.0, 0.0, 0.0);
    sample.maxDist = 0.0;
    sample.contribution = float3(0.0, 0.0, 0.0);
    sample.illumination = float3(0.0, 0.0, 0.0);
    sample.proposalPdf = proposalPdf;
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
    float NdotL = saturate(dot(surfaceNormal, Ls));
    if (NdotL <= 0.0)
        return false;

    float3 f_brdf;
    float dummyPdf;
    EvaluateBXDF_GivenDir(hit, V, Ls, /*out*/ f_brdf, /*out*/ dummyPdf);

    float3 Le = light.color * light.intensity;
    float3 sampleLe = Le;
    float3 contribution = throughput * Le * (f_brdf * NdotL);
    if (radius <= 1e-4)
    {
        contribution *= rcp(max(distS * distS, 1e-6));
    }
    else
    {
        float diskArea = max(GetDiskArea(radius), 1e-6);
        float pdf_area = rcp(diskArea);
        // PointLight intensity is also authored as the legacy total light
        // strength. When we jitter visibility over a finite disk to soften
        // shadows, convert that strength into a density over the disk so
        // SourceRadius changes the support, not the average brightness.
        sampleLe *= pdf_area;
        float3 nLight = normalize(hit.position - light.position);
        float cosThetaPrime = saturate(dot(nLight, -Ls));
        if (cosThetaPrime <= 1e-6)
            return false;

        float geom = cosThetaPrime / max(distS * distS, 1e-6);
        contribution = throughput * sampleLe * (f_brdf * NdotL) * geom;
        sample.proposalPdf *= pdf_area;
    }

    sample.direction = Ls;
    sample.maxDist = distS;
    CompleteDirectLightSample(sample, contribution * rangeAtten);
    return true;
}

bool ReevaluatePointLightDirectSample(
    RayHit hit,
    float3 V,
    float3 throughput,
    uint lightIdx,
    float3 samplePos,
    float proposalPdf,
    out DirectLightSample sample)
{
    float3 surfaceNormal = GetDirectLightSurfaceNormal(hit, V);
    sample.origin = hit.position + surfaceNormal * 1e-5;
    sample.direction = float3(0.0, 0.0, 0.0);
    sample.maxDist = 0.0;
    sample.contribution = float3(0.0, 0.0, 0.0);
    sample.illumination = float3(0.0, 0.0, 0.0);
    sample.proposalPdf = proposalPdf;
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

    float3 toSample = samplePos - hit.position;
    float distS = length(toSample);
    float3 Ls = toSample / max(distS, 1e-6);
    float NdotL = saturate(dot(surfaceNormal, Ls));
    if (NdotL <= 0.0)
        return false;

    float3 f_brdf;
    float dummyPdf;
    EvaluateBXDF_GivenDir(hit, V, Ls, /*out*/ f_brdf, /*out*/ dummyPdf);

    float3 Le = light.color * light.intensity;
    float radius = max(light.sourceRadius, 0.0);
    float3 contribution = throughput * Le * (f_brdf * NdotL);
    if (radius <= 1e-4)
    {
        contribution *= rcp(max(distS * distS, 1e-6));
    }
    else
    {
        float diskArea = max(GetDiskArea(radius), 1e-6);
        float3 sampleLe = Le * rcp(diskArea);
        float3 nLight = normalize(hit.position - light.position);
        float cosThetaPrime = saturate(dot(nLight, -Ls));
        if (cosThetaPrime <= 1e-6)
            return false;

        float geom = cosThetaPrime / max(distS * distS, 1e-6);
        contribution = throughput * sampleLe * (f_brdf * NdotL) * geom;
    }

    sample.direction = Ls;
    sample.maxDist = distS;
    CompleteDirectLightSample(sample, contribution * rangeAtten);
    return true;
}

bool BuildNeighborReusedDirectLightSample(
    RayHit hit,
    float3 V,
    float3 throughput,
    uint neighborPixelIndex,
    DirectLightReservoirData prevReservoir,
    out DirectLightSample candidate)
{
    candidate = (DirectLightSample)0;
    if (prevReservoir.sampleCount == 0u || prevReservoir.selectedWeight <= 0.0 || prevReservoir.proposalPdf <= 0.0)
        return false;

    bool accepted = false;
    if (prevReservoir.lightType == 1u)
    {
        accepted = ReevaluateSunDirectLightSample(
            hit,
            V,
            throughput,
            prevReservoir.direction,
            prevReservoir.proposalPdf,
            candidate);
    }
    else if (prevReservoir.lightType == 2u)
    {
        float3 samplePos = prevReservoir.origin + prevReservoir.direction * prevReservoir.maxDist;
        accepted = ReevaluatePointLightDirectSample(
            hit,
            V,
            throughput,
            prevReservoir.lightIndex,
            samplePos,
            prevReservoir.proposalPdf,
            candidate);
    }

    if (!accepted || !IsValidDirectLightSample(candidate))
        return false;

    candidate.reservoirWeight = candidate.targetLum * prevReservoir.selectedWeight * max((float)prevReservoir.sampleCount, 1.0);
    return true;
}

void GenerateShadowRays(RayHit hit, float3 V, float3 throughput, uint pixelIndex)
{
    bool usePrimaryReservoir = CurBounce == 0;
    bool recordPrimaryReservoir = ShouldRecordDirectLightReservoir();
    uint risCount = max((uint)_DirectLightRISCandidateCount, 1u);
    bool useDirectLightRIS = CurBounce == 0 && _UseDirectLightReservoirRIS;
    bool useNeighborReuse = useDirectLightRIS && _UseDirectLightReservoirNeighborReuse && _HasDirectLightReservoirHistory;
    // This is previous-frame neighbor replay only. It is not the paper's
    // temporal reuse pass, and it is not a full same-frame spatial pass.
    float3 surfaceNormal = GetDirectLightSurfaceNormal(hit, V);

    if (usePrimaryReservoir && IsDirectLightReservoirStatusDebugView())
        DirectLightDebugOutput[pixelIndex] = float3(0.0, 0.0, 0.08);

    bool hasSun = _DirectionalLightColor.a > 0.0;

    uint pointLightCount;
    uint pointLightOffset;
    bool useCulledList;
    bool hasPointLights = GetPointLightCandidateRange(pointLightCount, pointLightOffset, useCulledList);

    uint candidateCount = (hasSun ? 1u : 0u) + (hasPointLights ? pointLightCount : 0u);
    if (candidateCount == 0u)
    {
        if (usePrimaryReservoir && IsDirectLightReservoirStatusDebugView())
            DirectLightDebugOutput[pixelIndex] = float3(0.08, 0.0, 0.0);
        return;
    }

    float proposalPdf = 1.0 / (float)candidateCount;
    float u = RNG_Next(rng);
    uint candidateIndex = min(uint(u * candidateCount), candidateCount - 1u);
    DirectLightSample sample = (DirectLightSample)0;
    bool accepted = false;

    accepted = SampleDirectLightCandidate(
        hit,
        V,
        throughput,
        hasSun,
        pointLightOffset,
        useCulledList,
        candidateIndex,
        proposalPdf,
        sample);

    // Fallback keeps the first drawn candidate from the active proposal.
    // In RIS mode this means the same proposal family as the RIS candidates.
    DirectLightSample fallbackSample = sample;
    bool hasAcceptedFallbackSample = accepted;
    bool hasFallbackSample = accepted && IsValidDirectLightSample(sample);
    uint pathRngStateAfterDirectLight = rng.state;

    if (!useDirectLightRIS)
    {
        if (!accepted)
        {
            if (usePrimaryReservoir && IsDirectLightReservoirStatusDebugView())
                DirectLightDebugOutput[pixelIndex] = float3(1.0, 0.0, 1.0);
            return;
        }

        if (usePrimaryReservoir && recordPrimaryReservoir)
        {
            StoreInitialDirectLightReservoir(sample, surfaceNormal, pixelIndex);
            if (IsDirectLightReservoirStatusDebugView())
            {
                DirectLightReservoirData debugReservoir = DirectLightReservoirs[pixelIndex];
                MarkDirectLightReservoirStatusDebug(
                    pixelIndex,
                    _HasDirectLightReservoirHistory,
                    false,
                    false,
                    debugReservoir.sampleCount);
            }
        }

        rng.state = pathRngStateAfterDirectLight;
        QueueDirectLightSample(sample, pixelIndex);
        return;
    }

    DirectLightSample selectedSample = (DirectLightSample)0;
    bool hasSelectedSample = false;
    bool selectedLastCandidate = false;
    float weightSum = 0.0;
    uint representedSampleCount = risCount;

    if (accepted && IsValidDirectLightSample(sample))
    {
        selectedLastCandidate = UpdateDirectLightRISSelection(
            selectedSample,
            weightSum,
            hasSelectedSample,
            sample);
    }

    for (uint i = 1u; i < risCount; ++i)
    {
        u = RNG_Next(rng);
        candidateIndex = min(uint(u * candidateCount), candidateCount - 1u);
        sample = (DirectLightSample)0;
        accepted = false;

        accepted = SampleDirectLightCandidate(
            hit,
            V,
            throughput,
            hasSun,
            pointLightOffset,
            useCulledList,
            candidateIndex,
            proposalPdf,
            sample);

        if (!accepted || !IsValidDirectLightSample(sample))
            continue;

        selectedLastCandidate = UpdateDirectLightRISSelection(
            selectedSample,
            weightSum,
            hasSelectedSample,
            sample);
    }

    if (useNeighborReuse)
    {
        uint neighborReuseCount = min((uint)max(_DirectLightNeighborReuseCount, 1), 8u);
        uint neighborBase = (uint)_FrameCount & 7u;
        for (uint i = 0u; i < neighborReuseCount; ++i)
        {
            uint neighborPixelIndex;
            if (!TryGetDirectLightNeighborPixelIndex(pixelIndex, neighborBase + i, neighborPixelIndex))
                continue;

            DirectLightReservoirData prevReservoir = DirectLightReservoirsPrev[neighborPixelIndex];
            DirectLightSample candidate = (DirectLightSample)0;
            if (!BuildNeighborReusedDirectLightSample(hit, V, throughput, neighborPixelIndex, prevReservoir, candidate))
                continue;

            representedSampleCount += prevReservoir.sampleCount;
            selectedLastCandidate = UpdateDirectLightRISSelection(
                selectedSample,
                weightSum,
                hasSelectedSample,
                candidate);
        }
    }

    if (!hasSelectedSample)
    {
        if (hasAcceptedFallbackSample)
        {
            if (usePrimaryReservoir && recordPrimaryReservoir)
            {
                if (hasFallbackSample)
                    StoreInitialDirectLightReservoir(fallbackSample, surfaceNormal, pixelIndex);
                else
                    ClearDirectLightReservoir(pixelIndex);
            }
            rng.state = pathRngStateAfterDirectLight;
            QueueDirectLightSample(fallbackSample, pixelIndex);
            return;
        }

        if (usePrimaryReservoir && IsDirectLightReservoirStatusDebugView())
            DirectLightDebugOutput[pixelIndex] = float3(1.0, 0.0, 1.0);
        rng.state = pathRngStateAfterDirectLight;
        return;
    }

    float selectedWeight = GetDirectLightResamplingWeight(
        weightSum,
        selectedSample.targetLum,
        representedSampleCount);
    if (usePrimaryReservoir && recordPrimaryReservoir)
    {
        DirectLightReservoirData debugReservoir = MakeInitialDirectLightReservoir(selectedSample, surfaceNormal);
        debugReservoir.weightSum = weightSum;
        debugReservoir.sampleCount = representedSampleCount;
        debugReservoir.selectedWeight = selectedWeight;
        DirectLightReservoirs[pixelIndex] = debugReservoir;
        if (IsDirectLightReservoirStatusDebugView())
        {
            MarkDirectLightReservoirStatusDebug(
                pixelIndex,
                _HasDirectLightReservoirHistory,
                true,
                selectedLastCandidate,
                representedSampleCount);
        }
    }

    float3 risIllumination = selectedSample.contribution * selectedWeight;
    bool validReservoirPayload =
        all(isfinite(risIllumination)) &&
        selectedWeight > 0.0 &&
        dot(max(risIllumination, float3(0.0, 0.0, 0.0)), LUM) > 0.0;

    if (!validReservoirPayload)
    {
        if (hasAcceptedFallbackSample)
        {
            if (usePrimaryReservoir && recordPrimaryReservoir)
            {
                if (hasFallbackSample)
                    StoreInitialDirectLightReservoir(fallbackSample, surfaceNormal, pixelIndex);
                else
                    ClearDirectLightReservoir(pixelIndex);
            }
            rng.state = pathRngStateAfterDirectLight;
            QueueDirectLightSample(fallbackSample, pixelIndex);
            return;
        }

        rng.state = pathRngStateAfterDirectLight;
        return;
    }

    rng.state = pathRngStateAfterDirectLight;
    EnqueueShadowRay(
        selectedSample.origin,
        selectedSample.direction,
        selectedSample.maxDist,
        risIllumination,
        selectedSample.proposalPdf,
        pixelIndex);
    return;
}
