#pragma once

#include "global.hlsl"
#include "bxdf.hlsl"

float3 SampleSkybox(Ray ray)
{
    float3 dir = normalize(ray.dir);
    float2 uv = float2(atan2(dir.z, dir.x), acos(clamp(dir.y, -1.0, 1.0)));
    uv /= float2(2.0 * PI, PI);
    uv.x += 0.5;
    uv.y = 1.0 - uv.y;
    float3 envColor = _SkyboxTexture.SampleLevel(
        sampler_SkyboxTexture,
        uv,
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

float3 GetDirectLightNeighborReplayDebugNoSurfaceColor()
{
    return float3(0.15, 0.15, 0.15);
}

float3 GetDirectLightNeighborReplayDebugNoReplayColor()
{
    return float3(1.0, 0.0, 1.0);
}

float3 GetDirectLightNeighborReplayDebugWarmupColor()
{
    return float3(0.0, 0.0, 1.0);
}

float3 GetDirectLightNeighborReplayDebugNoCandidatesColor()
{
    return float3(0.0, 1.0, 1.0);
}

float3 GetDirectLightNeighborReplayDebugNoCurrentSampleColor()
{
    return float3(0.0, 0.0, 0.0);
}

float3 GetDirectLightNeighborReplayDebugNoSourceColor()
{
    return float3(1.0, 1.0, 1.0);
}

float3 GetDirectLightNeighborReplayDebugInvalidSourceColor()
{
    return float3(1.0, 0.5, 0.0);
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

bool IsDirectLightNeighborReplayDebugView()
{
    return _DirectLightDebugView == 5;
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

void MarkDirectLightNeighborReplayDebug(
    uint pixelIndex,
    uint storedReservoirCount,
    uint sourceReservoirCount,
    uint compatibleReservoirCount,
    uint reevaluatedReservoirCount)
{
    if (storedReservoirCount == 0u)
        DirectLightDebugOutput[pixelIndex] = GetDirectLightNeighborReplayDebugNoSourceColor();
    else if (sourceReservoirCount == 0u)
        DirectLightDebugOutput[pixelIndex] = GetDirectLightNeighborReplayDebugInvalidSourceColor();
    else if (compatibleReservoirCount == 0u)
        DirectLightDebugOutput[pixelIndex] = float3(1.0, 0.0, 0.0);
    else if (reevaluatedReservoirCount == 0u)
        DirectLightDebugOutput[pixelIndex] = float3(1.0, 1.0, 0.0);
    else
        DirectLightDebugOutput[pixelIndex] = float3(0.0, 1.0, 0.0);
}

void MarkDirectLightNeighborReplayUnavailableDebug(uint pixelIndex, float3 color)
{
    DirectLightDebugOutput[pixelIndex] = color;
}

bool IsValidNeighborReuseReservoirSource(DirectLightReservoirData prevReservoir);
bool HasStoredNeighborReuseReservoir(DirectLightReservoirData prevReservoir);

void WriteDirectLightDiagnosticSnapshot(uint pixelIndex, bool hasPrimarySurface)
{
    if (CurBounce == 0 && pixelIndex == (uint)_DirectLightDebugPixelIndex)
    {
        DirectLightReservoirData currentReservoir = DirectLightReservoirs[pixelIndex];
        DirectLightReservoirData previousReservoir = DirectLightReservoirsPrev[pixelIndex];
        float currentStored = HasStoredNeighborReuseReservoir(currentReservoir) ? 1.0 : 0.0;
        float previousStored = HasStoredNeighborReuseReservoir(previousReservoir) ? 1.0 : 0.0;
        float previousSourceValid = IsValidNeighborReuseReservoirSource(previousReservoir) ? 1.0 : 0.0;

        DirectLightDiagnostics[0] = float4(
            hasPrimarySurface ? 1.0 : 0.0,
            currentStored,
            previousStored,
            previousSourceValid);
        DirectLightDiagnostics[1] = float4(
            currentReservoir.targetLum,
            currentReservoir.weightSum,
            (float)currentReservoir.sampleCount,
            currentReservoir.maxDist);
        DirectLightDiagnostics[2] = float4(
            previousReservoir.targetLum,
            previousReservoir.weightSum,
            (float)previousReservoir.sampleCount,
            previousReservoir.maxDist);
        DirectLightDiagnostics[3] = float4(
            currentReservoir.proposalPdf,
            currentReservoir.selectedWeight,
            previousReservoir.proposalPdf,
            previousReservoir.selectedWeight);
        DirectLightDiagnostics[4] = float4(
            length(currentReservoir.direction),
            length(previousReservoir.direction),
            (float)currentReservoir.lightType,
            (float)previousReservoir.lightType);
    }
}

float4 MakeDirectLightCandidateDiagnosticData(float candidateCount, float hasSun, float pointLightCount, float firstCandidateIndex)
{
    return float4(candidateCount, hasSun, pointLightCount, firstCandidateIndex);
}

float4 MakeDirectLightFirstDrawDiagnosticData(float firstAccepted, float firstValid, float proposalPdf, float targetLum)
{
    return float4(firstAccepted, firstValid, proposalPdf, targetLum);
}

float4 MakeDirectLightFirstMetaDiagnosticData(float reservoirWeight, float maxDist, float lightType, float sunFacing)
{
    return float4(reservoirWeight, maxDist, lightType, sunFacing);
}

float4 MakeDirectLightRISLocalDiagnosticData(float hasSelectedSample, float hasLocalSelectedSample, float weightSum, float localWeightSum)
{
    return float4(hasSelectedSample, hasLocalSelectedSample, weightSum, localWeightSum);
}

float4 MakeDirectLightRISFinalDiagnosticData(float representedSampleCount, float localRepresentedSampleCount, float finalSelectedWeight, float validReservoirPayload)
{
    return float4(representedSampleCount, localRepresentedSampleCount, finalSelectedWeight, validReservoirPayload);
}

float4 MakeDirectLightMaterialDiagnosticData(float surfaceMode, float surfaceRoughness, float surfaceMetallic, float surfaceAlbedoLum)
{
    return float4(surfaceMode, surfaceRoughness, surfaceMetallic, surfaceAlbedoLum);
}

float4 MakeDirectLightShadingDiagnosticData(float surfaceNdotV, float firstBrdfLum, float firstLightLum, float firstThroughputLum)
{
    return float4(surfaceNdotV, firstBrdfLum, firstLightLum, firstThroughputLum);
}

float4 MakeDirectLightTemporalCompatibilityDiagnosticData(
    float normalDot,
    float planeDistanceCurrent,
    float planeDistancePrevious,
    float modeDelta)
{
    return float4(normalDot, planeDistanceCurrent, planeDistancePrevious, modeDelta);
}

float4 MakeDirectLightRawFloat3DiagnosticData(float3 rawValue)
{
    return float4(rawValue, all(isfinite(rawValue)) ? 1.0 : 0.0);
}

float4 MakeDirectLightNeighborReplayCountDiagnosticData(
    float storedReservoirCount,
    float sourceReservoirCount,
    float compatibleReservoirCount,
    float reevaluatedReservoirCount)
{
    return float4(
        storedReservoirCount,
        sourceReservoirCount,
        compatibleReservoirCount,
        reevaluatedReservoirCount);
}

float4 MakeDirectLightNeighborReplayStateDiagnosticData(
    float replayRequested,
    float replayActive,
    float hasTemporalReusePixel,
    float temporalReplayStage)
{
    return float4(
        replayRequested,
        replayActive,
        hasTemporalReusePixel,
        temporalReplayStage);
}

float4 MakeDirectLightTemporalReplayIndexDiagnosticData(
    float currentPixelIndex,
    float temporalReusePixelIndex,
    float temporalPixelMatchesCurrent,
    float usedSamePixelFallback)
{
    return float4(
        currentPixelIndex,
        temporalReusePixelIndex,
        temporalPixelMatchesCurrent,
        usedSamePixelFallback);
}

void WriteDirectLightSamplingDiagnosticSnapshot(
    uint pixelIndex,
    float4 candidateData,
    float4 firstDrawData,
    float4 firstMetaData,
    float4 risLocalData,
    float4 risFinalData,
    float4 materialData,
    float4 shadingData,
    float4 rawMaterialData,
    float4 rawThroughputData,
    float4 rawDirectionalLightData,
    float4 rawBrdfData,
    float4 rawContributionData,
    float4 finalSampleScalarData,
    float4 finalRawContributionData,
    float4 neighborReplayCountData,
    float4 neighborReplayStateData,
    float4 temporalReplayIndexData)
{
    if (CurBounce == 0 && pixelIndex == (uint)_DirectLightDebugPixelIndex)
    {
        DirectLightDiagnostics[5] = candidateData;
        DirectLightDiagnostics[6] = firstDrawData;
        DirectLightDiagnostics[7] = firstMetaData;
        DirectLightDiagnostics[8] = risLocalData;
        DirectLightDiagnostics[9] = risFinalData;
        DirectLightDiagnostics[10] = materialData;
        DirectLightDiagnostics[11] = shadingData;
        DirectLightDiagnostics[12] = rawMaterialData;
        DirectLightDiagnostics[13] = rawThroughputData;
        DirectLightDiagnostics[14] = rawDirectionalLightData;
        DirectLightDiagnostics[15] = rawBrdfData;
        DirectLightDiagnostics[16] = rawContributionData;
        DirectLightDiagnostics[17] = finalSampleScalarData;
        DirectLightDiagnostics[18] = finalRawContributionData;
        DirectLightDiagnostics[19] = neighborReplayCountData;
        DirectLightDiagnostics[20] = neighborReplayStateData;
        DirectLightDiagnostics[21] = temporalReplayIndexData;
    }
}

float GetPositiveDirectLightLuminance(float3 value)
{
    float3 c = max(value, float3(0.0, 0.0, 0.0));
    return c.x * 0.2126 + c.y * 0.7152 + c.z * 0.0722;
}

float GetDirectLightTarget(float3 contribution)
{
    return GetPositiveDirectLightLuminance(contribution);
}

float4 MakeDirectLightSampleScalarDiagnosticData(DirectLightSample sample)
{
    return float4(
        GetDirectLightTarget(sample.contribution),
        sample.targetLum,
        sample.reservoirWeight,
        all(isfinite(sample.contribution)) && isfinite(sample.targetLum) && isfinite(sample.reservoirWeight) ? 1.0 : 0.0);
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

float GetDirectLightReservoirSelectedWeight(DirectLightReservoirData reservoir)
{
    return GetDirectLightResamplingWeight(
        reservoir.weightSum,
        reservoir.targetLum,
        reservoir.sampleCount);
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
    if (dot(V, N) < 0.0)
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

DirectLightReservoirData MakeResolvedDirectLightReservoir(
    DirectLightSample sample,
    float3 surfaceNormal,
    float weightSum,
    uint sampleCount)
{
    DirectLightReservoirData reservoir = MakeInitialDirectLightReservoir(sample, surfaceNormal);
    reservoir.weightSum = weightSum;
    reservoir.sampleCount = sampleCount;
    reservoir.selectedWeight = GetDirectLightResamplingWeight(weightSum, sample.targetLum, sampleCount);
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
        IsDirectLightReservoirStatusDebugView() ||
        IsDirectLightNeighborReplayDebugView());
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

void UpdateDirectLightFallbackSample(
    bool accepted,
    DirectLightSample candidate,
    inout DirectLightSample fallbackSample,
    inout bool hasAcceptedFallbackSample,
    inout bool hasFallbackSample)
{
    if (!accepted)
        return;

    bool candidateValid = IsValidDirectLightSample(candidate);
    if (!hasAcceptedFallbackSample)
    {
        fallbackSample = candidate;
        hasAcceptedFallbackSample = true;
        hasFallbackSample = candidateValid;
        return;
    }

    if (!hasFallbackSample && candidateValid)
    {
        fallbackSample = candidate;
        hasFallbackSample = true;
    }
}

int2 GetDirectLightNeighborOffset(uint neighborIndex)
{
    uint ring = 1u + ((neighborIndex >> 3u) & 3u);
    int step = (int)(ring * 2u);

    switch (neighborIndex & 7u)
    {
        case 0u: return int2(-step, 0);
        case 1u: return int2(step, 0);
        case 2u: return int2(0, -step);
        case 3u: return int2(0, step);
        case 4u: return int2(-step, -step);
        case 5u: return int2(step, -step);
        case 6u: return int2(-step, step);
        default: return int2(step, step);
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

bool TryGetDirectLightTemporalReprojectionPixelIndex(float3 surfaceOrigin, out uint previousPixelIndex)
{
    float4 previousClip = mul(_PreviousCameraViewProjection, float4(surfaceOrigin, 1.0));
    if (previousClip.w <= 1e-6)
    {
        previousPixelIndex = 0u;
        return false;
    }

    float2 previousNdc = previousClip.xy / previousClip.w;
    if (any(previousNdc < -1.0) || any(previousNdc > 1.0))
    {
        previousPixelIndex = 0u;
        return false;
    }

    float2 previousScreen = previousNdc * 0.5 + 0.5;
    uint2 previousPixel = (uint2)floor(previousScreen * float2(_ScreenWidth, _ScreenHeight));
    previousPixel = min(previousPixel, uint2(_ScreenWidth - 1u, _ScreenHeight - 1u));
    previousPixelIndex = previousPixel.y * _ScreenWidth + previousPixel.x;
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
    hit.normal = surfaceNormal;
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

    // _InverseDirectionalLight already points from the surface toward the light
    // source, matching LightManager and the reference project.
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
    hit.normal = surfaceNormal;
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
    hit.normal = surfaceNormal;
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
    hit.normal = surfaceNormal;
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

bool IsValidNeighborReuseReservoirSource(DirectLightReservoirData prevReservoir)
{
    uint lightType = prevReservoir.lightType;
    if (lightType != 1u && lightType != 2u)
    {
        // Recover directional-light history from the stable geometric fields when
        // the packed metadata lane comes back as zeroed. This keeps previous-frame
        // replay driven by the reservoir's actual estimator invariants instead of
        // hard-failing on a missing lightType tag.
        if (prevReservoir.maxDist >= 1e19 && prevReservoir.targetLum > 0.0 && prevReservoir.weightSum > 0.0)
            lightType = 1u;
    }

    return (lightType == 1u || lightType == 2u) &&
        prevReservoir.targetLum > 0.0 &&
        prevReservoir.weightSum > 0.0 &&
        prevReservoir.maxDist > 0.0 &&
        isfinite(prevReservoir.maxDist) &&
        all(isfinite(prevReservoir.origin)) &&
        all(isfinite(prevReservoir.direction));
}

bool HasStoredNeighborReuseReservoir(DirectLightReservoirData prevReservoir)
{
    return prevReservoir.targetLum > 0.0 &&
        prevReservoir.weightSum > 0.0 &&
        prevReservoir.maxDist > 0.0 &&
        isfinite(prevReservoir.maxDist) &&
        all(isfinite(prevReservoir.origin)) &&
        all(isfinite(prevReservoir.direction));
}

uint GetNeighborReuseSourceLightType(DirectLightReservoirData prevReservoir)
{
    if (prevReservoir.lightType == 1u || prevReservoir.lightType == 2u)
        return prevReservoir.lightType;

    if (prevReservoir.maxDist >= 1e19 && HasStoredNeighborReuseReservoir(prevReservoir))
        return 1u;

    return 0u;
}

uint GetNeighborReuseSourceSampleCount(DirectLightReservoirData prevReservoir)
{
    return max(prevReservoir.sampleCount, 1u);
}

float GetNeighborReuseSourceProposalPdf(DirectLightReservoirData prevReservoir)
{
    return (isfinite(prevReservoir.proposalPdf) && prevReservoir.proposalPdf > 0.0)
        ? prevReservoir.proposalPdf
        : 1.0;
}

float GetNeighborReuseSourceSelectedWeight(DirectLightReservoirData prevReservoir)
{
    float selectedWeight = prevReservoir.selectedWeight;
    if (!isfinite(selectedWeight) || selectedWeight <= 0.0)
    {
        selectedWeight = GetDirectLightResamplingWeight(
            prevReservoir.weightSum,
            prevReservoir.targetLum,
            GetNeighborReuseSourceSampleCount(prevReservoir));
    }

    return selectedWeight;
}

bool IsDirectLightNeighborReuseCompatible(float3 surfaceOrigin, float3 surfaceNormal, DirectLightReservoirData prevReservoir)
{
    const float minNormalDot = 0.9;
    const float maxPlaneDistance = 0.05;

    float normalDot = dot(surfaceNormal, prevReservoir.surfaceNormal);
    if (normalDot < minNormalDot)
        return false;

    float3 surfaceDelta = prevReservoir.origin - surfaceOrigin;
    float planeDistanceCurrent = abs(dot(surfaceDelta, surfaceNormal));
    float planeDistancePrevious = abs(dot(surfaceDelta, prevReservoir.surfaceNormal));
    return planeDistanceCurrent <= maxPlaneDistance && planeDistancePrevious <= maxPlaneDistance;
}

bool IsDirectLightTemporalReuseCompatible(
    float3 surfaceOrigin,
    float3 surfaceNormal,
    float surfaceMode,
    HitData prevSurfaceHistory,
    DirectLightReservoirData prevReservoir)
{
    if (prevSurfaceHistory.distance >= 1e19)
        return false;

    float3 prevSurfaceNormal = prevSurfaceHistory.normal;
    if (length(prevSurfaceNormal) <= 1e-6)
        return false;

    prevSurfaceNormal = normalize(prevSurfaceNormal);
    if (dot(prevSurfaceNormal, prevReservoir.surfaceNormal) < 0.0)
        prevSurfaceNormal = -prevSurfaceNormal;

    const float minNormalDot = 0.9;
    const float maxPlaneDistance = 0.05;

    float normalDot = dot(surfaceNormal, prevSurfaceNormal);
    if (normalDot < minNormalDot)
        return false;

    float3 surfaceDelta = prevSurfaceHistory.position - surfaceOrigin;
    float planeDistanceCurrent = abs(dot(surfaceDelta, surfaceNormal));
    float planeDistancePrevious = abs(dot(surfaceDelta, prevSurfaceNormal));
    if (planeDistanceCurrent > maxPlaneDistance || planeDistancePrevious > maxPlaneDistance)
        return false;

    return abs(prevSurfaceHistory.mode - surfaceMode) <= 0.25;
}

float4 GetDirectLightTemporalCompatibilityDiagnosticData(
    float3 surfaceOrigin,
    float3 surfaceNormal,
    float surfaceMode,
    HitData prevSurfaceHistory,
    DirectLightReservoirData prevReservoir)
{
    if (prevSurfaceHistory.distance >= 1e19)
        return float4(-1.0, -1.0, -1.0, -1.0);

    float3 prevSurfaceNormal = prevSurfaceHistory.normal;
    if (length(prevSurfaceNormal) <= 1e-6)
        return float4(-2.0, -2.0, -2.0, -2.0);

    prevSurfaceNormal = normalize(prevSurfaceNormal);
    if (dot(prevSurfaceNormal, prevReservoir.surfaceNormal) < 0.0)
        prevSurfaceNormal = -prevSurfaceNormal;

    float normalDot = dot(surfaceNormal, prevSurfaceNormal);
    float3 surfaceDelta = prevSurfaceHistory.position - surfaceOrigin;
    float planeDistanceCurrent = abs(dot(surfaceDelta, surfaceNormal));
    float planeDistancePrevious = abs(dot(surfaceDelta, prevSurfaceNormal));
    float modeDelta = abs(prevSurfaceHistory.mode - surfaceMode);
    return MakeDirectLightTemporalCompatibilityDiagnosticData(normalDot, planeDistanceCurrent, planeDistancePrevious, modeDelta);
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
    if (!IsValidNeighborReuseReservoirSource(prevReservoir))
        return false;

    uint sourceLightType = GetNeighborReuseSourceLightType(prevReservoir);
    float sourceProposalPdf = GetNeighborReuseSourceProposalPdf(prevReservoir);
    bool accepted = false;
    if (sourceLightType == 1u)
    {
        accepted = ReevaluateSunDirectLightSample(
            hit,
            V,
            throughput,
            prevReservoir.direction,
            sourceProposalPdf,
            candidate);
    }
    else if (sourceLightType == 2u)
    {
        float3 samplePos = prevReservoir.origin + prevReservoir.direction * prevReservoir.maxDist;
        accepted = ReevaluatePointLightDirectSample(
            hit,
            V,
            throughput,
            prevReservoir.lightIndex,
            samplePos,
            sourceProposalPdf,
            candidate);
    }

    if (!accepted || !IsValidDirectLightSample(candidate))
        return false;

    float prevSelectedWeight = GetNeighborReuseSourceSelectedWeight(prevReservoir);
    if (!isfinite(prevSelectedWeight) || prevSelectedWeight <= 0.0)
        return false;

    candidate.reservoirWeight = candidate.targetLum * prevSelectedWeight * max((float)GetNeighborReuseSourceSampleCount(prevReservoir), 1.0);
    return true;
}

void StoreDirectLightReservoirHistory(
    bool useDirectLightRIS,
    bool hasLocalReservoirSample,
    DirectLightSample localReservoirSample,
    float localReservoirWeightSum,
    uint localReservoirSampleCount,
    bool hasFallbackSample,
    DirectLightSample fallbackSample,
    float3 surfaceNormal,
    uint pixelIndex)
{
    if (CurBounce != 0)
        return;

    if (useDirectLightRIS)
    {
        // Keep history tied to this pixel's local RIS draw only. The replayed
        // current-frame selection stays transient in the shadow payload so the
        // prev-frame neighbor prototype does not recursively replay replayed M.
        if (hasLocalReservoirSample)
        {
            DirectLightReservoirs[pixelIndex] = MakeResolvedDirectLightReservoir(
                localReservoirSample,
                surfaceNormal,
                localReservoirWeightSum,
                localReservoirSampleCount);
        }
        else if (hasFallbackSample)
        {
            StoreInitialDirectLightReservoir(fallbackSample, surfaceNormal, pixelIndex);
        }
        else
        {
            ClearDirectLightReservoir(pixelIndex);
        }
        return;
    }

    if (hasFallbackSample)
    {
        StoreInitialDirectLightReservoir(fallbackSample, surfaceNormal, pixelIndex);
    }
    else
    {
        ClearDirectLightReservoir(pixelIndex);
    }
}

void GenerateShadowRays(RayHit hit, float3 V, float3 throughput, uint pixelIndex)
{
    bool usePrimaryReservoir = CurBounce == 0;
    bool recordPrimaryReservoir = ShouldRecordDirectLightReservoir();
    uint risCount = max((uint)_DirectLightRISCandidateCount, 1u);
    bool useDirectLightRIS = CurBounce == 0 && _UseDirectLightReservoirRIS;
    bool neighborReplayRequested = useDirectLightRIS && _UseDirectLightReservoirNeighborReuse;
    bool useNeighborReuse = neighborReplayRequested && _HasDirectLightReservoirHistory;
    // This is previous-frame neighbor replay only. It is not the paper's
    // temporal reuse pass, and it is not a full same-frame spatial pass.
    float3 surfaceNormal = GetDirectLightSurfaceNormal(hit, V);
    float3 surfaceOrigin = hit.position + surfaceNormal * 1e-5;
    bool recordNeighborReplayDebug = usePrimaryReservoir && IsDirectLightNeighborReplayDebugView();
    bool canWriteNeighborReplayStageDebug = recordNeighborReplayDebug && useNeighborReuse;
    uint neighborStoredReservoirCount = 0u;
    uint neighborSourceReservoirCount = 0u;
    uint neighborCompatibleReservoirCount = 0u;
    uint neighborReevaluatedReservoirCount = 0u;
    bool hasTemporalReusePixelForDiagnostics = false;
    uint temporalReplayStageForDiagnostics = 0u;
    uint temporalReusePixelIndexForDiagnostics = pixelIndex;
    bool usedSamePixelFallbackForDiagnostics = false;
    bool hasSun = _DirectionalLightColor.a > 0.0;
    DirectLightSample firstSample = (DirectLightSample)0;
    uint firstCandidateIndex = 0u;
    bool firstAccepted = false;
    bool firstValid = false;
    float sunFacing = hasSun ? dot(surfaceNormal, normalize(_InverseDirectionalLight)) : 0.0;
    float finalSelectedWeight = 0.0;
    bool validReservoirPayload = false;
    float surfaceMode = hit.mode;
    float surfaceRoughness = hit.material.roughness;
    float surfaceMetallic = hit.material.metallic;
    float3 rawSurfaceAlbedo = hit.material.albedo;
    float3 rawThroughput = throughput;
    float4 rawDirectionalLight = _DirectionalLightColor;
    float surfaceAlbedoLum = GetPositiveDirectLightLuminance(rawSurfaceAlbedo);
    float surfaceNdotV = saturate(dot(surfaceNormal, V));
    float firstBrdfLum = 0.0;
    float firstLightLum = hasSun ? GetPositiveDirectLightLuminance(rawDirectionalLight.rgb * rawDirectionalLight.a) : 0.0;
    float firstThroughputLum = GetPositiveDirectLightLuminance(rawThroughput);
    float4 temporalCompatibilityDiagnosticData = MakeDirectLightShadingDiagnosticData(surfaceNdotV, firstBrdfLum, firstLightLum, firstThroughputLum);

    if (usePrimaryReservoir)
        WriteDirectLightDiagnosticSnapshot(pixelIndex, true);

    if (usePrimaryReservoir && IsDirectLightReservoirStatusDebugView())
        DirectLightDebugOutput[pixelIndex] = float3(0.0, 0.0, 0.08);

    uint pointLightCount;
    uint pointLightOffset;
    bool useCulledList;
    bool hasPointLights = GetPointLightCandidateRange(pointLightCount, pointLightOffset, useCulledList);

    uint candidateCount = (hasSun ? 1u : 0u) + (hasPointLights ? pointLightCount : 0u);
    if (candidateCount == 0u)
    {
        if (usePrimaryReservoir && IsDirectLightReservoirStatusDebugView())
            DirectLightDebugOutput[pixelIndex] = float3(0.08, 0.0, 0.0);
        if (recordNeighborReplayDebug)
        {
            if (!neighborReplayRequested)
                MarkDirectLightNeighborReplayUnavailableDebug(pixelIndex, GetDirectLightNeighborReplayDebugNoReplayColor());
            else if (!_HasDirectLightReservoirHistory)
                MarkDirectLightNeighborReplayUnavailableDebug(pixelIndex, GetDirectLightNeighborReplayDebugWarmupColor());
            else
                MarkDirectLightNeighborReplayUnavailableDebug(pixelIndex, GetDirectLightNeighborReplayDebugNoCandidatesColor());
        }
        if (usePrimaryReservoir)
        {
            WriteDirectLightSamplingDiagnosticSnapshot(
                pixelIndex,
                MakeDirectLightCandidateDiagnosticData((float)candidateCount, hasSun ? 1.0 : 0.0, (float)pointLightCount, (float)firstCandidateIndex),
                MakeDirectLightFirstDrawDiagnosticData(firstAccepted ? 1.0 : 0.0, firstValid ? 1.0 : 0.0, firstSample.proposalPdf, firstSample.targetLum),
                MakeDirectLightFirstMetaDiagnosticData(firstSample.reservoirWeight, firstSample.maxDist, (float)firstSample.lightType, sunFacing),
                MakeDirectLightRISLocalDiagnosticData(0.0, 0.0, 0.0, 0.0),
                MakeDirectLightRISFinalDiagnosticData((float)risCount, (float)risCount, finalSelectedWeight, validReservoirPayload ? 1.0 : 0.0),
                MakeDirectLightMaterialDiagnosticData(surfaceMode, surfaceRoughness, surfaceMetallic, surfaceAlbedoLum),
                MakeDirectLightShadingDiagnosticData(surfaceNdotV, firstBrdfLum, firstLightLum, firstThroughputLum),
                MakeDirectLightRawFloat3DiagnosticData(rawSurfaceAlbedo),
                MakeDirectLightRawFloat3DiagnosticData(rawThroughput),
                rawDirectionalLight,
                MakeDirectLightSampleScalarDiagnosticData(firstSample),
                MakeDirectLightRawFloat3DiagnosticData(firstSample.contribution),
                MakeDirectLightSampleScalarDiagnosticData(firstSample),
                MakeDirectLightRawFloat3DiagnosticData(firstSample.contribution),
                MakeDirectLightNeighborReplayCountDiagnosticData((float)neighborStoredReservoirCount, (float)neighborSourceReservoirCount, (float)neighborCompatibleReservoirCount, (float)neighborReevaluatedReservoirCount),
                MakeDirectLightNeighborReplayStateDiagnosticData(neighborReplayRequested ? 1.0 : 0.0, useNeighborReuse ? 1.0 : 0.0, hasTemporalReusePixelForDiagnostics ? 1.0 : 0.0, (float)temporalReplayStageForDiagnostics),
                MakeDirectLightTemporalReplayIndexDiagnosticData((float)pixelIndex, (float)temporalReusePixelIndexForDiagnostics, temporalReusePixelIndexForDiagnostics == pixelIndex ? 1.0 : 0.0, usedSamePixelFallbackForDiagnostics ? 1.0 : 0.0));
        }
        return;
    }

    float proposalPdf = 1.0 / (float)candidateCount;
    bool useSeparatedSunPointBudget = useDirectLightRIS && hasSun && hasPointLights && risCount > 1u;
    float pointOnlyProposalPdf = pointLightCount > 0u ? (1.0 / (float)pointLightCount) : 0.0;
    float u = RNG_Next(rng);
    uint candidateIndex = 0u;
    if (useSeparatedSunPointBudget)
    {
        candidateIndex = 0u;
    }
    else
    {
        candidateIndex = min(uint(u * candidateCount), candidateCount - 1u);
    }
    firstCandidateIndex = candidateIndex;
    DirectLightSample sample = (DirectLightSample)0;
    bool accepted = false;

    if (useSeparatedSunPointBudget)
    {
        accepted = BuildSunDirectLightSample(hit, V, throughput, 1.0, sample);
    }
    else
    {
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
    }
    firstSample = sample;
    firstAccepted = accepted;
    firstValid = accepted && IsValidDirectLightSample(sample);
    if (accepted && firstSample.lightType == 1u && firstLightLum > 0.0 && sunFacing > 0.0 && firstThroughputLum > 0.0)
    {
        float denom = firstLightLum * sunFacing * firstThroughputLum;
        if (denom > 1e-6)
            firstBrdfLum = firstSample.targetLum / denom;
    }

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
            if (recordNeighborReplayDebug)
                MarkDirectLightNeighborReplayUnavailableDebug(pixelIndex, GetDirectLightNeighborReplayDebugNoReplayColor());
            if (usePrimaryReservoir)
            {
                WriteDirectLightSamplingDiagnosticSnapshot(
                    pixelIndex,
                    MakeDirectLightCandidateDiagnosticData((float)candidateCount, hasSun ? 1.0 : 0.0, (float)pointLightCount, (float)firstCandidateIndex),
                    MakeDirectLightFirstDrawDiagnosticData(firstAccepted ? 1.0 : 0.0, firstValid ? 1.0 : 0.0, firstSample.proposalPdf, firstSample.targetLum),
                    MakeDirectLightFirstMetaDiagnosticData(firstSample.reservoirWeight, firstSample.maxDist, (float)firstSample.lightType, sunFacing),
                    MakeDirectLightRISLocalDiagnosticData(0.0, 0.0, 0.0, 0.0),
                    MakeDirectLightRISFinalDiagnosticData((float)risCount, (float)risCount, finalSelectedWeight, validReservoirPayload ? 1.0 : 0.0),
                    MakeDirectLightMaterialDiagnosticData(surfaceMode, surfaceRoughness, surfaceMetallic, surfaceAlbedoLum),
                    temporalCompatibilityDiagnosticData,
                    MakeDirectLightRawFloat3DiagnosticData(rawSurfaceAlbedo),
                    MakeDirectLightRawFloat3DiagnosticData(rawThroughput),
                    rawDirectionalLight,
                    MakeDirectLightSampleScalarDiagnosticData(firstSample),
                    MakeDirectLightRawFloat3DiagnosticData(firstSample.contribution),
                    MakeDirectLightSampleScalarDiagnosticData(firstSample),
                    MakeDirectLightRawFloat3DiagnosticData(firstSample.contribution),
                    MakeDirectLightNeighborReplayCountDiagnosticData((float)neighborStoredReservoirCount, (float)neighborSourceReservoirCount, (float)neighborCompatibleReservoirCount, (float)neighborReevaluatedReservoirCount),
                    MakeDirectLightNeighborReplayStateDiagnosticData(neighborReplayRequested ? 1.0 : 0.0, useNeighborReuse ? 1.0 : 0.0, hasTemporalReusePixelForDiagnostics ? 1.0 : 0.0, (float)temporalReplayStageForDiagnostics),
                    MakeDirectLightTemporalReplayIndexDiagnosticData((float)pixelIndex, (float)temporalReusePixelIndexForDiagnostics, temporalReusePixelIndexForDiagnostics == pixelIndex ? 1.0 : 0.0, usedSamePixelFallbackForDiagnostics ? 1.0 : 0.0));
            }
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
        if (recordNeighborReplayDebug)
            MarkDirectLightNeighborReplayUnavailableDebug(pixelIndex, GetDirectLightNeighborReplayDebugNoReplayColor());

        if (usePrimaryReservoir)
        {
            WriteDirectLightSamplingDiagnosticSnapshot(
                pixelIndex,
                MakeDirectLightCandidateDiagnosticData((float)candidateCount, hasSun ? 1.0 : 0.0, (float)pointLightCount, (float)firstCandidateIndex),
                MakeDirectLightFirstDrawDiagnosticData(firstAccepted ? 1.0 : 0.0, firstValid ? 1.0 : 0.0, firstSample.proposalPdf, firstSample.targetLum),
                MakeDirectLightFirstMetaDiagnosticData(firstSample.reservoirWeight, firstSample.maxDist, (float)firstSample.lightType, sunFacing),
                MakeDirectLightRISLocalDiagnosticData(1.0, 1.0, sample.reservoirWeight, sample.reservoirWeight),
                MakeDirectLightRISFinalDiagnosticData(1.0, 1.0, GetDirectLightResamplingWeight(sample.reservoirWeight, sample.targetLum, 1u), 1.0),
                MakeDirectLightMaterialDiagnosticData(surfaceMode, surfaceRoughness, surfaceMetallic, surfaceAlbedoLum),
                temporalCompatibilityDiagnosticData,
                MakeDirectLightRawFloat3DiagnosticData(rawSurfaceAlbedo),
                MakeDirectLightRawFloat3DiagnosticData(rawThroughput),
                rawDirectionalLight,
                MakeDirectLightSampleScalarDiagnosticData(firstSample),
                MakeDirectLightRawFloat3DiagnosticData(firstSample.contribution),
                MakeDirectLightSampleScalarDiagnosticData(firstSample),
                MakeDirectLightRawFloat3DiagnosticData(firstSample.contribution),
                MakeDirectLightNeighborReplayCountDiagnosticData((float)neighborStoredReservoirCount, (float)neighborSourceReservoirCount, (float)neighborCompatibleReservoirCount, (float)neighborReevaluatedReservoirCount),
                MakeDirectLightNeighborReplayStateDiagnosticData(neighborReplayRequested ? 1.0 : 0.0, useNeighborReuse ? 1.0 : 0.0, hasTemporalReusePixelForDiagnostics ? 1.0 : 0.0, (float)temporalReplayStageForDiagnostics),
                MakeDirectLightTemporalReplayIndexDiagnosticData((float)pixelIndex, (float)temporalReusePixelIndexForDiagnostics, temporalReusePixelIndexForDiagnostics == pixelIndex ? 1.0 : 0.0, usedSamePixelFallbackForDiagnostics ? 1.0 : 0.0));
        }
        rng.state = pathRngStateAfterDirectLight;
        QueueDirectLightSample(sample, pixelIndex);
        return;
    }

    uint localRepresentedSampleCount = risCount + (useSeparatedSunPointBudget ? 1u : 0u);
    DirectLightSample selectedSample = (DirectLightSample)0;
    bool hasSelectedSample = false;
    bool selectedLastCandidate = false;
    float weightSum = 0.0;
    uint representedSampleCount = localRepresentedSampleCount;
    DirectLightSample localSelectedSample = (DirectLightSample)0;
    bool hasLocalSelectedSample = false;
    float localWeightSum = 0.0;

    if (accepted && IsValidDirectLightSample(sample))
    {
        selectedLastCandidate = UpdateDirectLightRISSelection(
            selectedSample,
            weightSum,
            hasSelectedSample,
            sample);
        localSelectedSample = selectedSample;
        hasLocalSelectedSample = hasSelectedSample;
        localWeightSum = weightSum;
    }

    uint additionalLocalDrawCount = useSeparatedSunPointBudget ? risCount : (risCount > 0u ? (risCount - 1u) : 0u);
    for (uint i = 0u; i < additionalLocalDrawCount; ++i)
    {
        sample = (DirectLightSample)0;
        accepted = false;
        if (useSeparatedSunPointBudget)
        {
            u = RNG_Next(rng);
            uint pointCandidateIndex = min(uint(u * pointLightCount), pointLightCount - 1u);
            candidateIndex = pointCandidateIndex + 1u;
            accepted = SampleDirectLightCandidate(
                hit,
                V,
                throughput,
                hasSun,
                pointLightOffset,
                useCulledList,
                candidateIndex,
                pointOnlyProposalPdf,
                sample);
        }
        else
        {
            u = RNG_Next(rng);
            candidateIndex = min(uint(u * candidateCount), candidateCount - 1u);
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
        }

        UpdateDirectLightFallbackSample(
            accepted,
            sample,
            fallbackSample,
            hasAcceptedFallbackSample,
            hasFallbackSample);

        if (!accepted || !IsValidDirectLightSample(sample))
            continue;

        selectedLastCandidate = UpdateDirectLightRISSelection(
            selectedSample,
            weightSum,
            hasSelectedSample,
            sample);
        if (hasSelectedSample)
        {
            localSelectedSample = selectedSample;
            hasLocalSelectedSample = true;
            localWeightSum = weightSum;
        }
    }

    if (useNeighborReuse)
    {
        uint temporalReusePixelIndex = 0u;
        bool hasTemporalReusePixel = TryGetDirectLightTemporalReprojectionPixelIndex(surfaceOrigin, temporalReusePixelIndex);
        uint temporalReplayStage = 0u;
        hasTemporalReusePixelForDiagnostics = hasTemporalReusePixel;
        temporalReusePixelIndexForDiagnostics = hasTemporalReusePixel ? temporalReusePixelIndex : pixelIndex;
        if (hasTemporalReusePixel)
        {
            DirectLightReservoirData prevReservoir = DirectLightReservoirsPrev[temporalReusePixelIndex];
            HitData prevSurfaceHistory = PrimarySurfaceHistoryPrev[temporalReusePixelIndex];
            if (HasStoredNeighborReuseReservoir(prevReservoir))
            {
                neighborStoredReservoirCount += 1u;
                temporalReplayStage = 1u;
            }
            if (_HasPrimarySurfaceHistory && IsValidNeighborReuseReservoirSource(prevReservoir))
            {
                neighborSourceReservoirCount += 1u;
                temporalReplayStage = 2u;
                temporalCompatibilityDiagnosticData = GetDirectLightTemporalCompatibilityDiagnosticData(surfaceOrigin, surfaceNormal, hit.mode, prevSurfaceHistory, prevReservoir);
                if (IsDirectLightTemporalReuseCompatible(surfaceOrigin, surfaceNormal, hit.mode, prevSurfaceHistory, prevReservoir))
                {
                    neighborCompatibleReservoirCount += 1u;
                    temporalReplayStage = 3u;
                    representedSampleCount += GetNeighborReuseSourceSampleCount(prevReservoir);

                    DirectLightSample candidate = (DirectLightSample)0;
                    if (BuildNeighborReusedDirectLightSample(hit, V, throughput, temporalReusePixelIndex, prevReservoir, candidate))
                    {
                        neighborReevaluatedReservoirCount += 1u;
                        temporalReplayStage = 4u;
                        selectedLastCandidate = UpdateDirectLightRISSelection(
                            selectedSample,
                            weightSum,
                            hasSelectedSample,
                            candidate);
                    }
                }
            }
        }
        temporalReplayStageForDiagnostics = temporalReplayStage;

        if (!hasTemporalReusePixel || (temporalReplayStage < 3u && temporalReusePixelIndex != pixelIndex))
        {
            // This project still lacks the reference pipeline's full motion-vector /
            // temporal reprojection chain. If the clip-space lookup drifts to an
            // unrelated pixel, fall back to the previous frame's same screen pixel
            // before giving up on temporal history entirely.
            DirectLightReservoirData prevReservoir = DirectLightReservoirsPrev[pixelIndex];
            HitData prevSurfaceHistory = PrimarySurfaceHistoryPrev[pixelIndex];
            bool attemptedSamePixelFallback = true;
            if (HasStoredNeighborReuseReservoir(prevReservoir))
            {
                neighborStoredReservoirCount += 1u;
                temporalReplayStageForDiagnostics = max(temporalReplayStageForDiagnostics, 5u);
            }
            if (_HasPrimarySurfaceHistory && IsValidNeighborReuseReservoirSource(prevReservoir))
            {
                neighborSourceReservoirCount += 1u;
                temporalReplayStageForDiagnostics = max(temporalReplayStageForDiagnostics, 6u);
                temporalCompatibilityDiagnosticData = GetDirectLightTemporalCompatibilityDiagnosticData(surfaceOrigin, surfaceNormal, hit.mode, prevSurfaceHistory, prevReservoir);
                if (IsDirectLightTemporalReuseCompatible(surfaceOrigin, surfaceNormal, hit.mode, prevSurfaceHistory, prevReservoir))
                {
                    neighborCompatibleReservoirCount += 1u;
                    temporalReplayStageForDiagnostics = max(temporalReplayStageForDiagnostics, 7u);
                    representedSampleCount += GetNeighborReuseSourceSampleCount(prevReservoir);

                    DirectLightSample candidate = (DirectLightSample)0;
                    if (BuildNeighborReusedDirectLightSample(hit, V, throughput, pixelIndex, prevReservoir, candidate))
                    {
                        neighborReevaluatedReservoirCount += 1u;
                        temporalReplayStageForDiagnostics = max(temporalReplayStageForDiagnostics, 8u);
                        usedSamePixelFallbackForDiagnostics = attemptedSamePixelFallback;
                        selectedLastCandidate = UpdateDirectLightRISSelection(
                            selectedSample,
                            weightSum,
                            hasSelectedSample,
                            candidate);
                    }
                }
            }
        }

        uint neighborReuseCount = min((uint)max(_DirectLightNeighborReuseCount, 1), 8u);
        uint neighborBase = (uint)_FrameCount;
        for (uint i = 0u; i < neighborReuseCount; ++i)
        {
            uint neighborPixelIndex;
            if (!TryGetDirectLightNeighborPixelIndex(pixelIndex, neighborBase + i, neighborPixelIndex))
                continue;

            DirectLightReservoirData prevReservoir = DirectLightReservoirsPrev[neighborPixelIndex];
            if (HasStoredNeighborReuseReservoir(prevReservoir))
                neighborStoredReservoirCount += 1u;
            if (!IsValidNeighborReuseReservoirSource(prevReservoir))
                continue;
            neighborSourceReservoirCount += 1u;
            if (!IsDirectLightNeighborReuseCompatible(surfaceOrigin, surfaceNormal, prevReservoir))
                continue;
            neighborCompatibleReservoirCount += 1u;

            representedSampleCount += GetNeighborReuseSourceSampleCount(prevReservoir);
            DirectLightSample candidate = (DirectLightSample)0;
            if (!BuildNeighborReusedDirectLightSample(hit, V, throughput, neighborPixelIndex, prevReservoir, candidate))
                continue;
            neighborReevaluatedReservoirCount += 1u;

            selectedLastCandidate = UpdateDirectLightRISSelection(
                selectedSample,
                weightSum,
                hasSelectedSample,
                candidate);
        }
    }

    DirectLightSample diagnosticSample = firstSample;
    if (hasSelectedSample)
        diagnosticSample = selectedSample;

    if (!hasSelectedSample)
    {
        if (hasAcceptedFallbackSample)
        {
            if (usePrimaryReservoir && recordPrimaryReservoir)
            {
                StoreDirectLightReservoirHistory(
                    useDirectLightRIS,
                    hasLocalSelectedSample,
                    localSelectedSample,
                    localWeightSum,
                    localRepresentedSampleCount,
                    hasFallbackSample,
                    fallbackSample,
                    surfaceNormal,
                    pixelIndex);
                WriteDirectLightDiagnosticSnapshot(pixelIndex, true);
            }
            if (canWriteNeighborReplayStageDebug)
            {
                if (!hasLocalSelectedSample)
                    MarkDirectLightNeighborReplayUnavailableDebug(pixelIndex, GetDirectLightNeighborReplayDebugNoCurrentSampleColor());
                else
                    MarkDirectLightNeighborReplayDebug(pixelIndex, neighborStoredReservoirCount, neighborSourceReservoirCount, neighborCompatibleReservoirCount, neighborReevaluatedReservoirCount);
            }
            else if (recordNeighborReplayDebug)
                MarkDirectLightNeighborReplayUnavailableDebug(pixelIndex, neighborReplayRequested ? GetDirectLightNeighborReplayDebugWarmupColor() : GetDirectLightNeighborReplayDebugNoReplayColor());
            if (usePrimaryReservoir)
            {
                WriteDirectLightSamplingDiagnosticSnapshot(
                    pixelIndex,
                    MakeDirectLightCandidateDiagnosticData((float)candidateCount, hasSun ? 1.0 : 0.0, (float)pointLightCount, (float)firstCandidateIndex),
                    MakeDirectLightFirstDrawDiagnosticData(firstAccepted ? 1.0 : 0.0, firstValid ? 1.0 : 0.0, firstSample.proposalPdf, firstSample.targetLum),
                    MakeDirectLightFirstMetaDiagnosticData(firstSample.reservoirWeight, firstSample.maxDist, (float)firstSample.lightType, sunFacing),
                    MakeDirectLightRISLocalDiagnosticData(hasSelectedSample ? 1.0 : 0.0, hasLocalSelectedSample ? 1.0 : 0.0, weightSum, localWeightSum),
                    MakeDirectLightRISFinalDiagnosticData((float)representedSampleCount, (float)localRepresentedSampleCount, finalSelectedWeight, validReservoirPayload ? 1.0 : 0.0),
                    MakeDirectLightMaterialDiagnosticData(surfaceMode, surfaceRoughness, surfaceMetallic, surfaceAlbedoLum),
                    temporalCompatibilityDiagnosticData,
                    MakeDirectLightRawFloat3DiagnosticData(rawSurfaceAlbedo),
                    MakeDirectLightRawFloat3DiagnosticData(rawThroughput),
                    rawDirectionalLight,
                    MakeDirectLightSampleScalarDiagnosticData(firstSample),
                    MakeDirectLightRawFloat3DiagnosticData(firstSample.contribution),
                    MakeDirectLightSampleScalarDiagnosticData(diagnosticSample),
                    MakeDirectLightRawFloat3DiagnosticData(diagnosticSample.contribution),
                    MakeDirectLightNeighborReplayCountDiagnosticData((float)neighborStoredReservoirCount, (float)neighborSourceReservoirCount, (float)neighborCompatibleReservoirCount, (float)neighborReevaluatedReservoirCount),
                    MakeDirectLightNeighborReplayStateDiagnosticData(neighborReplayRequested ? 1.0 : 0.0, useNeighborReuse ? 1.0 : 0.0, hasTemporalReusePixelForDiagnostics ? 1.0 : 0.0, (float)temporalReplayStageForDiagnostics),
                    MakeDirectLightTemporalReplayIndexDiagnosticData((float)pixelIndex, (float)temporalReusePixelIndexForDiagnostics, temporalReusePixelIndexForDiagnostics == pixelIndex ? 1.0 : 0.0, usedSamePixelFallbackForDiagnostics ? 1.0 : 0.0));
            }
            rng.state = pathRngStateAfterDirectLight;
            QueueDirectLightSample(fallbackSample, pixelIndex);
            return;
        }

        if (usePrimaryReservoir && IsDirectLightReservoirStatusDebugView())
            DirectLightDebugOutput[pixelIndex] = float3(1.0, 0.0, 1.0);
        if (canWriteNeighborReplayStageDebug)
        {
            if (!hasLocalSelectedSample)
                MarkDirectLightNeighborReplayUnavailableDebug(pixelIndex, GetDirectLightNeighborReplayDebugNoCurrentSampleColor());
            else
                MarkDirectLightNeighborReplayDebug(pixelIndex, neighborStoredReservoirCount, neighborSourceReservoirCount, neighborCompatibleReservoirCount, neighborReevaluatedReservoirCount);
        }
        else if (recordNeighborReplayDebug)
            MarkDirectLightNeighborReplayUnavailableDebug(pixelIndex, neighborReplayRequested ? GetDirectLightNeighborReplayDebugWarmupColor() : GetDirectLightNeighborReplayDebugNoReplayColor());
        if (usePrimaryReservoir)
        {
            WriteDirectLightSamplingDiagnosticSnapshot(
                pixelIndex,
                MakeDirectLightCandidateDiagnosticData((float)candidateCount, hasSun ? 1.0 : 0.0, (float)pointLightCount, (float)firstCandidateIndex),
                MakeDirectLightFirstDrawDiagnosticData(firstAccepted ? 1.0 : 0.0, firstValid ? 1.0 : 0.0, firstSample.proposalPdf, firstSample.targetLum),
                MakeDirectLightFirstMetaDiagnosticData(firstSample.reservoirWeight, firstSample.maxDist, (float)firstSample.lightType, sunFacing),
                MakeDirectLightRISLocalDiagnosticData(hasSelectedSample ? 1.0 : 0.0, hasLocalSelectedSample ? 1.0 : 0.0, weightSum, localWeightSum),
                MakeDirectLightRISFinalDiagnosticData((float)representedSampleCount, (float)localRepresentedSampleCount, finalSelectedWeight, validReservoirPayload ? 1.0 : 0.0),
                MakeDirectLightMaterialDiagnosticData(surfaceMode, surfaceRoughness, surfaceMetallic, surfaceAlbedoLum),
                temporalCompatibilityDiagnosticData,
                MakeDirectLightRawFloat3DiagnosticData(rawSurfaceAlbedo),
                MakeDirectLightRawFloat3DiagnosticData(rawThroughput),
                rawDirectionalLight,
                MakeDirectLightSampleScalarDiagnosticData(firstSample),
                MakeDirectLightRawFloat3DiagnosticData(firstSample.contribution),
                MakeDirectLightSampleScalarDiagnosticData(diagnosticSample),
                MakeDirectLightRawFloat3DiagnosticData(diagnosticSample.contribution),
                MakeDirectLightNeighborReplayCountDiagnosticData((float)neighborStoredReservoirCount, (float)neighborSourceReservoirCount, (float)neighborCompatibleReservoirCount, (float)neighborReevaluatedReservoirCount),
                MakeDirectLightNeighborReplayStateDiagnosticData(neighborReplayRequested ? 1.0 : 0.0, useNeighborReuse ? 1.0 : 0.0, hasTemporalReusePixelForDiagnostics ? 1.0 : 0.0, (float)temporalReplayStageForDiagnostics),
                MakeDirectLightTemporalReplayIndexDiagnosticData((float)pixelIndex, (float)temporalReusePixelIndexForDiagnostics, temporalReusePixelIndexForDiagnostics == pixelIndex ? 1.0 : 0.0, usedSamePixelFallbackForDiagnostics ? 1.0 : 0.0));
        }
        rng.state = pathRngStateAfterDirectLight;
        return;
    }

    float selectedWeight = GetDirectLightResamplingWeight(
        weightSum,
        selectedSample.targetLum,
        representedSampleCount);
    finalSelectedWeight = selectedWeight;
    if (usePrimaryReservoir && recordPrimaryReservoir)
    {
        StoreDirectLightReservoirHistory(
            useDirectLightRIS,
            hasLocalSelectedSample,
            localSelectedSample,
            localWeightSum,
            localRepresentedSampleCount,
            hasFallbackSample,
            fallbackSample,
            surfaceNormal,
            pixelIndex);
        WriteDirectLightDiagnosticSnapshot(pixelIndex, true);
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
    validReservoirPayload =
        all(isfinite(risIllumination)) &&
        selectedWeight > 0.0 &&
        GetPositiveDirectLightLuminance(risIllumination) > 0.0;

    if (!validReservoirPayload)
    {
        if (hasAcceptedFallbackSample)
        {
            if (usePrimaryReservoir && recordPrimaryReservoir)
            {
                StoreDirectLightReservoirHistory(
                    useDirectLightRIS,
                    hasLocalSelectedSample,
                    localSelectedSample,
                    localWeightSum,
                    localRepresentedSampleCount,
                    hasFallbackSample,
                    fallbackSample,
                    surfaceNormal,
                    pixelIndex);
                WriteDirectLightDiagnosticSnapshot(pixelIndex, true);
            }
            if (canWriteNeighborReplayStageDebug)
                MarkDirectLightNeighborReplayDebug(pixelIndex, neighborStoredReservoirCount, neighborSourceReservoirCount, neighborCompatibleReservoirCount, neighborReevaluatedReservoirCount);
            else if (recordNeighborReplayDebug)
                MarkDirectLightNeighborReplayUnavailableDebug(pixelIndex, neighborReplayRequested ? GetDirectLightNeighborReplayDebugWarmupColor() : GetDirectLightNeighborReplayDebugNoReplayColor());
            if (usePrimaryReservoir)
            {
                WriteDirectLightSamplingDiagnosticSnapshot(
                    pixelIndex,
                    MakeDirectLightCandidateDiagnosticData((float)candidateCount, hasSun ? 1.0 : 0.0, (float)pointLightCount, (float)firstCandidateIndex),
                    MakeDirectLightFirstDrawDiagnosticData(firstAccepted ? 1.0 : 0.0, firstValid ? 1.0 : 0.0, firstSample.proposalPdf, firstSample.targetLum),
                    MakeDirectLightFirstMetaDiagnosticData(firstSample.reservoirWeight, firstSample.maxDist, (float)firstSample.lightType, sunFacing),
                    MakeDirectLightRISLocalDiagnosticData(hasSelectedSample ? 1.0 : 0.0, hasLocalSelectedSample ? 1.0 : 0.0, weightSum, localWeightSum),
                    MakeDirectLightRISFinalDiagnosticData((float)representedSampleCount, (float)localRepresentedSampleCount, finalSelectedWeight, validReservoirPayload ? 1.0 : 0.0),
                    MakeDirectLightMaterialDiagnosticData(surfaceMode, surfaceRoughness, surfaceMetallic, surfaceAlbedoLum),
                    temporalCompatibilityDiagnosticData,
                    MakeDirectLightRawFloat3DiagnosticData(rawSurfaceAlbedo),
                    MakeDirectLightRawFloat3DiagnosticData(rawThroughput),
                    rawDirectionalLight,
                    MakeDirectLightSampleScalarDiagnosticData(firstSample),
                    MakeDirectLightRawFloat3DiagnosticData(firstSample.contribution),
                    MakeDirectLightSampleScalarDiagnosticData(diagnosticSample),
                    MakeDirectLightRawFloat3DiagnosticData(diagnosticSample.contribution),
                    MakeDirectLightNeighborReplayCountDiagnosticData((float)neighborStoredReservoirCount, (float)neighborSourceReservoirCount, (float)neighborCompatibleReservoirCount, (float)neighborReevaluatedReservoirCount),
                    MakeDirectLightNeighborReplayStateDiagnosticData(neighborReplayRequested ? 1.0 : 0.0, useNeighborReuse ? 1.0 : 0.0, hasTemporalReusePixelForDiagnostics ? 1.0 : 0.0, (float)temporalReplayStageForDiagnostics),
                    MakeDirectLightTemporalReplayIndexDiagnosticData((float)pixelIndex, (float)temporalReusePixelIndexForDiagnostics, temporalReusePixelIndexForDiagnostics == pixelIndex ? 1.0 : 0.0, usedSamePixelFallbackForDiagnostics ? 1.0 : 0.0));
            }
            rng.state = pathRngStateAfterDirectLight;
            QueueDirectLightSample(fallbackSample, pixelIndex);
            return;
        }

        if (canWriteNeighborReplayStageDebug)
            MarkDirectLightNeighborReplayDebug(pixelIndex, neighborStoredReservoirCount, neighborSourceReservoirCount, neighborCompatibleReservoirCount, neighborReevaluatedReservoirCount);
        else if (recordNeighborReplayDebug)
            MarkDirectLightNeighborReplayUnavailableDebug(pixelIndex, neighborReplayRequested ? GetDirectLightNeighborReplayDebugWarmupColor() : GetDirectLightNeighborReplayDebugNoReplayColor());
        if (usePrimaryReservoir)
        {
            WriteDirectLightSamplingDiagnosticSnapshot(
                pixelIndex,
                MakeDirectLightCandidateDiagnosticData((float)candidateCount, hasSun ? 1.0 : 0.0, (float)pointLightCount, (float)firstCandidateIndex),
                MakeDirectLightFirstDrawDiagnosticData(firstAccepted ? 1.0 : 0.0, firstValid ? 1.0 : 0.0, firstSample.proposalPdf, firstSample.targetLum),
                MakeDirectLightFirstMetaDiagnosticData(firstSample.reservoirWeight, firstSample.maxDist, (float)firstSample.lightType, sunFacing),
                MakeDirectLightRISLocalDiagnosticData(hasSelectedSample ? 1.0 : 0.0, hasLocalSelectedSample ? 1.0 : 0.0, weightSum, localWeightSum),
                MakeDirectLightRISFinalDiagnosticData((float)representedSampleCount, (float)localRepresentedSampleCount, finalSelectedWeight, validReservoirPayload ? 1.0 : 0.0),
                MakeDirectLightMaterialDiagnosticData(surfaceMode, surfaceRoughness, surfaceMetallic, surfaceAlbedoLum),
                temporalCompatibilityDiagnosticData,
                MakeDirectLightRawFloat3DiagnosticData(rawSurfaceAlbedo),
                MakeDirectLightRawFloat3DiagnosticData(rawThroughput),
                rawDirectionalLight,
                MakeDirectLightSampleScalarDiagnosticData(firstSample),
                MakeDirectLightRawFloat3DiagnosticData(firstSample.contribution),
                MakeDirectLightSampleScalarDiagnosticData(diagnosticSample),
                MakeDirectLightRawFloat3DiagnosticData(diagnosticSample.contribution),
                MakeDirectLightNeighborReplayCountDiagnosticData((float)neighborStoredReservoirCount, (float)neighborSourceReservoirCount, (float)neighborCompatibleReservoirCount, (float)neighborReevaluatedReservoirCount),
                MakeDirectLightNeighborReplayStateDiagnosticData(neighborReplayRequested ? 1.0 : 0.0, useNeighborReuse ? 1.0 : 0.0, hasTemporalReusePixelForDiagnostics ? 1.0 : 0.0, (float)temporalReplayStageForDiagnostics),
                MakeDirectLightTemporalReplayIndexDiagnosticData((float)pixelIndex, (float)temporalReusePixelIndexForDiagnostics, temporalReusePixelIndexForDiagnostics == pixelIndex ? 1.0 : 0.0, usedSamePixelFallbackForDiagnostics ? 1.0 : 0.0));
        }
        rng.state = pathRngStateAfterDirectLight;
        return;
    }

    if (canWriteNeighborReplayStageDebug)
        MarkDirectLightNeighborReplayDebug(pixelIndex, neighborStoredReservoirCount, neighborSourceReservoirCount, neighborCompatibleReservoirCount, neighborReevaluatedReservoirCount);
    else if (recordNeighborReplayDebug)
        MarkDirectLightNeighborReplayUnavailableDebug(pixelIndex, neighborReplayRequested ? GetDirectLightNeighborReplayDebugWarmupColor() : GetDirectLightNeighborReplayDebugNoReplayColor());
    if (usePrimaryReservoir)
    {
        WriteDirectLightSamplingDiagnosticSnapshot(
            pixelIndex,
            MakeDirectLightCandidateDiagnosticData((float)candidateCount, hasSun ? 1.0 : 0.0, (float)pointLightCount, (float)firstCandidateIndex),
            MakeDirectLightFirstDrawDiagnosticData(firstAccepted ? 1.0 : 0.0, firstValid ? 1.0 : 0.0, firstSample.proposalPdf, firstSample.targetLum),
            MakeDirectLightFirstMetaDiagnosticData(firstSample.reservoirWeight, firstSample.maxDist, (float)firstSample.lightType, sunFacing),
            MakeDirectLightRISLocalDiagnosticData(hasSelectedSample ? 1.0 : 0.0, hasLocalSelectedSample ? 1.0 : 0.0, weightSum, localWeightSum),
            MakeDirectLightRISFinalDiagnosticData((float)representedSampleCount, (float)localRepresentedSampleCount, finalSelectedWeight, validReservoirPayload ? 1.0 : 0.0),
            MakeDirectLightMaterialDiagnosticData(surfaceMode, surfaceRoughness, surfaceMetallic, surfaceAlbedoLum),
            temporalCompatibilityDiagnosticData,
            MakeDirectLightRawFloat3DiagnosticData(rawSurfaceAlbedo),
            MakeDirectLightRawFloat3DiagnosticData(rawThroughput),
            rawDirectionalLight,
            MakeDirectLightSampleScalarDiagnosticData(firstSample),
            MakeDirectLightRawFloat3DiagnosticData(firstSample.contribution),
            MakeDirectLightSampleScalarDiagnosticData(diagnosticSample),
            MakeDirectLightRawFloat3DiagnosticData(diagnosticSample.contribution),
            MakeDirectLightNeighborReplayCountDiagnosticData((float)neighborStoredReservoirCount, (float)neighborSourceReservoirCount, (float)neighborCompatibleReservoirCount, (float)neighborReevaluatedReservoirCount),
            MakeDirectLightNeighborReplayStateDiagnosticData(neighborReplayRequested ? 1.0 : 0.0, useNeighborReuse ? 1.0 : 0.0, hasTemporalReusePixelForDiagnostics ? 1.0 : 0.0, (float)temporalReplayStageForDiagnostics),
            MakeDirectLightTemporalReplayIndexDiagnosticData((float)pixelIndex, (float)temporalReusePixelIndexForDiagnostics, temporalReusePixelIndexForDiagnostics == pixelIndex ? 1.0 : 0.0, usedSamePixelFallbackForDiagnostics ? 1.0 : 0.0));
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
