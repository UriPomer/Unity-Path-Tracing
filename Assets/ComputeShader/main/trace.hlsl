#pragma once

#include "global.hlsl"
#include "bxdf.hlsl"

float3 SampleSkyboxDirection(float3 direction)
{
    float3 dir = normalize(direction);
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

float3 SampleSkybox(Ray ray)
{
    return SampleSkyboxDirection(ray.dir);
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

float GetPositiveDirectLightLuminance(float3 value)
{
    float3 c = max(value, float3(0.0, 0.0, 0.0));
    return c.x * 0.2126 + c.y * 0.7152 + c.z * 0.0722;
}

float GetDirectLightTarget(float3 contribution)
{
    return GetPositiveDirectLightLuminance(contribution);
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
    if (NdotL <= 0.0) return false;

    float3 color = _DirectionalLightColor.rgb * _DirectionalLightColor.a;
    if (_SunAngularRadius > 0.0)
    {
        float solidAngle = max(GetSphericalCapSolidAngle(cos(_SunAngularRadius)), 1e-6);
        color *= rcp(solidAngle);
    }

    float3 f_brdf;
    float dummyPdf;
    EvaluateBXDF_GivenDir(hit, V, sample.direction, f_brdf, dummyPdf);
    CompleteDirectLightSample(sample, throughput * color * f_brdf * NdotL);
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
    if (light.intensity <= 0.0 || light.range <= 0.0) return false;

    float3 toCenter = light.position - hit.position;
    float distC = length(toCenter);
    float rangeAtten = GetPointLightRangeAttenuation(distC, light.range);
    if (rangeAtten <= 0.0) return false;

    float3 toSample = samplePos - hit.position;
    float distS = length(toSample);
    float3 Ls = toSample / max(distS, 1e-6);
    float NdotL = saturate(dot(surfaceNormal, Ls));
    if (NdotL <= 0.0) return false;

    float3 f_brdf;
    float dummyPdf;
    EvaluateBXDF_GivenDir(hit, V, Ls, f_brdf, dummyPdf);

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
        if (cosThetaPrime <= 1e-6) return false;

        float geom = cosThetaPrime / max(distS * distS, 1e-6);
        contribution = throughput * sampleLe * (f_brdf * NdotL) * geom;
    }

    sample.direction = Ls;
    sample.maxDist = distS;
    CompleteDirectLightSample(sample, contribution * rangeAtten);
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
    if (candidateCount == 0u) return;

    float proposalPdf = rcp((float)candidateCount);
    float u = RNG_Next(rng);
    uint candidateIndex = min(uint(u * candidateCount), candidateCount - 1u);

    DirectLightSample sample = (DirectLightSample)0;
    bool accepted = SampleDirectLightCandidate(
        hit, V, throughput, hasSun, pointLightOffset, useCulledList,
        candidateIndex, proposalPdf, sample);

    if (accepted)
        QueueDirectLightSample(sample, pixelIndex);
}
