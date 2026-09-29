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
    return sample.targetLum > 0.0 && sample.proposalPdf > 0.0
        && isfinite(sample.targetLum) && isfinite(sample.proposalPdf)
        && isfinite(sample.reservoirWeight)
        && all(isfinite(sample.contribution)) && all(isfinite(sample.illumination));
}

void CompleteDirectLightSample(inout DirectLightSample sample, float3 contribution)
{
    sample.contribution = contribution;
    sample.targetLum = GetDirectLightTarget(contribution);
    sample.reservoirWeight = sample.proposalPdf > 0.0
        ? sample.targetLum / sample.proposalPdf : 0.0;
    sample.illumination = sample.proposalPdf > 0.0
        ? contribution / sample.proposalPdf : 0.0;
}

float3 GetDirectLightSurfaceNormal(RayHit hit, float3 V)
{
    float3 N = hit.normal;
    if (dot(V, N) < 0.0)
        N = -N;
    return N;
}

void GetPointLightDiskBasis(float3 lightPosition, float3 receiverPosition,
    out float3 right, out float3 up)
{
    float3 axis = normalize(lightPosition - receiverPosition);
    float3 upReference = abs(axis.y) < 0.99 ? float3(0, 1, 0) : float3(1, 0, 0);
    right = normalize(cross(upReference, axis));
    up = cross(axis, right);
}

// The finite point-light disk faces each receiver. Reuse its two disk coordinates,
// rather than a world-space point on the previous receiver's disk plane.
float3 ReprojectPointLightDiskPosition(PointLightData light,
    float3 oldReceiverPosition, float3 newReceiverPosition, float3 oldSamplePosition)
{
    if (light.sourceRadius <= 1e-4)
        return light.position;

    float3 oldRight, oldUp, newRight, newUp;
    GetPointLightDiskBasis(light.position, oldReceiverPosition, oldRight, oldUp);
    GetPointLightDiskBasis(light.position, newReceiverPosition, newRight, newUp);
    float3 oldOffset = oldSamplePosition - light.position;
    return light.position + dot(oldOffset, oldRight) * newRight
        + dot(oldOffset, oldUp) * newUp;
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

void QueueDirectLightSample(DirectLightSample sample, uint pixelIndex)
{
    EnqueueShadowRay(sample.origin, sample.direction, sample.maxDist, sample.illumination, sample.proposalPdf, pixelIndex);
}

bool SampleDirectLightCandidate(
    RayHit hit,
    float3 V,
    float3 throughput,
    bool hasSun,
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
        uint lightIdx = pointCandidateIndex;
        accepted = BuildPointLightDirectSample(hit, V, throughput, lightIdx, proposalPdf, branchSample);
    }

    if (accepted)
        sample = branchSample;
    return accepted;
}

bool ReevaluateSunDirectLightSample(RayHit hit, float3 V, float3 throughput,
    float3 sampleDir, float proposalPdf, out DirectLightSample sample);
bool ReevaluatePointLightDirectSample(RayHit hit, float3 V, float3 throughput,
    uint lightIdx, float3 samplePos, float proposalPdf, out DirectLightSample sample);

bool BuildSunDirectLightSample(RayHit hit, float3 V, float3 throughput,
    float proposalPdf, out DirectLightSample sample)
{
    float3 direction = normalize(_InverseDirectionalLight);
    if (_SunAngularRadius > 0.0)
    {
        float cosThetaMax = cos(_SunAngularRadius);
        float2 u = RNG_Next2(rng);
        direction = SampleUniformSphericalCap(direction, cosThetaMax, u.x, u.y);
        proposalPdf /= GetSphericalCapSolidAngle(cosThetaMax);
    }
    return ReevaluateSunDirectLightSample(hit, V, throughput, direction, proposalPdf, sample);
}

bool BuildPointLightDirectSample(RayHit hit, float3 V, float3 throughput,
    uint lightIdx, float proposalPdf, out DirectLightSample sample)
{
    PointLightData light = LoadPointLight(lightIdx);
    float3 position = light.position;
    if (light.sourceRadius > 1e-4)
    {
        float3 right, up;
        GetPointLightDiskBasis(light.position, hit.position, right, up);
        float2 u = RNG_Next2(rng);
        float2 disk = SampleDisk(u.x, u.y) * light.sourceRadius;
        position += disk.x * right + disk.y * up;
        proposalPdf /= GetDiskArea(light.sourceRadius);
    }
    return ReevaluatePointLightDirectSample(hit, V, throughput, lightIdx, position, proposalPdf, sample);
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
    if (hit.mode < 3.0) hit.normal = surfaceNormal;
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

    float NdotL = hit.mode >= 3.0 ? abs(dot(surfaceNormal, sample.direction)) : saturate(dot(surfaceNormal, sample.direction));
    sample.origin = hit.position + hit.geometryNormal * (dot(hit.geometryNormal, sample.direction) >= 0.0 ? 1e-5 : -1e-5);
    if (NdotL <= 0.0) return false;

    float3 color = _DirectionalLightColor.rgb * _DirectionalLightColor.a;
    if (_SunAngularRadius > 0.0)
    {
        float solidAngle = GetSphericalCapSolidAngle(cos(_SunAngularRadius));
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
    if (hit.mode < 3.0) hit.normal = surfaceNormal;
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
    if (distS <= 0.0) return false;
    float3 Ls = toSample / distS;
    float NdotL = hit.mode >= 3.0 ? abs(dot(surfaceNormal, Ls)) : saturate(dot(surfaceNormal, Ls));
    sample.origin = hit.position + hit.geometryNormal * (dot(hit.geometryNormal, Ls) >= 0.0 ? 1e-5 : -1e-5);
    if (NdotL <= 0.0) return false;

    float3 f_brdf;
    float dummyPdf;
    EvaluateBXDF_GivenDir(hit, V, Ls, f_brdf, dummyPdf);

    float3 Le = light.color * light.intensity;
    float radius = max(light.sourceRadius, 0.0);
    float3 contribution = throughput * Le * (f_brdf * NdotL);
    if (radius <= 1e-4)
    {
        contribution *= rcp((distS * distS));
    }
    else
    {
        float diskArea = GetDiskArea(radius);
        float3 sampleLe = Le * rcp(diskArea);
        float3 nLight = normalize(hit.position - light.position);
        float cosThetaPrime = saturate(dot(nLight, -Ls));
        if (cosThetaPrime <= 0.0) return false;

        float geom = cosThetaPrime / (distS * distS);
        contribution = throughput * sampleLe * (f_brdf * NdotL) * geom;
    }

    float3 shadowVector = samplePos - sample.origin;
    sample.maxDist = length(shadowVector);
    if (sample.maxDist <= 0.0) return false;
    sample.direction = shadowVector / sample.maxDist;
    CompleteDirectLightSample(sample, contribution * rangeAtten);
    return true;
}

void GenerateShadowRays(RayHit hit, float3 V, float3 throughput, uint pixelIndex)
{
    bool hasSun = _DirectionalLightColor.a > 0.0;

    uint candidateCount = (hasSun ? 1u : 0u) + (uint)max(_PointLightsCount, 0);
    if (candidateCount == 0u) return;

    float proposalPdf = rcp((float)candidateCount);
    float u = RNG_Next(rng);
    uint candidateIndex = min(uint(u * candidateCount), candidateCount - 1u);

    DirectLightSample sample = (DirectLightSample)0;
    bool accepted = SampleDirectLightCandidate(
        hit, V, throughput, hasSun,
        candidateIndex, proposalPdf, sample);

    if (accepted)
        QueueDirectLightSample(sample, pixelIndex);
}
