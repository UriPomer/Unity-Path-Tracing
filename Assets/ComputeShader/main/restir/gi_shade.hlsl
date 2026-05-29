#pragma once

bool EvaluateVisibleGISample(
    HitData hd,
    IndirectReservoirData res,
    out float3 weightedReflectedRadiance)
{
    weightedReflectedRadiance = 0.0;

    float3 reflectedRadiance;
    float3 weightedRadiance;
    float3 trueBrdf;
    if (!EvaluateIndirectSampleAtSurface(hd, res, trueBrdf, weightedRadiance, reflectedRadiance))
        return false;

    RayHit primaryHit = BuildPrimaryRayHit(hd);
    float3 cameraPos = float3(_CameraToWorld._m03, _CameraToWorld._m13, _CameraToWorld._m23);
    float3 V = normalize(cameraPos - hd.position);
    float3 primaryNormal = GetDirectLightSurfaceNormal(primaryHit, V);
    primaryHit.normal = primaryNormal;

    float3 toSecondary = res.secondaryPosition - hd.position;
    float distToSecondary = length(toSecondary);
    if (distToSecondary <= 1e-5)
        return false;

    float3 L = toSecondary / distToSecondary;

    Ray shadowRay;
    shadowRay.origin = hd.position + primaryNormal * 1e-5;
    shadowRay.dir = L;
    shadowRay.invDir = 1.0 / L;
    float tMax = distToSecondary * 0.999;
    if (IntersectTlasFast(shadowRay, tMax))
        return false;

    weightedReflectedRadiance = reflectedRadiance * res.selectedWeight;
    return all(isfinite(weightedReflectedRadiance));
}

[numthreads(64, 1, 1)]
void kernel_shade_gi_samples(uint3 id : SV_DispatchThreadID)
{
    uint pixelCount = _ScreenWidth * _ScreenHeight;
    if (id.x >= pixelCount) return;

    HitData hd = _RestirGbuffer[id.x];
    if (hd.distance >= 1e19) return;

    IndirectReservoirData finalRes = IndirectReservoirsRead[_RestirShadingReservoirOffset + id.x];
    IndirectReservoirData initialRes = IndirectReservoirsRead[_RestirInitialReservoirOffset + id.x];

    float3 finalWeightedReflectedRadiance;
    bool hasFinal = EvaluateVisibleGISample(hd, finalRes, finalWeightedReflectedRadiance);

    float3 initialWeightedReflectedRadiance;
    bool hasInitial = EvaluateVisibleGISample(hd, initialRes, initialWeightedReflectedRadiance);

    if (!hasFinal && !hasInitial)
        return;

    float finalWeight = hasFinal ? 1.0 : 0.0;
    float initialWeight = hasFinal ? 0.0 : (hasInitial ? 1.0 : 0.0);
    float3 gi = hasFinal
        ? max(finalWeightedReflectedRadiance, 0.0)
        : max(initialWeightedReflectedRadiance, 0.0);

    if (id.x == _RestirDebugPixelIndex)
    {
        ReSTIRDebugData[0] = float4(hasFinal ? 1.0 : 0.0, hasInitial ? 1.0 : 0.0, finalWeight, initialWeight);
        ReSTIRDebugData[1] = float4(gi, 0.0);
        ReSTIRDebugData[2] = float4(
            hasFinal ? finalRes.targetLum : 0.0,
            hasInitial ? initialRes.targetLum : 0.0,
            hasFinal ? finalRes.selectedWeight : 0.0,
            hasInitial ? initialRes.selectedWeight : 0.0);
    }

    if (!all(isfinite(gi))) return;
    GlobalColors[id.x].L += max(gi, 0.0);
}
