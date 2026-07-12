#pragma once

bool EvaluateVisibleGISample(
    HitData hd,
    IndirectReservoirData res,
    out float3 weightedReflectedRadiance,
    out float3 reflectedRadiance)
{
    weightedReflectedRadiance = 0.0;
    reflectedRadiance = 0.0;
    if (!EvaluateIndirectSampleAtSurface(hd, res, reflectedRadiance))
        return false;

    RayHit primaryHit = BuildPrimaryRayHit(hd);
    float3 cameraPos = float3(_CameraToWorld._m03, _CameraToWorld._m13, _CameraToWorld._m23);
    float3 V = normalize(cameraPos - hd.position);
    float3 primaryNormal = GetDirectLightSurfaceNormal(primaryHit, V);
    primaryHit.normal = primaryNormal;

    float3 L;
    float distToSecondary;
    if (!ResolveIndirectSampleDirection(hd, res.secondaryPosition, res.sampleFlags, L, distToSecondary))
        return false;

    Ray shadowRay;
    shadowRay.origin = hd.position + primaryNormal * 1e-5;
    shadowRay.dir = L;
    shadowRay.invDir = 1.0 / L;
    float tMax = IsIndirectEnvironmentSample(res.sampleFlags)
        ? distToSecondary
        : distToSecondary * 0.999;
    if (IntersectTlasFast(shadowRay, tMax))
        return false;

    // RTXDI parity: FinalShading.hlsl:66 -> radiance * reservoir.weightSum.
    // After Finalize, weightSum encodes the RIS unbiased contribution weight W/UCW.
    weightedReflectedRadiance = reflectedRadiance * res.weightSum;
    return all(isfinite(weightedReflectedRadiance));
}

[numthreads(64, 1, 1)]
void kernel_shade_gi_samples(uint3 id : SV_DispatchThreadID)
{
    uint pixelCount = _ScreenWidth * _ScreenHeight;
    if (id.x >= pixelCount) return;

    HitData hd = _RestirGbuffer[id.x];
    if (hd.distance >= 1e19)
    {
        RestirTelemetryCount(RESTIR_COUNTER_GI_FINAL_INVALID_PRIMARY, id.x);
        WriteIndirectReservoirTelemetry(
            3u, RESTIR_STAGE_GI_FINAL, RESTIR_REASON_INVALID_SURFACE, id.x,
            EmptyIndirectReservoir(), 0.0);
        return;
    }

    IndirectReservoirData finalRes = IndirectReservoirsRead[_RestirShadingReservoirOffset + id.x];
    bool reservoirWeightFinite = isfinite(finalRes.weightSum);
    bool reservoirWeightExcessive = reservoirWeightFinite && abs(finalRes.weightSum) > 1e20;
    if (!reservoirWeightFinite)
    {
        RestirTelemetryCount(RESTIR_COUNTER_GI_FINAL_NONFINITE_WEIGHT, id.x);
        RestirTelemetryCountCritical(RESTIR_COUNTER_CRITICAL_NONFINITE);
    }
    if (reservoirWeightExcessive)
        RestirTelemetryCount(RESTIR_COUNTER_GI_FINAL_EXCESSIVE_WEIGHT, id.x);

    bool finalReservoirValid = IsIndirectReservoirValid(finalRes);
    if (!finalReservoirValid)
    {
        RestirTelemetryCount(RESTIR_COUNTER_GI_FINAL_INVALID_RESERVOIR, id.x);
        if (id.x == RestirTelemetrySelectedPixel())
            WriteIndirectReservoirTelemetry(
                3u,
                RESTIR_STAGE_GI_FINAL,
                !reservoirWeightFinite
                    ? RESTIR_REASON_NONFINITE_WEIGHT
                    : (reservoirWeightExcessive ? RESTIR_REASON_EXCESSIVE_WEIGHT : RESTIR_REASON_INVALID_SURFACE),
                id.x,
                finalRes,
                float4(0.0, 0.0, 0.0, 0.0));
        return;
    }

    float3 finalWeightedReflectedRadiance;
    float3 finalReflectedRadiance;
    bool finalVisible = EvaluateVisibleGISample(hd, finalRes, finalWeightedReflectedRadiance, finalReflectedRadiance);
    if (!finalVisible)
    {
        RestirTelemetryCount(RESTIR_COUNTER_GI_FINAL_VISIBILITY_REJECTED, id.x);
        if (id.x == RestirTelemetrySelectedPixel())
            WriteIndirectReservoirTelemetry(
                3u,
                RESTIR_STAGE_GI_FINAL,
                RESTIR_REASON_VISIBILITY_REJECTED,
                id.x,
                finalRes,
                float4(0.0, 0.0, 0.0, 0.0));
        return;
    }

    float3 gi = max(finalWeightedReflectedRadiance, 0.0);
    float giLum = max(gi.x, max(gi.y, gi.z));
    if (giLum <= 0.0)
        RestirTelemetryCount(RESTIR_COUNTER_GI_FINAL_ZERO_RADIANCE, id.x);

    float3 rawWeightedRadiance = finalReflectedRadiance * finalRes.weightSum;
    float rawLum = max(rawWeightedRadiance.x, max(rawWeightedRadiance.y, rawWeightedRadiance.z));
    if (rawLum > 1e4)
        RestirTelemetryCount(RESTIR_COUNTER_GI_FINAL_EXCESSIVE_CONTRIBUTION, id.x);

    if (id.x == _RestirDebugPixelIndex)
    {
        ReSTIRDebugData[0] = float4(1.0, 0.0, 1.0, 0.0);
        ReSTIRDebugData[1] = float4(gi, 0.0);
        ReSTIRDebugData[2] = float4(finalReflectedRadiance, 0.0);
        ReSTIRDebugData[3] = float4(
            0.0,
            0.0,
            finalWeightedReflectedRadiance.x,
            finalWeightedReflectedRadiance.y);
        ReSTIRDebugData[4] = float4(
            finalWeightedReflectedRadiance.z,
            0.0,
            0.0,
            0.0);
    }

    if (!all(isfinite(gi)))
    {
        RestirTelemetryCount(RESTIR_COUNTER_GI_FINAL_NONFINITE_CONTRIBUTION, id.x);
        RestirTelemetryCountCritical(RESTIR_COUNTER_CRITICAL_NONFINITE);
        return;
    }
    if (giLum > 0.0)
        RestirTelemetryCount(RESTIR_COUNTER_GI_FINAL_POSITIVE_CONTRIBUTION, id.x);
    if (id.x == RestirTelemetrySelectedPixel())
        WriteIndirectReservoirTelemetry(
            3u,
            RESTIR_STAGE_GI_FINAL,
            RESTIR_REASON_NONE,
            id.x,
            finalRes,
            float4(gi, rawLum));
    GlobalColors[id.x].L += max(gi, 0.0);
}
