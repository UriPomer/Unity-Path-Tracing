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
        return;
    }

    IndirectReservoirData finalRes = IndirectReservoirsRead[_RestirShadingReservoirOffset + id.x];
    IndirectReservoirData initialRes = IndirectReservoirsRead[_RestirInitialReservoirOffset + id.x];
    bool finalReservoirValid = IsIndirectReservoirValid(finalRes);
    bool initialReservoirValid = IsIndirectReservoirValid(initialRes);
    if (!finalReservoirValid && !initialReservoirValid)
        RestirTelemetryCount(RESTIR_COUNTER_GI_FINAL_INVALID_RESERVOIR, id.x);

    bool reservoirWeightFinite = isfinite(finalRes.weightSum) && isfinite(initialRes.weightSum);
    if (!reservoirWeightFinite)
    {
        RestirTelemetryCount(RESTIR_COUNTER_GI_FINAL_NONFINITE_WEIGHT, id.x);
        RestirTelemetryCountCritical(RESTIR_COUNTER_CRITICAL_NONFINITE);
    }
    if (abs(finalRes.weightSum) > 1e6 || abs(initialRes.weightSum) > 1e6)
    {
        RestirTelemetryCount(RESTIR_COUNTER_GI_FINAL_EXCESSIVE_WEIGHT, id.x);
        RestirTelemetryCountCritical(RESTIR_COUNTER_CRITICAL_OUT_OF_RANGE);
    }

    float3 finalWeightedReflectedRadiance;
    float3 finalReflectedRadiance;
    bool hasFinal = EvaluateVisibleGISample(hd, finalRes, finalWeightedReflectedRadiance, finalReflectedRadiance);

    float3 initialWeightedReflectedRadiance;
    float3 initialReflectedRadiance;
    bool hasInitial = EvaluateVisibleGISample(hd, initialRes, initialWeightedReflectedRadiance, initialReflectedRadiance);

    if (!hasFinal && !hasInitial)
    {
        if (finalReservoirValid || initialReservoirValid)
            RestirTelemetryCount(RESTIR_COUNTER_GI_FINAL_VISIBILITY_REJECTED, id.x);
        if (id.x == RestirTelemetrySelectedPixel())
        {
            RestirTelemetryWriteRecord(
                3u,
                RESTIR_STAGE_GI_FINAL,
                finalReservoirValid || initialReservoirValid
                    ? RESTIR_REASON_VISIBILITY_REJECTED
                    : RESTIR_REASON_INVALID_SURFACE,
                id.x,
                float4(finalRes.secondaryPosition, finalRes.proposalPdf),
                float4(finalRes.secondaryNormal, finalRes.targetLum),
                float4(finalRes.radiance, finalRes.weightSum),
                float4(finalRes.contribution, finalRes.selectedWeight),
                float4(finalRes.primaryNormal, finalRes.sampleCount),
                float4(0.0, 0.0, 0.0, 0.0));
        }
        return;
    }

    float finalWeight = hasFinal ? 1.0 : 0.0;
    float initialWeight = hasFinal ? 0.0 : (hasInitial ? 1.0 : 0.0);
    float3 gi = hasFinal
        ? max(finalWeightedReflectedRadiance, 0.0)
        : max(initialWeightedReflectedRadiance, 0.0);
    float giLum = max(gi.x, max(gi.y, gi.z));
    if (giLum <= 0.0)
        RestirTelemetryCount(RESTIR_COUNTER_GI_FINAL_ZERO_RADIANCE, id.x);

    float3 rawWeightedRadiance = hasFinal
        ? finalReflectedRadiance * finalRes.weightSum
        : initialReflectedRadiance * initialRes.weightSum;
    float rawLum = max(rawWeightedRadiance.x, max(rawWeightedRadiance.y, rawWeightedRadiance.z));
    if (rawLum > 1e4)
        RestirTelemetryCount(RESTIR_COUNTER_GI_FINAL_EXCESSIVE_CONTRIBUTION, id.x);

    if (id.x == _RestirDebugPixelIndex)
    {
        ReSTIRDebugData[0] = float4(hasFinal ? 1.0 : 0.0, hasInitial ? 1.0 : 0.0, finalWeight, initialWeight);
        ReSTIRDebugData[1] = float4(gi, 0.0);
        ReSTIRDebugData[2] = float4(
            hasFinal ? finalReflectedRadiance.x : 0.0,
            hasFinal ? finalReflectedRadiance.y : 0.0,
            hasFinal ? finalReflectedRadiance.z : 0.0,
            hasInitial ? initialReflectedRadiance.x : 0.0);
        ReSTIRDebugData[3] = float4(
            hasInitial ? initialReflectedRadiance.y : 0.0,
            hasInitial ? initialReflectedRadiance.z : 0.0,
            hasFinal ? finalWeightedReflectedRadiance.x : 0.0,
            hasFinal ? finalWeightedReflectedRadiance.y : 0.0);
        ReSTIRDebugData[4] = float4(
            hasFinal ? finalWeightedReflectedRadiance.z : 0.0,
            hasInitial ? initialWeightedReflectedRadiance.x : 0.0,
            hasInitial ? initialWeightedReflectedRadiance.y : 0.0,
            hasInitial ? initialWeightedReflectedRadiance.z : 0.0);
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
    {
        IndirectReservoirData selectedRes = initialRes;
        if (hasFinal)
            selectedRes = finalRes;
        RestirTelemetryWriteRecord(
            3u,
            RESTIR_STAGE_GI_FINAL,
            RESTIR_REASON_NONE,
            id.x,
            float4(selectedRes.secondaryPosition, selectedRes.proposalPdf),
            float4(selectedRes.secondaryNormal, selectedRes.targetLum),
            float4(selectedRes.radiance, selectedRes.weightSum),
            float4(selectedRes.contribution, selectedRes.selectedWeight),
            float4(selectedRes.primaryNormal, selectedRes.sampleCount),
            float4(gi, rawLum));
    }
    GlobalColors[id.x].L += max(gi, 0.0);
}
