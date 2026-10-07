#pragma once

// Spatial normalization already evaluated this receiver's selected connection.
// Shade that same contribution and visibility before discarding them.
void ShadeGISample(uint pixelIndex, HitData hd, IndirectReservoirData finalRes, bool finalVisible)
{
    uint3 id = uint3(pixelIndex,0,0);
    if (hd.distance >= 1e19)
    {
        RestirTelemetryCount(RESTIR_COUNTER_GI_FINAL_INVALID_PRIMARY, id.x);
        WriteIndirectReservoirTelemetry(
            3u, RESTIR_STAGE_GI_FINAL, RESTIR_REASON_INVALID_SURFACE, id.x,
            EmptyIndirectReservoir(), 0.0);
        return;
    }

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

    float3 finalReflectedRadiance = finalRes.contribution;
    float3 finalWeightedReflectedRadiance = finalReflectedRadiance * finalRes.weightSum;
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
    if (_DenoiseEnabled)
    {
        RayHit primary = BuildPrimaryRayHit(hd);
        float3 V = normalize(_CameraToWorld._m03_m13_m23-hd.position);
        primary.normal = GetDirectLightSurfaceNormal(primary,V);
        float3 direction; float distance;
        ResolveIndirectSampleDirection(hd,finalRes.secondaryPosition,finalRes.sampleFlags,direction,distance);
        _DenoiseDiffuse[uint2(id.x % _ScreenWidth,id.x / _ScreenWidth)] +=
            float4(gi * OpaqueDiffuseFraction(primary,V,direction),0);
    }
}
