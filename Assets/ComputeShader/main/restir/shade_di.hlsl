#pragma once

// Depends on: global.hlsl, intersection.hlsl (IntersectTlasFast), reservoir.hlsl

[numthreads(64, 1, 1)]
void kernel_shade_di_samples(uint3 id : SV_DispatchThreadID)
{
    uint pixelCount = _ScreenWidth * _ScreenHeight;
    if (id.x >= pixelCount) return;

    DirectLightReservoirData res = DirectLightReservoirs[_RestirShadingReservoirOffset + id.x];
    if (!IsReservoirValid(res))
    {
        RestirTelemetryCount(RESTIR_COUNTER_DI_SHADE_INVALID_RESERVOIR, id.x);
        WriteDirectReservoirTelemetry(
            6u, RESTIR_STAGE_DI_SHADE, RESTIR_REASON_INVALID_SURFACE, id.x,
            res, 0.0, 0.0);
        return;
    }

    // Inline shadow ray (any-hit)
    Ray shadowRay;
    shadowRay.origin = res.origin;
    shadowRay.dir = res.direction;
    shadowRay.invDir = 1.0 / res.direction;
    float tMax = res.maxDist > 0.0 ? res.maxDist * 0.999 : 1e20;
    if (IntersectTlasFast(shadowRay, tMax))
    {
        RestirTelemetryCount(RESTIR_COUNTER_DI_SHADE_VISIBILITY_REJECTED, id.x);
        WriteDirectReservoirTelemetry(
            6u, RESTIR_STAGE_DI_SHADE, RESTIR_REASON_VISIBILITY_REJECTED, id.x,
            res,
            float4((float)res.lightType, (float)res.lightIndex, (float)res.sampleCount, res.selectedWeight),
            float4(0.0, 0.0, 0.0, tMax));
        return;
    }

    // Visible: accumulate weighted contribution
    float3 di = res.contribution * res.selectedWeight;
    if (!all(isfinite(di)))
    {
        RestirTelemetryCountCritical(RESTIR_COUNTER_CRITICAL_NONFINITE);
        WriteDirectReservoirTelemetry(
            6u, RESTIR_STAGE_DI_SHADE, RESTIR_REASON_NONFINITE_CONTRIBUTION, id.x,
            res,
            float4((float)res.lightType, (float)res.lightIndex, (float)res.sampleCount, res.selectedWeight),
            float4(di, tMax));
        return;
    }
    RestirTelemetryCount(RESTIR_COUNTER_DI_SHADE_POSITIVE_CONTRIBUTION, id.x);
    WriteDirectReservoirTelemetry(
        6u, RESTIR_STAGE_DI_SHADE, RESTIR_REASON_NONE, id.x, res,
        float4((float)res.lightType, (float)res.lightIndex, (float)res.sampleCount, res.selectedWeight),
        float4(di, tMax));
    GlobalColors[id.x].L += max(di, float3(0, 0, 0));
}
