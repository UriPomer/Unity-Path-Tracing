#pragma once

// Depends on: global.hlsl, reservoir.hlsl

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

    float tMax = res.maxDist > 0.0 ? res.maxDist : 1e20;
    // Evaluate current-frame visibility once, after selection. Reservoirs keep
    // unoccluded contributions and targets; alpha transmittance is not squared.
    float visibility = TraceVisibility(GenRay(res.origin,res.direction),tMax,true);
    float3 di = res.contribution * (res.selectedWeight * visibility);
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
    if (any(di > 0.0))
        RestirTelemetryCount(RESTIR_COUNTER_DI_SHADE_POSITIVE_CONTRIBUTION, id.x);
    WriteDirectReservoirTelemetry(
        6u, RESTIR_STAGE_DI_SHADE, RESTIR_REASON_NONE, id.x, res,
        float4((float)res.lightType, (float)res.lightIndex, (float)res.sampleCount, res.selectedWeight),
        float4(di, tMax));
    GlobalColors[id.x].L += max(di, float3(0, 0, 0));
    if (_DenoiseEnabled)
    {
        RayHit primary = BuildPrimaryRayHit(_RestirGbuffer[id.x]);
        float3 V = normalize(_CameraToWorld._m03_m13_m23-primary.position);
        primary.normal = GetDirectLightSurfaceNormal(primary,V);
        float3 diffuse = max(di,0) * OpaqueDiffuseFraction(primary,V,res.direction);
        uint2 pixel = uint2(id.x % _ScreenWidth,id.x / _ScreenWidth);
        _Pixel = pixel;
        float variance = 0;
        if (primary.mode < 2.0 && primary.material.roughness >= 0.1)
        {
            float observation = DirectDiffuseLuminance(primary,V,res.direction,di);
            float delta = observation - IndependentDirectDiffuseObservation(primary,V);
            variance = 0.5 * delta * delta;
        }
        _DenoiseDirect[pixel] = float4(diffuse,variance);
        _DenoiseDiffuse[pixel] += float4(diffuse,0);
    }
}
