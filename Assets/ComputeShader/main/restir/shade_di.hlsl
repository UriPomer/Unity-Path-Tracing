#pragma once

// Depends on: global.hlsl, intersection.hlsl (IntersectTlasFast), reservoir.hlsl

uint _RestirShadingReservoirOffset;

[numthreads(64, 1, 1)]
void kernel_shade_di_samples(uint3 id : SV_DispatchThreadID)
{
    uint pixelCount = _ScreenWidth * _ScreenHeight;
    if (id.x >= pixelCount) return;

    DirectLightReservoirData res = DirectLightReservoirs[_RestirShadingReservoirOffset + id.x];
    if (!IsReservoirValid(res)) return;

    // Inline shadow ray (any-hit)
    Ray shadowRay;
    shadowRay.origin = res.origin;
    shadowRay.dir = res.direction;
    shadowRay.invDir = 1.0 / res.direction;
    float tMax = res.maxDist > 0.0 ? res.maxDist * 0.999 : 1e20;
    if (IntersectTlasFast(shadowRay, tMax)) return;

    // Visible: accumulate weighted contribution
    float3 di = res.contribution * res.selectedWeight;
    if (!all(isfinite(di))) return;
    GlobalColors[id.x].L += max(di, float3(0, 0, 0));
}
