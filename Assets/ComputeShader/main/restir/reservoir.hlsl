#pragma once

// Depends on DirectLightReservoirData from global.hlsl

float ComputeMISWeight(float weightSum, float targetLum, uint sampleCount)
{
    if (targetLum <= 0.0 || sampleCount == 0u) return 0.0;
    return weightSum / (targetLum * (float)sampleCount);
}

bool IsReservoirValid(DirectLightReservoirData r)
{
    return r.targetLum > 0.0 && r.weightSum > 0.0 && r.maxDist > 0.0
        && isfinite(r.weightSum) && isfinite(r.maxDist)
        && all(isfinite(r.origin)) && all(isfinite(r.direction))
        && (r.lightType == 1u || r.lightType == 2u);
}

bool IsTemporalCompatible(
    float3 curPos, float3 curN, float curMode,
    float3 prevPos, float3 prevN, float prevMode)
{
    if (dot(curN, prevN) < 0.9) return false;
    float3 d = prevPos - curPos;
    if (abs(dot(d, curN)) > 0.05 || abs(dot(d, prevN)) > 0.05) return false;
    return abs(curMode - prevMode) <= 0.25;
}
