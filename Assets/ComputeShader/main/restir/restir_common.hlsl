#pragma once

// Depends on symbols from global.hlsl, trace.hlsl (included by Tracing.compute first)

static const uint RESTIR_RIS_POOL_SIZE = 2048u;

struct LightDataPacked
{
    float3 position;      float  range;
    float3 color;         float  intensity;
    float3 direction;     float  sourceRadius;
    float  power;         float  cdf;
    uint   lightType;     uint   originalIndex;
};
