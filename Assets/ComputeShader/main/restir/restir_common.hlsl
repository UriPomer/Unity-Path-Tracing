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

// Shared ReSTIR resources/params used by both DI and GI stages.
StructuredBuffer<HitData> _RestirGbuffer;
StructuredBuffer<HitData> _RestirGbufferPrevious;
StructuredBuffer<LightDataPacked> _RestirLightData;
uint _RestirLightCount;
uint _RestirInitialReservoirOffset;
uint _RestirTemporalReservoirOffset;
uint _RestirPrevReservoirOffset;
uint _RestirShadingReservoirOffset;
uint _RestirSpatialReservoirOffset;
uint _RestirCandidateCount;
float4x4 _RestirPreviousViewProjection;
