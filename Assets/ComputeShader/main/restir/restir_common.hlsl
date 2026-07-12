#pragma once

// Depends on symbols from global.hlsl, trace.hlsl (included by Tracing.compute first)

// Shared ReSTIR resources/params used by both DI and GI stages.
StructuredBuffer<HitData> _RestirGbuffer;
StructuredBuffer<HitData> _RestirGbufferPrevious;
uint _RestirInitialReservoirOffset;
uint _RestirTemporalReservoirOffset;
uint _RestirPrevReservoirOffset;
uint _RestirShadingReservoirOffset;
uint _RestirSpatialReservoirOffset;
uint _RestirCandidateCount;
float4x4 _RestirPreviousViewProjection;

bool IsDirectLightSampleVisible(DirectLightSample sample)
{
    Ray shadowRay;
    shadowRay.origin = sample.origin;
    shadowRay.dir = sample.direction;
    shadowRay.invDir = 1.0 / sample.direction;
    float tMax = sample.maxDist > 0.0 ? sample.maxDist * 0.999 : 1e20;
    return !IntersectTlasFast(shadowRay, tMax);
}
