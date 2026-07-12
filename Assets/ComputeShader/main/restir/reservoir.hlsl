#pragma once

// Depends on DirectLightReservoirData from global.hlsl

static const uint RESTIR_DI_MAX_RESERVOIR_SAMPLES = 32u;
static const float RESTIR_MATERIAL_ROUGHNESS_THRESHOLD = 0.2;
static const float RESTIR_MATERIAL_METALLIC_THRESHOLD = 0.2;
static const float RESTIR_MATERIAL_ALPHA_THRESHOLD = 0.2;
static const float RESTIR_MATERIAL_IOR_THRESHOLD = 0.25;
static const float RESTIR_MATERIAL_ALBEDO_RELATIVE_THRESHOLD = 0.35;

float ComputeMISWeight(float weightSum, float targetLum, uint sampleCount)
{
    if (targetLum <= 0.0 || sampleCount == 0u) return 0.0;
    return weightSum / (targetLum * (float)sampleCount);
}

float ComputeDirectBiasCorrectedWeight(
    float streamWeightSum,
    float selectedTargetPdf,
    float normalizationNumerator,
    float piSum)
{
    float normalizationDenominator = selectedTargetPdf * piSum;
    return normalizationDenominator > 0.0
        ? streamWeightSum * normalizationNumerator / normalizationDenominator
        : 0.0;
}

bool IsReservoirValid(DirectLightReservoirData r)
{
    return r.targetLum > 0.0 && r.weightSum > 0.0 && r.maxDist > 0.0
        && isfinite(r.weightSum) && isfinite(r.maxDist)
        && r.selectedWeight > 0.0 && isfinite(r.selectedWeight)
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

bool AreRestirMaterialsSimilar(HitData a, HitData b)
{
    if (abs(a.mode - b.mode) > 0.25 ||
        abs(a.roughness - b.roughness) > RESTIR_MATERIAL_ROUGHNESS_THRESHOLD ||
        abs(a.metallic - b.metallic) > RESTIR_MATERIAL_METALLIC_THRESHOLD ||
        abs(a.alpha - b.alpha) > RESTIR_MATERIAL_ALPHA_THRESHOLD ||
        abs(a.ior - b.ior) > RESTIR_MATERIAL_IOR_THRESHOLD)
    {
        return false;
    }

    float albedoReference = max(max(length(a.albedo), length(b.albedo)), 0.1);
    return length(a.albedo - b.albedo) <= RESTIR_MATERIAL_ALBEDO_RELATIVE_THRESHOLD * albedoReference;
}

void WriteDirectReservoirTelemetry(
    uint recordIndex,
    uint stage,
    uint reason,
    uint pixelIndex,
    DirectLightReservoirData reservoir,
    float4 stageData0,
    float4 stageData1)
{
    RestirTelemetryWriteRecord(
        recordIndex,
        stage,
        reason,
        pixelIndex,
        float4(reservoir.origin, reservoir.maxDist),
        float4(reservoir.direction, reservoir.targetLum),
        float4(reservoir.contribution, reservoir.weightSum),
        float4(reservoir.surfaceNormal, reservoir.proposalPdf),
        stageData0,
        stageData1);
}
