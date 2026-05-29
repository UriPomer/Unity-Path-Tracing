#pragma once

static const float RESTIR_GI_MAX_RESERVOIR_SAMPLES = 32.0;
static const float RESTIR_GI_MAX_JACOBIAN = 3.0;
static const float RESTIR_GI_MIN_JACOBIAN = 1.0 / 3.0;
static const float RESTIR_GI_DISCARD_JACOBIAN = 10.0;
static const float RESTIR_GI_FINITE_LIMIT = 1e30;
static const float RESTIR_GI_MIN_PROPOSAL_PDF = 1e-3;

bool IsFiniteIndirectScalar(float v)
{
    return abs(v) <= RESTIR_GI_FINITE_LIMIT;
}

bool IsFiniteIndirectFloat3(float3 v)
{
    return all(abs(v) <= RESTIR_GI_FINITE_LIMIT);
}

float ComputeIndirectMISWeight(float weightSum, float targetLum, float sampleCount)
{
    if (targetLum <= 0.0 || sampleCount <= 0.0) return 0.0;
    return weightSum / max(targetLum * sampleCount, 1e-6);
}

float ComputeIndirectProposalInversePdf(float proposalPdf)
{
    return proposalPdf > 0.0 ? rcp(max(proposalPdf, RESTIR_GI_MIN_PROPOSAL_PDF)) : 0.0;
}

bool IsIndirectReservoirValid(IndirectReservoirData r)
{
    return r.targetLum > 0.0 && r.weightSum > 0.0 &&
        r.sampleCount > 0.0 && r.sampleCount <= RESTIR_GI_MAX_RESERVOIR_SAMPLES &&
        IsFiniteIndirectFloat3(r.secondaryPosition) &&
        IsFiniteIndirectFloat3(r.secondaryNormal) &&
        IsFiniteIndirectFloat3(r.radiance) &&
        IsFiniteIndirectFloat3(r.contribution) &&
        IsFiniteIndirectScalar(r.proposalPdf) &&
        IsFiniteIndirectScalar(r.selectedWeight) &&
        IsFiniteIndirectScalar(r.sampleCount) &&
        r.proposalPdf > 0.0;
}

RayHit BuildPrimaryRayHit(HitData hd)
{
    RayHit hit;
    hit.position = hd.position;
    hit.distance = hd.distance;
    hit.normal = hd.normal;
    hit.mode = hd.mode;
    hit.material.albedo = hd.albedo;
    hit.material.emission = hd.emission;
    hit.material.emissionIntensity = hd.emissionIntensity;
    hit.material.roughness = hd.roughness;
    hit.material.metallic = hd.metallic;
    hit.material.alpha = hd.alpha;
    hit.material.ior = hd.ior;
    hit.should_break = false;
    return hit;
}

bool EvaluateIndirectRadianceAtSurface(
    HitData hd,
    float3 secondaryPosition,
    float3 sampleRadiance,
    out float3 f_brdf,
    out float3 reflectedRadiance)
{
    f_brdf = 0.0;
    reflectedRadiance = 0.0;

    if (hd.distance >= 1e19 || !IsFiniteIndirectFloat3(secondaryPosition) || !IsFiniteIndirectFloat3(sampleRadiance))
        return false;

    RayHit primaryHit = BuildPrimaryRayHit(hd);
    float3 cameraPos = float3(_CameraToWorld._m03, _CameraToWorld._m13, _CameraToWorld._m23);
    float3 V = normalize(cameraPos - hd.position);
    float3 primaryNormal = GetDirectLightSurfaceNormal(primaryHit, V);
    primaryHit.normal = primaryNormal;

    float3 toSecondary = secondaryPosition - hd.position;
    float distToSecondary = length(toSecondary);
    if (distToSecondary <= 1e-5)
        return false;

    float3 L = toSecondary / distToSecondary;
    float NdotL = saturate(dot(primaryNormal, L));
    if (NdotL <= 0.0)
        return false;

    float proposalPdf;
    EvaluateBXDF_GivenDir(primaryHit, V, L, f_brdf, proposalPdf);
    if (!IsFiniteIndirectFloat3(f_brdf))
        return false;

    float3 radiance = max(sampleRadiance, 0.0);
    float3 reflected = max(f_brdf * NdotL * radiance, 0.0);
    if (!IsFiniteIndirectFloat3(reflected))
        return false;

    reflectedRadiance = reflected;
    return true;
}

bool EvaluateIndirectSampleAtSurface(
    HitData hd,
    IndirectReservoirData sample,
    out float3 f_brdf,
    out float3 weightedRadiance,
    out float3 reflectedRadiance)
{
    f_brdf = 0.0;
    weightedRadiance = 0.0;
    reflectedRadiance = 0.0;

    if (!IsIndirectReservoirValid(sample) || hd.distance >= 1e19)
        return false;

    if (!EvaluateIndirectRadianceAtSurface(hd, sample.secondaryPosition, sample.radiance, f_brdf, reflectedRadiance))
        return false;

    weightedRadiance = sample.radiance * sample.selectedWeight;
    return IsFiniteIndirectFloat3(reflectedRadiance) && IsFiniteIndirectFloat3(weightedRadiance);
}

bool ReevaluateIndirectReservoirAtSurface(
    HitData hd,
    IndirectReservoirData sample,
    out float3 radiance,
    out float3 contribution,
    out float targetLum)
{
    radiance = 0.0;
    contribution = 0.0;
    targetLum = 0.0;

    float3 f_brdf;
    float3 weightedRadiance;
    float3 reflectedRadiance;
    if (!EvaluateIndirectSampleAtSurface(hd, sample, f_brdf, weightedRadiance, reflectedRadiance))
        return false;

    radiance = max(sample.radiance, 0.0);
    contribution = reflectedRadiance;
    targetLum = max(reflectedRadiance.x, max(reflectedRadiance.y, reflectedRadiance.z));
    return targetLum > 0.0 && IsFiniteIndirectFloat3(contribution);
}

bool ReevaluateIndirectReservoirAtSurfaceDebug(
    HitData hd,
    IndirectReservoirData sample,
    out float3 radiance,
    out float3 contribution,
    out float targetLum,
    out uint failureCode)
{
    radiance = 0.0;
    contribution = 0.0;
    targetLum = 0.0;
    failureCode = 0u;

    if (!IsIndirectReservoirValid(sample))
    {
        failureCode = 1u;
        return false;
    }

    if (hd.distance >= 1e19)
    {
        failureCode = 2u;
        return false;
    }

    float3 f_brdf;
    float3 weightedRadiance;
    float3 reflectedRadiance;
    if (!EvaluateIndirectSampleAtSurface(hd, sample, f_brdf, weightedRadiance, reflectedRadiance))
    {
        RayHit primaryHit = BuildPrimaryRayHit(hd);
        float3 cameraPos = float3(_CameraToWorld._m03, _CameraToWorld._m13, _CameraToWorld._m23);
        float3 V = normalize(cameraPos - hd.position);
        float3 primaryNormal = GetDirectLightSurfaceNormal(primaryHit, V);

        float3 toSecondary = sample.secondaryPosition - hd.position;
        float distToSecondary = length(toSecondary);
        if (distToSecondary <= 1e-5)
        {
            failureCode = 3u;
            return false;
        }

        float3 L = toSecondary / distToSecondary;
        float NdotL = saturate(dot(primaryNormal, L));
        if (NdotL <= 0.0)
        {
            failureCode = 5u;
            return false;
        }

        float proposalPdf;
        EvaluateBXDF_GivenDir(primaryHit, V, L, f_brdf, proposalPdf);
        if (!IsFiniteIndirectFloat3(f_brdf))
        {
            failureCode = 4u;
            return false;
        }

        failureCode = 6u;
        return false;
    }

    radiance = max(sample.radiance, 0.0);
    contribution = reflectedRadiance;
    targetLum = max(reflectedRadiance.x, max(reflectedRadiance.y, reflectedRadiance.z));
    if (!(targetLum > 0.0) || !IsFiniteIndirectFloat3(contribution))
    {
        failureCode = 6u;
        return false;
    }

    return true;
}

float CalculateIndirectJacobian(
    float3 receiverPos,
    float3 neighborReceiverPos,
    float3 neighborSamplePos,
    float3 neighborSampleNormal)
{
    float3 toNew = receiverPos - neighborSamplePos;
    float3 toOld = neighborReceiverPos - neighborSamplePos;

    float newDistSqr = dot(toNew, toNew);
    float oldDistSqr = dot(toOld, toOld);
    if (newDistSqr <= 1e-8 || oldDistSqr <= 1e-8)
        return 0.0;

    float newCos = saturate(dot(neighborSampleNormal, toNew * rsqrt(newDistSqr)));
    float oldCos = saturate(dot(neighborSampleNormal, toOld * rsqrt(oldDistSqr)));
    if (newCos <= 1e-6 || oldCos <= 1e-6)
        return 0.0;

    float jacobian = (newCos * oldDistSqr) / (oldCos * newDistSqr);
    return IsFiniteIndirectScalar(jacobian) ? jacobian : 0.0;
}

bool ValidateIndirectJacobian(inout float jacobian)
{
    if (jacobian > RESTIR_GI_DISCARD_JACOBIAN || jacobian < (1.0 / RESTIR_GI_DISCARD_JACOBIAN))
        return false;

    jacobian = clamp(jacobian, RESTIR_GI_MIN_JACOBIAN, RESTIR_GI_MAX_JACOBIAN);
    return true;
}

float ComputeIndirectTargetPdf(IndirectReservoirData sample)
{
    return max(sample.targetLum, 0.0);
}

IndirectReservoirData EmptyIndirectReservoir()
{
    return (IndirectReservoirData)0;
}

void InitializeIndirectReservoirSample(
    inout IndirectReservoirData reservoir,
    float3 secondaryPosition,
    float proposalPdf,
    float3 secondaryNormal,
    float targetLum,
    float3 radiance,
    float3 contribution,
    float3 primaryNormal)
{
    reservoir.secondaryPosition = secondaryPosition;
    reservoir.proposalPdf = max(proposalPdf, RESTIR_GI_MIN_PROPOSAL_PDF);
    reservoir.secondaryNormal = secondaryNormal;
    reservoir.targetLum = targetLum;
    reservoir.radiance = radiance;
    reservoir.weightSum = targetLum * ComputeIndirectProposalInversePdf(reservoir.proposalPdf);
    reservoir.contribution = contribution;
    reservoir.selectedWeight = ComputeIndirectMISWeight(reservoir.weightSum, targetLum, 1.0);
    reservoir.primaryNormal = primaryNormal;
    reservoir.sampleCount = 1.0;
}

float GetIndirectReservoirRISWeight(IndirectReservoirData candidate, float targetPdf)
{
    return max(targetPdf, 0.0) * ComputeIndirectProposalInversePdf(candidate.proposalPdf) * max(candidate.sampleCount, 0.0);
}

bool CombineIndirectReservoirs(
    inout IndirectReservoirData reservoir,
    IndirectReservoirData candidate,
    float random,
    float targetPdf)
{
    float risWeight = GetIndirectReservoirRISWeight(candidate, targetPdf);
    if (risWeight <= 0.0)
        return false;

    reservoir.sampleCount += candidate.sampleCount;
    reservoir.weightSum += risWeight;

    bool selectSample = random * reservoir.weightSum <= risWeight;
    if (selectSample)
    {
        reservoir.secondaryPosition = candidate.secondaryPosition;
        reservoir.proposalPdf = candidate.proposalPdf;
        reservoir.secondaryNormal = candidate.secondaryNormal;
        reservoir.targetLum = targetPdf;
        reservoir.radiance = candidate.radiance;
        reservoir.contribution = candidate.contribution;
        reservoir.primaryNormal = candidate.primaryNormal;
    }

    return selectSample;
}

void FinalizeIndirectReservoir(
    inout IndirectReservoirData reservoir,
    float normalizationNumerator,
    float normalizationDenominator)
{
    reservoir.weightSum = normalizationDenominator <= 0.0
        ? 0.0
        : (reservoir.weightSum * normalizationNumerator) / normalizationDenominator;

    reservoir.selectedWeight = ComputeIndirectMISWeight(reservoir.weightSum, reservoir.targetLum, reservoir.sampleCount);
}
