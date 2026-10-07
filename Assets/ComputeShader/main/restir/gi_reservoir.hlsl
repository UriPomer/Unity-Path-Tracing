#pragma once

static const float RESTIR_GI_MAX_RESERVOIR_SAMPLES = 32.0;
static const uint RESTIR_GI_RESERVOIR_FLAG_ENVIRONMENT = 1u;

bool IsFiniteIndirectScalar(float v)
{
    return isfinite(v);
}

bool IsFiniteIndirectFloat3(float3 v)
{
    return all(isfinite(v));
}

// weightSum holds the temporary RIS stream sum while combining, and the
// finalized contribution multiplier W after FinalizeIndirectReservoir.
float ComputeIndirectProposalInversePdf(float proposalPdf)
{
    return proposalPdf > 0.0 ? rcp(proposalPdf) : 0.0;
}

bool IsIndirectReservoirValid(IndirectReservoirData r)
{
    return r.targetLum > 0.0 && r.weightSum > 0.0 &&
        r.sampleCount > 0.0 && r.sampleCount <= RESTIR_GI_MAX_RESERVOIR_SAMPLES &&
        (r.sampleFlags & ~RESTIR_GI_RESERVOIR_FLAG_ENVIRONMENT) == 0u &&
        IsFiniteIndirectFloat3(r.secondaryPosition) &&
        IsFiniteIndirectFloat3(r.secondaryNormal) &&
        IsFiniteIndirectFloat3(r.secondaryGeometryNormal) &&
        IsFiniteIndirectFloat3(r.radiance) &&
        IsFiniteIndirectFloat3(r.contribution) &&
        IsFiniteIndirectScalar(r.proposalPdf) &&
        IsFiniteIndirectScalar(r.targetLum) &&
        IsFiniteIndirectScalar(r.weightSum) &&
        IsFiniteIndirectScalar(r.sampleCount) &&
        r.proposalPdf > 0.0;
}

bool IsIndirectEnvironmentSample(uint sampleFlags)
{
    return (sampleFlags & RESTIR_GI_RESERVOIR_FLAG_ENVIRONMENT) != 0u;
}

bool ResolveIndirectSampleDirection(
    HitData hd,
    float3 secondaryPosition,
    uint sampleFlags,
    out float3 direction,
    out float distance)
{
    float3 directionVector = IsIndirectEnvironmentSample(sampleFlags)
        ? secondaryPosition
        : secondaryPosition - hd.position;
    float directionLength = length(directionVector);
    if (directionLength <= 1e-5 || !isfinite(directionLength))
    {
        direction = 0.0;
        distance = 0.0;
        return false;
    }

    direction = directionVector / directionLength;
    distance = IsIndirectEnvironmentSample(sampleFlags) ? 1e20 : directionLength;
    return all(isfinite(direction));
}

RayHit BuildPrimaryRayHit(HitData hd)
{
    RayHit hit;
    hit.position = hd.position;
    hit.distance = hd.distance;
    hit.normal = hd.normal;
    hit.geometryNormal = hd.geometryNormal;
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
    uint sampleFlags,
    float3 sampleRadiance,
    out float3 f_brdf,
    out float3 reflectedRadiance,
    bool previousView = false)
{
    f_brdf = 0.0;
    reflectedRadiance = 0.0;

    if (hd.distance >= 1e19 || !IsFiniteIndirectFloat3(secondaryPosition) || !IsFiniteIndirectFloat3(sampleRadiance))
        return false;

    RayHit primaryHit = BuildPrimaryRayHit(hd);
    float3 cameraPos = previousView ? _RestirPreviousCameraPosition : _CameraToWorld._m03_m13_m23;
    float3 V = normalize(cameraPos - hd.position);
    float3 primaryNormal = GetDirectLightSurfaceNormal(primaryHit, V);
    primaryHit.normal = primaryNormal;

    float3 L;
    float distToSecondary;
    if (!ResolveIndirectSampleDirection(hd, secondaryPosition, sampleFlags, L, distToSecondary))
        return false;
    float NdotL = saturate(dot(primaryNormal, L));
    if (NdotL <= 0.0)
        return false;

    float proposalPdf;
    EvaluateOpaqueBXDF_GivenDir(primaryHit, V, L, f_brdf, proposalPdf);
    if (!IsFiniteIndirectFloat3(f_brdf))
        return false;

    float3 radiance = max(sampleRadiance, 0.0);
    float3 reflected = max(f_brdf * NdotL * radiance, 0.0);
    if (!IsFiniteIndirectFloat3(reflected))
        return false;

    reflectedRadiance = reflected;
    return true;
}

// Validity-gated wrapper around EvaluateIndirectRadianceAtSurface for reservoir samples:
// returns the reflected radiance integrand (f_brdf * NdotL * source radiance) for finite,
// valid reservoir samples. Callers multiply by sample.weightSum themselves to obtain the
// unbiased GI estimate radiance * NdotL * f_brdf * W (RTXDI FinalShading.hlsl:66 parity).
bool EvaluateIndirectSampleAtSurface(
    HitData hd,
    IndirectReservoirData sample,
    out float3 radiance,
    out float3 reflectedRadiance,
    bool previousView = false)
{
    radiance = 0.0;
    reflectedRadiance = 0.0;

    if (!IsIndirectReservoirValid(sample) || hd.distance >= 1e19)
        return false;

    if (!IsIndirectEnvironmentSample(sample.sampleFlags) &&
        (dot(sample.secondaryNormal, hd.position - sample.secondaryPosition) <= 0.0 ||
         dot(sample.secondaryGeometryNormal, hd.position - sample.secondaryPosition) <= 0.0))
        return false;

    radiance = max(sample.radiance, 0.0);
    float3 f_brdf;
    return EvaluateIndirectRadianceAtSurface(hd, sample.secondaryPosition, sample.sampleFlags, radiance, f_brdf, reflectedRadiance, previousView)
        && IsFiniteIndirectFloat3(reflectedRadiance);
}

bool EvaluateIndirectSampleAtSurface(
    HitData hd,
    IndirectReservoirData sample,
    out float3 reflectedRadiance)
{
    float3 radiance;
    return EvaluateIndirectSampleAtSurface(hd, sample, radiance, reflectedRadiance);
}

bool ReevaluateIndirectReservoirAtSurface(
    HitData hd,
    IndirectReservoirData sample,
    out float3 radiance,
    out float3 contribution,
    out float targetLum,
    bool previousView = false)
{
    radiance = 0.0;
    contribution = 0.0;
    targetLum = 0.0;

    float3 reflectedRadiance;
    if (!EvaluateIndirectSampleAtSurface(hd, sample, radiance, reflectedRadiance, previousView))
        return false;

    contribution = reflectedRadiance;
    targetLum = max(reflectedRadiance.x, max(reflectedRadiance.y, reflectedRadiance.z));
    return targetLum > 0.0 && IsFiniteIndirectFloat3(contribution);
}

bool IsIndirectSampleVisibleAtSurface(HitData surface, IndirectReservoirData sample)
{
    float3 direction;
    float distance;
    if (!ResolveIndirectSampleDirection(surface, sample.secondaryPosition, sample.sampleFlags, direction, distance))
        return false;
    Ray ray;
    ray.origin = surface.position + surface.geometryNormal *
        (dot(surface.geometryNormal, direction) >= 0.0 ? 1e-5 : -1e-5);
    if (!IsIndirectEnvironmentSample(sample.sampleFlags))
    {
        float3 connection = sample.secondaryPosition - ray.origin;
        distance = length(connection);
        direction = connection / distance;
    }
    ray.dir = direction;
    ray.invDir = 1.0 / direction;
    float tMax = IsIndirectEnvironmentSample(sample.sampleFlags) ? distance : max(distance - 1e-5, 0.0);
    return !IntersectTlasFast(ray, tMax);
}

float IndirectSourceTarget(HitData surface, IndirectReservoirData sample, bool previousView = false)
{
    float3 radiance, contribution;
    float target;
    if (!ReevaluateIndirectReservoirAtSurface(surface, sample, radiance, contribution, target, previousView))
        return 0.0;
    // MIS source weights must vanish outside that source's path support.
    return IsIndirectSampleVisibleAtSurface(surface, sample) ? target : 0.0;
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

    float3 reflectedRadiance;
    if (!EvaluateIndirectSampleAtSurface(hd, sample, radiance, reflectedRadiance))
    {
        RayHit primaryHit = BuildPrimaryRayHit(hd);
        float3 cameraPos = float3(_CameraToWorld._m03, _CameraToWorld._m13, _CameraToWorld._m23);
        float3 V = normalize(cameraPos - hd.position);
        float3 primaryNormal = GetDirectLightSurfaceNormal(primaryHit, V);

        float3 L;
        float distToSecondary;
        if (!ResolveIndirectSampleDirection(hd, sample.secondaryPosition, sample.sampleFlags, L, distToSecondary))
        {
            failureCode = 3u;
            return false;
        }
        float NdotL = saturate(dot(primaryNormal, L));
        if (NdotL <= 0.0)
        {
            failureCode = 5u;
            return false;
        }

        float3 f_brdf;
        float proposalPdf;
        EvaluateOpaqueBXDF_GivenDir(primaryHit, V, L, f_brdf, proposalPdf);
        if (!IsFiniteIndirectFloat3(f_brdf))
        {
            failureCode = 4u;
            return false;
        }

        failureCode = 6u;
        return false;
    }

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
    float3 neighborSampleNormal,
    uint sampleFlags)
{
    if (IsIndirectEnvironmentSample(sampleFlags))
        return 1.0;

    float3 toNew = receiverPos - neighborSamplePos;
    float3 toOld = neighborReceiverPos - neighborSamplePos;

    float newDistSqr = dot(toNew, toNew);
    float oldDistSqr = dot(toOld, toOld);
    if (newDistSqr <= 0.0 || oldDistSqr <= 0.0)
        return 0.0;

    float newCos = saturate(dot(neighborSampleNormal, toNew * rsqrt(newDistSqr)));
    float oldCos = saturate(dot(neighborSampleNormal, toOld * rsqrt(oldDistSqr)));
    if (newCos <= 0.0 || oldCos <= 0.0)
        return 0.0;

    float jacobian = (newCos * oldDistSqr) / (oldCos * newDistSqr);
    return IsFiniteIndirectScalar(jacobian) ? jacobian : 0.0;
}

bool ValidateIndirectJacobian(float jacobian)
{
    return jacobian > 0.0 && isfinite(jacobian);
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
    float3 secondaryGeometryNormal,
    float targetLum,
    float3 radiance,
    float3 contribution,
    uint sampleFlags)
{
    // RTXDI_MakeGIReservoir parity (Reservoir.hlsl:188-202):
    //   weightSum = 1 / samplePdf
    // i.e. the no-resampling reservoir starts with the path inverse PDF, not targetLum/p.
    // This makes stage2 reservoirs already in the "after Finalize" form. Reuse restores
    // their represented candidate domain through targetPdf * weightSum * M.
    reservoir.secondaryPosition = secondaryPosition;
    reservoir.proposalPdf = proposalPdf;
    reservoir.secondaryNormal = secondaryNormal;
    reservoir.secondaryGeometryNormal = secondaryGeometryNormal;
    reservoir.targetLum = targetLum;
    reservoir.radiance = radiance;
    reservoir.weightSum = ComputeIndirectProposalInversePdf(reservoir.proposalPdf);
    reservoir.contribution = contribution;
    reservoir.sampleFlags = sampleFlags;
    reservoir.reserved = 0.0;
    reservoir.sampleCount = 1.0;
}

// A reused reservoir's finalized weight is normalized over its candidate domain.
// Restore that domain count when streaming it into the next reservoir, as in RTXDI's
// targetPdf * weightSum * M combination rule. Fresh reservoirs have M = 1.
float GetIndirectReservoirRISWeight(IndirectReservoirData candidate, float targetPdf)
{
    return max(targetPdf, 0.0) * max(candidate.weightSum, 0.0) * max(candidate.sampleCount, 0.0);
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
        reservoir.secondaryGeometryNormal = candidate.secondaryGeometryNormal;
        reservoir.targetLum = targetPdf;
        reservoir.radiance = candidate.radiance;
        reservoir.contribution = candidate.contribution;
        reservoir.sampleFlags = candidate.sampleFlags;
    }

    return selectSample;
}

void FinalizeIndirectReservoir(
    inout IndirectReservoirData reservoir,
    float normalizationNumerator,
    float normalizationDenominator)
{
    // RTXDI form: accumulated RIS stream weight becomes the selected sample's
    // unbiased contribution weight W. denom == 0 short-circuits to 0
    // (mirrors RTXDI_FinalizeGIResampling).
    reservoir.weightSum = normalizationDenominator <= 0.0
        ? 0.0
        : (reservoir.weightSum * normalizationNumerator) / normalizationDenominator;


}

void WriteIndirectReservoirTelemetry(
    uint recordIndex,
    uint stage,
    uint reason,
    uint pixelIndex,
    IndirectReservoirData reservoir,
    float4 stageData)
{
    RestirTelemetryWriteRecord(
        recordIndex,
        stage,
        reason,
        pixelIndex,
        float4(reservoir.secondaryPosition, reservoir.proposalPdf),
        float4(reservoir.secondaryNormal, reservoir.targetLum),
        float4(reservoir.radiance, reservoir.weightSum),
        float4(reservoir.contribution, reservoir.weightSum),
        float4((float)reservoir.sampleFlags, reservoir.reserved, reservoir.sampleCount),
        stageData);
}
