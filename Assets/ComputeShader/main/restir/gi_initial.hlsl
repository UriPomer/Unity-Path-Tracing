#pragma once

static const float RESTIR_GI_STAGE1_FLAG_SKY = 1.0;
static const float RESTIR_GI_STAGE1_FLAG_SPECULAR = 2.0;
static const float RESTIR_GI_STAGE1_FLAG_DELTA = 4.0;
static const float RESTIR_GI_STAGE1_FLAG_BYPASS = 8.0;
static const uint RESTIR_GI_MAX_SECONDARY_LIGHT_CANDIDATES = 8u;

bool IsGISecondaryMiss(float flags)
{
    return flags >= 0.0 && (((uint)flags & (uint)RESTIR_GI_STAGE1_FLAG_SKY) != 0u);
}

bool IsGISecondaryBypass(float flags)
{
    return flags >= 0.0 && (((uint)flags & (uint)RESTIR_GI_STAGE1_FLAG_BYPASS) != 0u);
}

float3 EvaluateSecondaryMissRadiance(float3 secondaryNormal)
{
    return SampleSkyboxDirection(secondaryNormal);
}

float3 EvaluateSecondaryHitRadiance(RayHit secondaryHit, float3 viewDir)
{
    float3 radiance = 0.0;

    if (secondaryHit.material.emissionIntensity > 0.0)
        radiance += secondaryHit.material.emission * secondaryHit.material.emissionIntensity;

    float3 V2 = viewDir;
    float3 secondaryNormal = GetDirectLightSurfaceNormal(secondaryHit, V2);
    secondaryHit.normal = secondaryNormal;

    bool hasSun = _DirectionalLightColor.a > 0.0;
    uint pointLightCount = (uint)max(_PointLightsCount, 0);
    uint candidateCount = (hasSun ? 1u : 0u) + pointLightCount;
    if (candidateCount == 0u)
        return max(radiance, 0.0);

    uint sampleCount = min(candidateCount, RESTIR_GI_MAX_SECONDARY_LIGHT_CANDIDATES);
    uint candidateStart = min(uint(RNG_Next(rng) * candidateCount), candidateCount - 1u);
    uint candidateStride = max(candidateCount / sampleCount, 1u);
    float proposalPdf = rcp((float)candidateCount);
    DirectLightSample selectedSample = (DirectLightSample)0;
    float weightSum = 0.0;
    bool hasSelectedSample = false;
    [loop]
    for (uint candidate = 0u; candidate < sampleCount; candidate++)
    {
        uint candidateIndex = (candidateStart + candidate * candidateStride) % candidateCount;
        DirectLightSample sample = (DirectLightSample)0;
        if (!SampleDirectLightCandidate(
                secondaryHit,
                V2,
                float3(1, 1, 1),
                hasSun,
                0u,
                false,
                candidateIndex,
                proposalPdf,
                sample) ||
            !IsValidDirectLightSample(sample))
        {
            continue;
        }

        float candidateWeight = sample.targetLum / max(sample.proposalPdf, 1e-6);
        weightSum += candidateWeight;
        if (!hasSelectedSample || RNG_Next(rng) * weightSum < candidateWeight)
        {
            selectedSample = sample;
            hasSelectedSample = true;
        }
    }

    if (hasSelectedSample && IsDirectLightSampleVisible(selectedSample))
    {
        float selectedWeight = ComputeMISWeight(
            weightSum,
            selectedSample.targetLum,
            sampleCount);
        radiance += selectedSample.contribution * selectedWeight;
    }

    return max(radiance, 0.0);
}

[numthreads(64, 1, 1)]
void kernel_generate_gi_secondary_surfaces(uint3 id : SV_DispatchThreadID)
{
    uint pixelCount = _ScreenWidth * _ScreenHeight;
    if (id.x >= pixelCount) return;

    SecondarySurfaceData data = (SecondarySurfaceData)0;
    data.flags = -10.0;
    SecondarySurfaces[id.x] = data;

    HitData hd = _RestirGbuffer[id.x];
    data.primaryDistance = hd.distance;
    if (hd.distance >= 1e19)
    {
        RestirTelemetryCount(RESTIR_COUNTER_GI_INITIAL_PRIMARY_MISS, id.x);
        data.flags = -11.0;
        SecondarySurfaces[id.x] = data;
        return;
    }

    uint2 pixel = uint2(id.x % _ScreenWidth, id.x / _ScreenWidth);
    RNG_SeedPixel(rng, pixel, _FrameCount);
    _Pixel = pixel;

    RayHit primaryHit;
    primaryHit.position = hd.position;
    primaryHit.distance = hd.distance;
    primaryHit.normal = hd.normal;
    primaryHit.mode = hd.mode;
    primaryHit.material.albedo = hd.albedo;
    primaryHit.material.emission = hd.emission;
    primaryHit.material.emissionIntensity = hd.emissionIntensity;
    primaryHit.material.roughness = hd.roughness;
    primaryHit.material.metallic = hd.metallic;
    primaryHit.material.alpha = hd.alpha;
    primaryHit.material.ior = hd.ior;
    primaryHit.should_break = false;

    float3 cameraPos = _CameraToWorld._m03_m13_m23;
    float3 V = normalize(cameraPos - hd.position);
    float3 primaryNormal = GetDirectLightSurfaceNormal(primaryHit, V);
    primaryHit.normal = primaryNormal;

    Ray bounceRay = (Ray)0;
    bounceRay.origin = primaryHit.position + primaryNormal * 1e-5;
    bounceRay.dir = -V;
    bounceRay.invDir = 1.0 / bounceRay.dir;

    data.position = primaryHit.position;
    data.normal = primaryNormal;
    data.primaryDistance = primaryHit.distance;
    data.flags = -1.0;
    data.reserved = 0.0;

    float3 throughputSample;
    bool sampledSpecular;
    float throughputZeroReason;
    EvaluateBXDFWithDotAndPDFDetailed(primaryHit, bounceRay, throughputSample, sampledSpecular, throughputZeroReason);
    data.throughput = throughputSample;
    data.normal = bounceRay.dir;
    data.reserved = throughputZeroReason;
    if (all(throughputSample <= 0.0))
    {
        RestirTelemetryCount(RESTIR_COUNTER_GI_INITIAL_ZERO_THROUGHPUT, id.x);
        data.flags = -2.0;
        SecondarySurfaces[id.x] = data;
        return;
    }

    float3 pdfBrdf;
    float proposalPdf;
    EvaluateBXDF_GivenDir(primaryHit, V, bounceRay.dir, pdfBrdf, proposalPdf);
    if (proposalPdf <= 0.0)
    {
        RestirTelemetryCount(RESTIR_COUNTER_GI_INITIAL_INVALID_PROPOSAL, id.x);
        data.proposalPdf = proposalPdf;
        data.flags = -3.0;
        SecondarySurfaces[id.x] = data;
        return;
    }

    data.proposalPdf = proposalPdf;

    bounceRay.origin = primaryHit.position + primaryNormal * 1e-5;
    bounceRay.invDir = 1.0 / bounceRay.dir;

    RayHit secondaryHit = (RayHit)0;
    secondaryHit = Trace(bounceRay);

    data.position = secondaryHit.distance >= 1e19 ? normalize(bounceRay.dir) : secondaryHit.position;
    data.proposalPdf = proposalPdf;
    data.normal = secondaryHit.distance >= 1e19 ? -bounceRay.dir : secondaryHit.normal;
    data.primaryDistance = primaryHit.distance;
    bool isDeltaSurface = primaryHit.material.roughness < 1e-4;
    float sampleFlags = 0.0;
    if (secondaryHit.distance >= 1e19)
        sampleFlags += RESTIR_GI_STAGE1_FLAG_SKY;
    if (sampledSpecular)
        sampleFlags += RESTIR_GI_STAGE1_FLAG_SPECULAR;
    if (isDeltaSurface)
        sampleFlags += RESTIR_GI_STAGE1_FLAG_DELTA;
    if (sampledSpecular && isDeltaSurface)
        sampleFlags += RESTIR_GI_STAGE1_FLAG_BYPASS;

    data.flags = sampleFlags;
    if (!IsGISecondaryBypass(sampleFlags))
        data.throughput = 1.0;
    data.albedo = secondaryHit.distance >= 1e19 ? 0.0 : secondaryHit.material.albedo;
    data.roughness = secondaryHit.distance >= 1e19 ? 0.0 : secondaryHit.material.roughness;
    data.emissionRadiance = secondaryHit.distance >= 1e19
        ? SampleSkyboxDirection(bounceRay.dir)
        : max(secondaryHit.material.emission * secondaryHit.material.emissionIntensity, 0.0);
    data.metallic = secondaryHit.distance >= 1e19 ? 0.0 : secondaryHit.material.metallic;
    data.alpha = secondaryHit.distance >= 1e19 ? 1.0 : secondaryHit.material.alpha;
    data.ior = secondaryHit.distance >= 1e19 ? 1.0 : secondaryHit.material.ior;
    data.mode = secondaryHit.distance >= 1e19 ? 0.0 : secondaryHit.mode;
    SecondarySurfaces[id.x] = data;
}

[numthreads(64, 1, 1)]
void kernel_shade_gi_secondary_surfaces(uint3 id : SV_DispatchThreadID)
{
    uint pixelCount = _ScreenWidth * _ScreenHeight;
    if (id.x >= pixelCount) return;

    IndirectReservoirData reservoir = EmptyIndirectReservoir();
    IndirectReservoirs[_RestirInitialReservoirOffset + id.x] = reservoir;

    SecondarySurfaceData secondary = SecondarySurfacesRead[id.x];
    if (secondary.proposalPdf <= 0.0 || all(secondary.throughput <= 0.0))
    {
        RestirTelemetryCount(RESTIR_COUNTER_GI_INITIAL_INVALID_SECONDARY, id.x);
        WriteIndirectReservoirTelemetry(
            0u,
            RESTIR_STAGE_GI_INITIAL,
            secondary.proposalPdf <= 0.0 ? RESTIR_REASON_INVALID_PROPOSAL_PDF : RESTIR_REASON_ZERO_THROUGHPUT,
            id.x,
            reservoir,
            float4(secondary.throughput, secondary.flags));
        if (id.x == _RestirDebugPixelIndex)
        {
            ReSTIRDebugData[0] = float4(10.0, secondary.proposalPdf, secondary.throughput.x, secondary.flags);
            ReSTIRDebugData[1] = float4(
                secondary.emissionRadiance.x,
                secondary.emissionRadiance.y,
                secondary.emissionRadiance.z,
                0.0);
        }
        return;
    }

    RayHit secondaryHit = (RayHit)0;
    secondaryHit.position = secondary.position;
    bool isMissSample = IsGISecondaryMiss(secondary.flags);
    bool bypassReservoir = IsGISecondaryBypass(secondary.flags);

    secondaryHit.distance = isMissSample ? 1e19 : 1.0;
    secondaryHit.normal = secondary.normal;
    secondaryHit.mode = secondary.mode;
    secondaryHit.material.albedo = secondary.albedo;
    secondaryHit.material.emission = secondary.emissionRadiance;
    secondaryHit.material.emissionIntensity = 1.0;
    secondaryHit.material.roughness = secondary.roughness;
    secondaryHit.material.metallic = secondary.metallic;
    secondaryHit.material.alpha = secondary.alpha;
    secondaryHit.material.ior = secondary.ior;
    secondaryHit.should_break = false;

    uint2 pixel = uint2(id.x % _ScreenWidth, id.x / _ScreenWidth);
    RNG_SeedPixel(rng, pixel, _FrameCount + 1543u);

    float3 viewDir = isMissSample
        ? -normalize(secondary.position)
        : normalize(_RestirGbuffer[id.x].position - secondary.position);
    float3 secondaryRadiance = max(secondary.emissionRadiance, 0.0);
    if (!isMissSample)
    {
        secondaryRadiance = EvaluateSecondaryHitRadiance(secondaryHit, viewDir);
    }

    if (id.x == _RestirDebugPixelIndex)
    {
        ReSTIRDebugData[0] = float4(11.0, secondary.proposalPdf, secondary.flags, secondary.primaryDistance);
        ReSTIRDebugData[1] = float4(secondaryRadiance.x, secondaryRadiance.y, secondaryRadiance.z, length(viewDir));
        ReSTIRDebugData[2] = float4(
            secondary.position.x,
            secondary.position.y,
            secondary.position.z,
            0.0);
    }

    if (bypassReservoir)
    {
        float3 contribution = max(secondary.throughput * secondaryRadiance, 0.0);
        float targetLum = max(contribution.x, max(contribution.y, contribution.z));
        GlobalColors[id.x].L += contribution;
        WriteIndirectReservoirTelemetry(
            0u, RESTIR_STAGE_GI_INITIAL, RESTIR_REASON_NONE, id.x,
            reservoir, float4(secondary.throughput, secondary.flags));
        if (id.x == _RestirDebugPixelIndex)
        {
            ReSTIRDebugData[0] = float4(15.0, secondary.proposalPdf, targetLum, secondary.flags);
            ReSTIRDebugData[1] = float4(contribution.x, contribution.y, contribution.z, 0.0);
            ReSTIRDebugData[2] = float4(secondary.throughput.x, secondary.throughput.y, secondary.throughput.z, 0.0);
        }
        return;
    }

    float3 brdfAtPrimary;
    float3 contribution;
    uint reservoirSampleFlags = isMissSample ? RESTIR_GI_RESERVOIR_FLAG_ENVIRONMENT : 0u;
    if (!EvaluateIndirectRadianceAtSurface(
            _RestirGbuffer[id.x],
            secondary.position,
            reservoirSampleFlags,
            secondaryRadiance,
            brdfAtPrimary,
            contribution))
    {
        RestirTelemetryCount(RESTIR_COUNTER_GI_INITIAL_INVALID_SECONDARY, id.x);
        WriteIndirectReservoirTelemetry(
            0u, RESTIR_STAGE_GI_INITIAL, RESTIR_REASON_REEVALUATION_REJECTED, id.x,
            reservoir, float4(secondary.throughput, secondary.flags));
        if (id.x == _RestirDebugPixelIndex)
        {
            ReSTIRDebugData[0] = float4(12.0, secondary.proposalPdf, secondary.flags, secondary.primaryDistance);
            ReSTIRDebugData[1] = float4(secondaryRadiance.x, secondaryRadiance.y, secondaryRadiance.z, 0.0);
            ReSTIRDebugData[2] = float4(brdfAtPrimary.x, brdfAtPrimary.y, brdfAtPrimary.z, 0.0);
        }
        return;
    }

    float targetLum = max(contribution.x, max(contribution.y, contribution.z));
    if (targetLum <= 0.0)
    {
        RestirTelemetryCount(RESTIR_COUNTER_GI_INITIAL_ZERO_TARGET, id.x);
        WriteIndirectReservoirTelemetry(
            0u, RESTIR_STAGE_GI_INITIAL, RESTIR_REASON_ZERO_TARGET, id.x,
            reservoir, float4(secondary.throughput, secondary.flags));
        if (id.x == _RestirDebugPixelIndex)
        {
            ReSTIRDebugData[0] = float4(13.0, secondary.proposalPdf, secondary.flags, secondary.primaryDistance);
            ReSTIRDebugData[1] = float4(contribution.x, contribution.y, contribution.z, targetLum);
            ReSTIRDebugData[2] = float4(brdfAtPrimary.x, brdfAtPrimary.y, brdfAtPrimary.z, 0.0);
        }
        return;
    }

    if (id.x == _RestirDebugPixelIndex)
    {
        ReSTIRDebugData[0] = float4(14.0, secondary.proposalPdf, targetLum, secondary.flags);
        ReSTIRDebugData[1] = float4(contribution.x, contribution.y, contribution.z, 0.0);
        ReSTIRDebugData[2] = float4(brdfAtPrimary.x, brdfAtPrimary.y, brdfAtPrimary.z, 0.0);
    }

    InitializeIndirectReservoirSample(
        reservoir,
        secondary.position,
        secondary.proposalPdf,
        secondary.normal,
        targetLum,
        secondaryRadiance,
        contribution,
        reservoirSampleFlags);
    IndirectReservoirs[_RestirInitialReservoirOffset + id.x] = reservoir;
    RestirTelemetryCount(RESTIR_COUNTER_GI_INITIAL_ACCEPTED, id.x);

    bool reservoirFinite = all(isfinite(reservoir.secondaryPosition)) &&
        all(isfinite(reservoir.secondaryNormal)) && all(isfinite(reservoir.radiance)) &&
        all(isfinite(reservoir.contribution)) &&
        isfinite(reservoir.proposalPdf) && isfinite(reservoir.targetLum) &&
        isfinite(reservoir.weightSum) && isfinite(reservoir.selectedWeight) &&
        isfinite(reservoir.sampleCount);
    if (!reservoirFinite)
    {
        RestirTelemetryCount(RESTIR_COUNTER_GI_INITIAL_NONFINITE, id.x);
        RestirTelemetryCountCritical(RESTIR_COUNTER_CRITICAL_NONFINITE);
    }

    WriteIndirectReservoirTelemetry(
        0u,
        RESTIR_STAGE_GI_INITIAL,
        reservoirFinite ? RESTIR_REASON_NONE : RESTIR_REASON_NONFINITE_RESERVOIR,
        id.x,
        reservoir,
        float4(secondary.throughput, secondary.flags));
}
