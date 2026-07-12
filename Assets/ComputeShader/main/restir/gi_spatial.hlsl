#pragma once

static const int2 kNeighborOffsets[8] =
{
    int2(-1, 0),
    int2(1, 0),
    int2(0, -1),
    int2(0, 1),
    int2(-1, -1),
    int2(1, -1),
    int2(-1, 1),
    int2(1, 1)
};

int WrapNeighborOffsetIndex(int idx)
{
    return idx >= 8 ? (idx - 8) : idx;
}

[numthreads(64, 1, 1)]
void kernel_spatial_gi_resampling(uint3 id : SV_DispatchThreadID)
{
    uint pixelCount = _ScreenWidth * _ScreenHeight;
    if (id.x >= pixelCount) return;

    if (id.x == _RestirDebugPixelIndex)
    {
        [unroll]
        for (int debugSlot = 0; debugSlot < 5; debugSlot++)
            ReSTIRDebugData[debugSlot] = 0.0;
    }

    uint curIdx = _RestirShadingReservoirOffset + id.x;
    uint outIdx = _RestirSpatialReservoirOffset + id.x;

    IndirectReservoirData cur = IndirectReservoirs[curIdx];
    bool currentReservoirValid = IsIndirectReservoirValid(cur);
    IndirectReservoirs[outIdx] = cur;

    HitData hdCur = _RestirGbuffer[id.x];
    if (hdCur.distance >= 1e19)
    {
        RestirTelemetryCount(RESTIR_COUNTER_GI_SPATIAL_INVALID_CURRENT, id.x);
        WriteIndirectReservoirTelemetry(
            2u, RESTIR_STAGE_GI_SPATIAL, RESTIR_REASON_INVALID_SURFACE, id.x,
            cur, 0.0);
        return;
    }
    if (!currentReservoirValid && hdCur.roughness < 1e-4)
    {
        WriteIndirectReservoirTelemetry(
            2u, RESTIR_STAGE_GI_SPATIAL, RESTIR_REASON_NONE, id.x,
            cur, 0.0);
        return;
    }

    uint2 pixel = uint2(id.x % _ScreenWidth, id.x / _ScreenWidth);
    RNG_SeedPixel(rng, pixel, _FrameCount + 7919u);

    IndirectReservoirData outR = EmptyIndirectReservoir();
    float curTargetPdf = currentReservoirValid ? ComputeIndirectTargetPdf(cur) : 0.0;
    float selectedTargetPdf = 0.0;
    if (currentReservoirValid)
    {
        CombineIndirectReservoirs(outR, cur, 0.5, curTargetPdf);
        selectedTargetPdf = curTargetPdf;
    }
    else
    {
        RestirTelemetryCount(RESTIR_COUNTER_GI_SPATIAL_INVALID_CURRENT, id.x);
    }

    uint cachedResult = 0u;
    int selected = -1;
    uint compatibleCount = 0u;
    uint reevaluateCount = 0u;
    uint jacobianCount = 0u;
    uint combinedCount = 0u;
    uint reevaluateFailInvalidSample = 0u;
    uint reevaluateFailInvalidSurface = 0u;
    uint reevaluateFailDistance = 0u;
    uint reevaluateFailBrdf = 0u;
    uint reevaluateFailBackfacing = 0u;
    uint reevaluateFailZeroTarget = 0u;
    float selectedNeighborOriginalProposalPdf = 0.0;
    float selectedNeighborReuseProposalPdf = 0.0;
    float selectedNeighborJacobian = 0.0;
    float selectedNeighborTargetPdf = 0.0;

    int neighborStartIdx = min((int)(RNG_Next(rng) * 8.0), 7);
    [unroll]
    for (int neighborSampleIdx = 0; neighborSampleIdx < 8; neighborSampleIdx++)
    {
        int neighborOffsetIdx = WrapNeighborOffsetIndex(neighborStartIdx + neighborSampleIdx);
        int2 neighborPixel = int2(pixel) + kNeighborOffsets[neighborOffsetIdx];
        if (neighborPixel.x < 0 || neighborPixel.x >= (int)_ScreenWidth ||
            neighborPixel.y < 0 || neighborPixel.y >= (int)_ScreenHeight)
        {
            continue;
        }

        uint neighborIdx = (uint)(neighborPixel.y * (int)_ScreenWidth + neighborPixel.x);
        HitData hdNeighbor = _RestirGbuffer[neighborIdx];
        if (hdNeighbor.distance >= 1e19)
        {
            RestirTelemetryCount(RESTIR_COUNTER_GI_SPATIAL_INVALID_NEIGHBOR, id.x);
            continue;
        }

        if (!IsTemporalCompatible(hdCur.position, hdCur.normal, hdCur.mode,
                                  hdNeighbor.position, hdNeighbor.normal, hdNeighbor.mode))
        {
            RestirTelemetryCount(RESTIR_COUNTER_GI_SPATIAL_INCOMPATIBLE_NEIGHBOR, id.x);
            continue;
        }
        if (!AreRestirMaterialsSimilar(hdCur, hdNeighbor))
        {
            RestirTelemetryCount(RESTIR_COUNTER_GI_SPATIAL_INCOMPATIBLE_NEIGHBOR, id.x);
            continue;
        }
        compatibleCount++;

        if (dot(hdCur.normal, hdNeighbor.normal) < 0.95)
        {
            RestirTelemetryCount(RESTIR_COUNTER_GI_SPATIAL_INCOMPATIBLE_NEIGHBOR, id.x);
            continue;
        }

        uint neighborReservoirIdx = _RestirShadingReservoirOffset + neighborIdx;
        IndirectReservoirData neighbor = IndirectReservoirs[neighborReservoirIdx];
        if (!IsIndirectReservoirValid(neighbor))
        {
            RestirTelemetryCount(RESTIR_COUNTER_GI_SPATIAL_INVALID_NEIGHBOR, id.x);
            continue;
        }

        float3 neighborRadianceCur;
        float3 neighborContributionCur;
        float neighborTargetLumCur;
        uint reevaluateFailureCode = 0u;
        if (!ReevaluateIndirectReservoirAtSurfaceDebug(
                hdCur,
                neighbor,
                neighborRadianceCur,
                neighborContributionCur,
                neighborTargetLumCur,
                reevaluateFailureCode))
        {
            if (reevaluateFailureCode == 1u) reevaluateFailInvalidSample++;
            else if (reevaluateFailureCode == 2u) reevaluateFailInvalidSurface++;
            else if (reevaluateFailureCode == 3u) reevaluateFailDistance++;
            else if (reevaluateFailureCode == 4u) reevaluateFailBrdf++;
            else if (reevaluateFailureCode == 5u) reevaluateFailBackfacing++;
            else if (reevaluateFailureCode == 6u) reevaluateFailZeroTarget++;
            RestirTelemetryCount(RESTIR_COUNTER_GI_SPATIAL_REEVALUATION_REJECTED, id.x);
            continue;
        }
        reevaluateCount++;

        float jacobian = CalculateIndirectJacobian(
            hdCur.position,
            hdNeighbor.position,
            neighbor.secondaryPosition,
            neighbor.secondaryNormal,
            neighbor.sampleFlags);
        if (!ValidateIndirectJacobian(jacobian))
        {
            RestirTelemetryCount(RESTIR_COUNTER_GI_SPATIAL_JACOBIAN_REJECTED, id.x);
            continue;
        }
        jacobianCount++;

        IndirectReservoirData neighborCandidate = neighbor;
        neighborCandidate.radiance = neighborRadianceCur;
        neighborCandidate.contribution = neighborContributionCur;
        neighborCandidate.targetLum = neighborTargetLumCur;
        // Transform the finalized neighbor estimator into the current receiver's
        // solid-angle domain before combining it (RTXDI spatial parity).
        neighborCandidate.weightSum *= jacobian;
        neighborCandidate.proposalPdf = neighborCandidate.proposalPdf > 0.0
            ? max(neighborCandidate.proposalPdf / jacobian, RESTIR_GI_MIN_REUSE_PROPOSAL_PDF)
            : 0.0;
        // Clamp only the represented domain count. weightSum is already a finalized
        // estimator and must not be attenuated when history M is shortened.
        float neighborSampleCountClamped = min(max(neighborCandidate.sampleCount, 1.0), RESTIR_GI_MAX_RESERVOIR_SAMPLES - 1.0);
        if (neighborCandidate.sampleCount > neighborSampleCountClamped)
            RestirTelemetryCount(RESTIR_COUNTER_GI_SPATIAL_M_CAPPED, id.x);
        neighborCandidate.sampleCount = neighborSampleCountClamped;

        cachedResult |= (1u << uint(neighborSampleIdx));
        combinedCount++;
        RestirTelemetryCount(RESTIR_COUNTER_GI_SPATIAL_NEIGHBOR_COMBINED, id.x);
        bool candidateSelected = CombineIndirectReservoirs(outR, neighborCandidate, RNG_Next(rng), neighborTargetLumCur);
        if (candidateSelected)
        {
            RestirTelemetryCount(RESTIR_COUNTER_GI_SPATIAL_NEIGHBOR_SELECTED, id.x);
            selected = neighborSampleIdx;
            selectedTargetPdf = neighborTargetLumCur;
            selectedNeighborOriginalProposalPdf = neighbor.proposalPdf;
            selectedNeighborReuseProposalPdf = neighborCandidate.proposalPdf;
            selectedNeighborJacobian = jacobian;
            selectedNeighborTargetPdf = neighborTargetLumCur;
        }
    }

    // Same proportional-scale M cap, applied to the streamed reservoir before the second
    // (normalization) pass. With up to 8 neighbors combined, outR.sampleCount can reach
    // 1 (cur) + 8 * (MAX-1) which exceeds MAX; without scaling weightSum the per-neighbor
    // RIS contributions would still be summed at full magnitude.
    float outSampleCountBeforeCap = outR.sampleCount;
    float outSampleCountClamped = min(outSampleCountBeforeCap, RESTIR_GI_MAX_RESERVOIR_SAMPLES);
    float spatialMCapScale = 1.0;
    if (outSampleCountBeforeCap > outSampleCountClamped)
    {
        RestirTelemetryCount(RESTIR_COUNTER_GI_SPATIAL_M_CAPPED, id.x);
        spatialMCapScale = outSampleCountClamped / outSampleCountBeforeCap;
        outR.weightSum *= spatialMCapScale;
    }
    outR.sampleCount = outSampleCountClamped;

    float pi = selectedTargetPdf;
    // The global cap scales the streamed weight and its effective domain counts together.
    // Otherwise Finalize applies the cap once through weightSum and again through piSum.
    float piSum = curTargetPdf * max(cur.sampleCount, 1.0) * spatialMCapScale;
    [unroll]
    for (int cachedSampleIdx = 0; cachedSampleIdx < 8; cachedSampleIdx++)
    {
        if ((cachedResult & (1u << uint(cachedSampleIdx))) == 0)
            continue;

        int cachedNeighborOffsetIdx = WrapNeighborOffsetIndex(neighborStartIdx + cachedSampleIdx);
        int2 neighborPixel = int2(pixel) + kNeighborOffsets[cachedNeighborOffsetIdx];
        if (neighborPixel.x < 0 || neighborPixel.x >= (int)_ScreenWidth ||
            neighborPixel.y < 0 || neighborPixel.y >= (int)_ScreenHeight)
        {
            continue;
        }

        uint neighborIdx = (uint)(neighborPixel.y * (int)_ScreenWidth + neighborPixel.x);
        HitData hdNeighbor = _RestirGbuffer[neighborIdx];
        if (hdNeighbor.distance >= 1e19)
            continue;

        uint neighborReservoirIdx = _RestirShadingReservoirOffset + neighborIdx;
        IndirectReservoirData neighbor = IndirectReservoirs[neighborReservoirIdx];
        if (!IsIndirectReservoirValid(neighbor))
            continue;

        float3 neighborRadianceNeighbor;
        float3 neighborContributionNeighbor;
        float neighborP = 0.0;
        if (ReevaluateIndirectReservoirAtSurface(hdNeighbor, outR, neighborRadianceNeighbor, neighborContributionNeighbor, neighborP))
        {
            pi = selected == cachedSampleIdx ? neighborP : pi;
            float neighborSampleCount = min(max(neighbor.sampleCount, 1.0), RESTIR_GI_MAX_RESERVOIR_SAMPLES - 1.0);
            piSum += neighborP * max(neighborSampleCount, 0.0) * spatialMCapScale;
        }
    }

    float normalizationNumerator = pi;
    float normalizationDenominator = selectedTargetPdf * piSum;
    if (normalizationDenominator <= 0.0)
        RestirTelemetryCount(RESTIR_COUNTER_GI_SPATIAL_ZERO_NORMALIZATION, id.x);
    FinalizeIndirectReservoir(outR, normalizationNumerator, normalizationDenominator);
    bool outputFinite = all(isfinite(outR.secondaryPosition)) &&
        all(isfinite(outR.secondaryNormal)) && all(isfinite(outR.radiance)) &&
        all(isfinite(outR.contribution)) &&
        isfinite(outR.proposalPdf) && isfinite(outR.targetLum) &&
        isfinite(outR.weightSum) && isfinite(outR.selectedWeight) && isfinite(outR.sampleCount);
    if (!outputFinite)
    {
        RestirTelemetryCount(RESTIR_COUNTER_GI_SPATIAL_NONFINITE_OUTPUT, id.x);
        RestirTelemetryCountCritical(RESTIR_COUNTER_CRITICAL_NONFINITE);
    }
    if (id.x == RestirTelemetrySelectedPixel())
    {
        WriteIndirectReservoirTelemetry(
            2u,
            RESTIR_STAGE_GI_SPATIAL,
            outputFinite ? RESTIR_REASON_NONE : RESTIR_REASON_NONFINITE_RESERVOIR,
            id.x,
            outR,
            float4((float)compatibleCount, (float)reevaluateCount, (float)combinedCount, normalizationDenominator));
        RestirTelemetryWriteRecord(
            7u,
            RESTIR_STAGE_GI_SPATIAL,
            RESTIR_REASON_NONE,
            id.x,
            float4(spatialMCapScale, outSampleCountBeforeCap, piSum, normalizationDenominator),
            float4(0.0, 0.0, 0.0, 0.0),
            float4(0.0, 0.0, 0.0, 0.0),
            float4(0.0, 0.0, 0.0, 0.0),
            float4(0.0, 0.0, 0.0, 0.0),
            float4(0.0, 0.0, 0.0, 0.0));
    }
    if (id.x == _RestirDebugPixelIndex)
    {
        ReSTIRDebugData[0] = float4(
            (float)compatibleCount,
            (float)reevaluateCount,
            (float)jacobianCount,
            (float)combinedCount);
        ReSTIRDebugData[1] = float4(
            (float)reevaluateFailInvalidSample,
            (float)reevaluateFailInvalidSurface,
            (float)reevaluateFailDistance,
            (float)reevaluateFailBrdf);
        ReSTIRDebugData[2] = float4(
            (float)reevaluateFailBackfacing,
            (float)reevaluateFailZeroTarget,
            (float)selected,
            selectedTargetPdf);
        ReSTIRDebugData[3] = float4(
            selectedNeighborOriginalProposalPdf,
            selectedNeighborReuseProposalPdf,
            selectedNeighborJacobian,
            selectedNeighborTargetPdf);
        ReSTIRDebugData[4] = float4(
            pi,
            piSum,
            normalizationDenominator,
            curTargetPdf);
    }
    IndirectReservoirs[outIdx] = outR;
}
