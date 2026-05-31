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

    uint curIdx = _RestirShadingReservoirOffset + id.x;
    uint outIdx = _RestirSpatialReservoirOffset + id.x;

    IndirectReservoirData cur = IndirectReservoirs[curIdx];
    IndirectReservoirs[outIdx] = cur;
    if (!IsIndirectReservoirValid(cur)) return;

    HitData hdCur = _RestirGbuffer[id.x];
    if (hdCur.distance >= 1e19) return;

    uint2 pixel = uint2(id.x % _ScreenWidth, id.x / _ScreenWidth);
    RNG_SeedPixel(rng, pixel, _FrameCount + 7919u);

    IndirectReservoirData outR = EmptyIndirectReservoir();
    float curTargetPdf = ComputeIndirectTargetPdf(cur);
    float selectedTargetPdf = 0.0;
    if (!CombineIndirectReservoirs(outR, cur, 0.5, curTargetPdf))
        return;
    selectedTargetPdf = curTargetPdf;

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
            continue;

        if (!IsTemporalCompatible(hdCur.position, hdCur.normal, hdCur.mode,
                                  hdNeighbor.position, hdNeighbor.normal, hdNeighbor.mode))
        {
            continue;
        }
        compatibleCount++;

        if (dot(hdCur.normal, hdNeighbor.normal) < 0.95)
            continue;

        uint neighborReservoirIdx = _RestirShadingReservoirOffset + neighborIdx;
        IndirectReservoirData neighbor = IndirectReservoirs[neighborReservoirIdx];
        if (!IsIndirectReservoirValid(neighbor))
            continue;

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
            continue;
        }
        reevaluateCount++;

        float jacobian = CalculateIndirectJacobian(hdCur.position, hdNeighbor.position, neighbor.secondaryPosition, neighbor.secondaryNormal);
        if (!ValidateIndirectJacobian(jacobian))
            continue;
        jacobianCount++;

        IndirectReservoirData neighborCandidate = neighbor;
        neighborCandidate.radiance = neighborRadianceCur;
        neighborCandidate.contribution = neighborContributionCur;
        neighborCandidate.targetLum = neighborTargetLumCur;
        neighborCandidate.primaryNormal = hdCur.normal;
        neighborCandidate.proposalPdf = neighborCandidate.proposalPdf > 0.0
            ? max(neighborCandidate.proposalPdf / jacobian, RESTIR_GI_MIN_PROPOSAL_PDF)
            : 0.0;
        // M-cap with proportional Wsum scaling, mirroring gi_temporal.hlsl. Without this
        // the spatial RIS streaming weight can drift unbounded across multi-neighbor reuse:
        // the 2026-05-31 Sponza regression showed spatialNormalizationDenominator hitting
        // 7.3e6 with 7/8 active neighbors. Scaling weightSum by clamped/sampleCount keeps
        // the streamed-weight to streamed-count ratio invariant when history is dropped.
        float neighborSampleCountClamped = min(max(neighborCandidate.sampleCount, 1.0), RESTIR_GI_MAX_RESERVOIR_SAMPLES - 1.0);
        if (neighborCandidate.sampleCount > neighborSampleCountClamped)
        {
            neighborCandidate.weightSum *= neighborSampleCountClamped / neighborCandidate.sampleCount;
        }
        neighborCandidate.sampleCount = neighborSampleCountClamped;

        cachedResult |= (1u << uint(neighborSampleIdx));
        combinedCount++;
        bool candidateSelected = CombineIndirectReservoirs(outR, neighborCandidate, RNG_Next(rng), neighborTargetLumCur);
        if (candidateSelected)
        {
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
    float outSampleCountClamped = min(outR.sampleCount, RESTIR_GI_MAX_RESERVOIR_SAMPLES);
    if (outR.sampleCount > outSampleCountClamped)
    {
        outR.weightSum *= outSampleCountClamped / outR.sampleCount;
    }
    outR.sampleCount = outSampleCountClamped;

    float pi = selectedTargetPdf;
    float piSum = curTargetPdf * max(cur.sampleCount, 1.0);
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
            piSum += neighborP * max(neighborSampleCount, 0.0);
        }
    }

    float normalizationNumerator = pi;
    float normalizationDenominator = selectedTargetPdf * piSum;
    FinalizeIndirectReservoir(outR, normalizationNumerator, normalizationDenominator);
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
