#pragma once

static const int2 kTemporalOffsets[5] =
{
    int2(0, 0),
    int2(-1, 0),
    int2(1, 0),
    int2(0, -1),
    int2(0, 1)
};

int WrapTemporalOffsetIndex(int idx)
{
    return idx >= 5 ? (idx - 5) : idx;
}

[numthreads(64, 1, 1)]
void kernel_temporal_gi_resampling(uint3 id : SV_DispatchThreadID)
{
    uint pixelCount = _ScreenWidth * _ScreenHeight;
    if (id.x >= pixelCount) return;

    uint curIdx = _RestirInitialReservoirOffset + id.x;
    uint outIdx = _RestirTemporalReservoirOffset + id.x;

    IndirectReservoirData cur = IndirectReservoirs[curIdx];
    IndirectReservoirs[outIdx] = cur;
    if (!IsIndirectReservoirValid(cur)) return;

    HitData hdCur = _RestirGbuffer[id.x];
    if (hdCur.distance >= 1e19) return;

    float4 prevClip = mul(_RestirPreviousViewProjection, float4(hdCur.position, 1.0));
    if (prevClip.w <= 1e-6) return;
    float2 prevUV = prevClip.xy / prevClip.w;
    bool reprojectedInBounds = !any(prevUV < -1.0) && !any(prevUV > 1.0);

    float2 prevScreen = prevUV * 0.5 + 0.5;
    int2 prevBasePx = reprojectedInBounds
        ? clamp(
            int2(prevScreen * float2(_ScreenWidth, _ScreenHeight)),
            int2(0, 0),
            int2((int)_ScreenWidth - 1, (int)_ScreenHeight - 1))
        : int2((int)(id.x % _ScreenWidth), (int)(id.x / _ScreenWidth));

    uint2 pixel = uint2(id.x % _ScreenWidth, id.x / _ScreenWidth);
    RNG_SeedPixel(rng, pixel, _FrameCount);

    IndirectReservoirData outR = EmptyIndirectReservoir();
    float curTargetPdf = ComputeIndirectTargetPdf(cur);
    float selectedTargetPdf = 0.0;
    if (!CombineIndirectReservoirs(outR, cur, 0.5, curTargetPdf))
        return;
    selectedTargetPdf = curTargetPdf;

    bool selectedPrevious = false;
    HitData selectedPrevSurface = (HitData)0;
    IndirectReservoirData selectedPrevCandidate = (IndirectReservoirData)0;
    float selectedPrevTargetPdf = 0.0;
    float selectedPrevOriginalProposalPdf = 0.0;
    float selectedPrevJacobian = 0.0;
    int selectedTemporalOffsetIdx = -1;
    bool selectedFallbackSample = false;

    int temporalSampleStartIdx = min((int)(RNG_Next(rng) * 5.0), 4);
    [unroll]
    for (int sampleIdx = 0; sampleIdx < 6; sampleIdx++)
    {
        bool isFallbackSample = sampleIdx == 5;
        int temporalOffsetIdx = WrapTemporalOffsetIndex(temporalSampleStartIdx + sampleIdx);
        int2 offset = isFallbackSample ? int2(0, 0) : kTemporalOffsets[temporalOffsetIdx];
        int2 candidatePx = isFallbackSample ? int2(pixel) : (prevBasePx + offset);
        if (candidatePx.x < 0 || candidatePx.x >= (int)_ScreenWidth ||
            candidatePx.y < 0 || candidatePx.y >= (int)_ScreenHeight)
        {
            continue;
        }

        uint prevPxIdx = (uint)(candidatePx.y * (int)_ScreenWidth + candidatePx.x);
        IndirectReservoirData prev = IndirectReservoirs[_RestirPrevReservoirOffset + prevPxIdx];
        if (!IsIndirectReservoirValid(prev))
            continue;

        HitData hdPrev = _RestirGbufferPrevious[prevPxIdx];
        if (hdPrev.distance >= 1e19)
            continue;

        if (!isFallbackSample &&
            !IsTemporalCompatible(hdCur.position, hdCur.normal, hdCur.mode, hdPrev.position, hdPrev.normal, hdPrev.mode))
        {
            continue;
        }

        float3 prevRadianceCur;
        float3 prevContributionCur;
        float prevTargetLumCur;
        if (!ReevaluateIndirectReservoirAtSurface(hdCur, prev, prevRadianceCur, prevContributionCur, prevTargetLumCur))
            continue;

        float jacobian = CalculateIndirectJacobian(hdCur.position, hdPrev.position, prev.secondaryPosition, prev.secondaryNormal);
        if (!ValidateIndirectJacobian(jacobian))
            continue;

        IndirectReservoirData prevCandidate = prev;
        prevCandidate.radiance = prevRadianceCur;
        prevCandidate.contribution = prevContributionCur;
        prevCandidate.targetLum = prevTargetLumCur;
        prevCandidate.primaryNormal = hdCur.normal;
        prevCandidate.proposalPdf = prevCandidate.proposalPdf > 0.0
            ? max(prevCandidate.proposalPdf / jacobian, RESTIR_GI_MIN_PROPOSAL_PDF)
            : 0.0;
        // M-cap with proportional Wsum scaling (TrueTrace-style firefly fence):
        // ratio of streamed weight to streamed sample count must stay invariant when
        // we drop history beyond MAX-1 samples, otherwise stale reservoirs accumulate
        // unbounded weightSum across frames.
        float prevSampleCountClamped = min(max(prevCandidate.sampleCount, 1.0), RESTIR_GI_MAX_RESERVOIR_SAMPLES - 1.0);
        if (prevCandidate.sampleCount > prevSampleCountClamped && prevCandidate.sampleCount > 0.0)
        {
            prevCandidate.weightSum *= prevSampleCountClamped / prevCandidate.sampleCount;
        }
        prevCandidate.sampleCount = prevSampleCountClamped;

        bool candidateSelected = CombineIndirectReservoirs(outR, prevCandidate, RNG_Next(rng), prevTargetLumCur);
        if (candidateSelected)
        {
            selectedPrevious = true;
            selectedTargetPdf = prevTargetLumCur;
            selectedPrevSurface = hdPrev;
            selectedPrevCandidate = prevCandidate;
            selectedPrevTargetPdf = prevTargetLumCur;
            selectedPrevOriginalProposalPdf = prev.proposalPdf;
            selectedPrevJacobian = jacobian;
            selectedTemporalOffsetIdx = temporalOffsetIdx;
            selectedFallbackSample = isFallbackSample;
            break;
        }
    }

    // Same proportional-scale M cap, applied to the streamed reservoir before Finalize.
    float outSampleCountClamped = min(outR.sampleCount, RESTIR_GI_MAX_RESERVOIR_SAMPLES);
    if (outR.sampleCount > outSampleCountClamped && outR.sampleCount > 0.0)
    {
        outR.weightSum *= outSampleCountClamped / outR.sampleCount;
    }
    outR.sampleCount = outSampleCountClamped;
    float pi = selectedTargetPdf;
    float piSum = curTargetPdf * max(cur.sampleCount, 1.0);
    float temporalP = 0.0;
    if (selectedTargetPdf > 0.0 && selectedPrevious)
    {
        IndirectReservoirData selectedSample = outR;
        float3 selectedRadiancePrev;
        float3 selectedContributionPrev;
        float selectedTargetLumPrev;
        temporalP = ReevaluateIndirectReservoirAtSurface(selectedPrevSurface, selectedSample, selectedRadiancePrev, selectedContributionPrev, selectedTargetLumPrev)
            ? selectedTargetLumPrev
            : 0.0;
    }
    pi = selectedPrevious ? temporalP : selectedTargetPdf;
    piSum += temporalP * max(selectedPrevCandidate.sampleCount, 0.0);

    float normalizationNumerator = pi;
    float normalizationDenominator = selectedTargetPdf * piSum;
    FinalizeIndirectReservoir(outR, normalizationNumerator, normalizationDenominator);
    if (id.x == _RestirDebugPixelIndex)
    {
        ReSTIRDebugData[0] = float4(
            selectedPrevious ? 1.0 : 0.0,
            (float)selectedTemporalOffsetIdx,
            selectedFallbackSample ? 1.0 : 0.0,
            curTargetPdf);
        ReSTIRDebugData[1] = float4(
            selectedTargetPdf,
            selectedPrevTargetPdf,
            selectedPrevOriginalProposalPdf,
            selectedPrevCandidate.proposalPdf);
        ReSTIRDebugData[2] = float4(
            selectedPrevJacobian,
            pi,
            piSum,
            normalizationDenominator);
    }
    IndirectReservoirs[outIdx] = outR;
}
