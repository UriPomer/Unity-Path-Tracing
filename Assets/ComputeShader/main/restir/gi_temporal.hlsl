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

    if (id.x == _RestirDebugPixelIndex)
    {
        [unroll]
        for (int debugSlot = 0; debugSlot < 5; debugSlot++)
            ReSTIRDebugData[debugSlot] = 0.0;
    }

    uint curIdx = _RestirInitialReservoirOffset + id.x;
    uint outIdx = _RestirTemporalReservoirOffset + id.x;

    IndirectReservoirData cur = IndirectReservoirs[curIdx];
    IndirectReservoirs[outIdx] = cur;
    bool currentReservoirValid = IsIndirectReservoirValid(cur);
    if (!currentReservoirValid)
        RestirTelemetryCount(RESTIR_COUNTER_GI_TEMPORAL_INVALID_CURRENT, id.x);

    HitData hdCur = _RestirGbuffer[id.x];
    if (hdCur.distance >= 1e19)
    {
        RestirTelemetryCount(RESTIR_COUNTER_GI_TEMPORAL_INVALID_CURRENT, id.x);
        WriteIndirectReservoirTelemetry(
            1u, RESTIR_STAGE_GI_TEMPORAL, RESTIR_REASON_INVALID_SURFACE, id.x,
            cur, 0.0);
        return;
    }
    // Delta-primary samples bypass the GI reservoir and are shaded directly in
    // gi_initial.hlsl; history from a non-delta surface must not fill that path.
    if (!currentReservoirValid && hdCur.roughness < 1e-4)
    {
        WriteIndirectReservoirTelemetry(
            1u, RESTIR_STAGE_GI_TEMPORAL, RESTIR_REASON_NONE, id.x,
            cur, 0.0);
        return;
    }

    float4 prevClip = mul(_RestirPreviousViewProjection, float4(hdCur.position, 1.0));
    if (prevClip.w <= 1e-6)
    {
        RestirTelemetryCount(RESTIR_COUNTER_GI_TEMPORAL_REPROJECTION_OOB, id.x);
        WriteIndirectReservoirTelemetry(
            1u, RESTIR_STAGE_GI_TEMPORAL, RESTIR_REASON_REPROJECTION_OOB, id.x,
            cur, 0.0);
        return;
    }
    float2 prevUV = prevClip.xy / prevClip.w;
    bool reprojectedInBounds = !any(prevUV < -1.0) && !any(prevUV > 1.0);
    if (!reprojectedInBounds)
        RestirTelemetryCount(RESTIR_COUNTER_GI_TEMPORAL_REPROJECTION_OOB, id.x);

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
    float curTargetPdf = currentReservoirValid ? ComputeIndirectTargetPdf(cur) : 0.0;
    float selectedTargetPdf = 0.0;
    if (currentReservoirValid)
    {
        CombineIndirectReservoirs(outR, cur, 0.5, curTargetPdf);
        selectedTargetPdf = curTargetPdf;
    }

    bool selectedPrevious = false;
    HitData selectedPrevSurface = (HitData)0;
    IndirectReservoirData selectedPrevCandidate = (IndirectReservoirData)0;
    float selectedPrevTargetPdf = 0.0;
    float selectedPrevOriginalProposalPdf = 0.0;
    float selectedPrevJacobian = 0.0;
    int selectedTemporalOffsetIdx = -1;
    bool selectedFallbackSample = false;
    bool combinedPrevious = false;
    HitData combinedPrevSurface = (HitData)0;
    IndirectReservoirData combinedPrevCandidate = (IndirectReservoirData)0;
    float combinedPrevTargetPdf = 0.0;
    float combinedPrevOriginalProposalPdf = 0.0;
    float combinedPrevJacobian = 0.0;

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
        {
            RestirTelemetryCount(RESTIR_COUNTER_GI_TEMPORAL_INVALID_HISTORY, id.x);
            continue;
        }

        HitData hdPrev = _RestirGbufferPrevious[prevPxIdx];
        if (hdPrev.distance >= 1e19)
        {
            RestirTelemetryCount(RESTIR_COUNTER_GI_TEMPORAL_INVALID_HISTORY, id.x);
            continue;
        }

        if (!AreRestirMaterialsSimilar(hdCur, hdPrev) ||
            (!isFallbackSample &&
             !IsTemporalCompatible(hdCur.position, hdCur.normal, hdCur.mode, hdPrev.position, hdPrev.normal, hdPrev.mode)))
        {
            RestirTelemetryCount(RESTIR_COUNTER_GI_TEMPORAL_INCOMPATIBLE, id.x);
            continue;
        }

        float3 prevRadianceCur;
        float3 prevContributionCur;
        float prevTargetLumCur;
        if (!ReevaluateIndirectReservoirAtSurface(hdCur, prev, prevRadianceCur, prevContributionCur, prevTargetLumCur))
        {
            RestirTelemetryCount(RESTIR_COUNTER_GI_TEMPORAL_REEVALUATION_REJECTED, id.x);
            continue;
        }

        float jacobian = CalculateIndirectJacobian(
            hdCur.position,
            hdPrev.position,
            prev.secondaryPosition,
            prev.secondaryNormal,
            prev.sampleFlags);
        if (!ValidateIndirectJacobian(jacobian))
        {
            RestirTelemetryCount(RESTIR_COUNTER_GI_TEMPORAL_JACOBIAN_REJECTED, id.x);
            continue;
        }

        IndirectReservoirData prevCandidate = prev;
        prevCandidate.radiance = prevRadianceCur;
        prevCandidate.contribution = prevContributionCur;
        prevCandidate.targetLum = prevTargetLumCur;
        // The stored weight is already finalized. Transform that estimator into the
        // current receiver's solid-angle domain before streaming it again.
        prevCandidate.weightSum *= jacobian;
        prevCandidate.proposalPdf = prevCandidate.proposalPdf > 0.0
            ? max(prevCandidate.proposalPdf / jacobian, RESTIR_GI_MIN_REUSE_PROPOSAL_PDF)
            : 0.0;
        // M is the represented domain count, not part of the finalized estimator.
        // Clamp history length without attenuating weightSum (RTXDI temporal parity).
        float prevSampleCountClamped = min(max(prevCandidate.sampleCount, 1.0), RESTIR_GI_MAX_RESERVOIR_SAMPLES - 1.0);
        if (prevCandidate.sampleCount > prevSampleCountClamped)
            RestirTelemetryCount(RESTIR_COUNTER_GI_TEMPORAL_M_CAPPED, id.x);
        prevCandidate.sampleCount = prevSampleCountClamped;

        float prevRISWeight = GetIndirectReservoirRISWeight(prevCandidate, prevTargetLumCur);
        if (prevRISWeight <= 0.0)
            continue;

        bool candidateSelected = CombineIndirectReservoirs(outR, prevCandidate, RNG_Next(rng), prevTargetLumCur);
        RestirTelemetryCount(RESTIR_COUNTER_GI_TEMPORAL_HISTORY_COMBINED, id.x);
        combinedPrevious = true;
        combinedPrevSurface = hdPrev;
        combinedPrevCandidate = prevCandidate;
        combinedPrevTargetPdf = prevTargetLumCur;
        combinedPrevOriginalProposalPdf = prev.proposalPdf;
        combinedPrevJacobian = jacobian;
        if (candidateSelected)
        {
            RestirTelemetryCount(RESTIR_COUNTER_GI_TEMPORAL_HISTORY_SELECTED, id.x);
            selectedPrevious = true;
            selectedTargetPdf = prevTargetLumCur;
            selectedPrevSurface = hdPrev;
            selectedPrevCandidate = prevCandidate;
            selectedPrevTargetPdf = prevTargetLumCur;
            selectedPrevOriginalProposalPdf = prev.proposalPdf;
            selectedPrevJacobian = jacobian;
            selectedTemporalOffsetIdx = temporalOffsetIdx;
            selectedFallbackSample = isFallbackSample;
        }

        break;
    }

    // The loop combines at most one history reservoir, whose M is capped to MAX-1.
    // Together with the current sample (M <= 1), outR is bounded by MAX by construction.
    float pi = selectedTargetPdf;
    float piSum = curTargetPdf * max(cur.sampleCount, 1.0);
    float temporalP = 0.0;
    if (selectedTargetPdf > 0.0 && combinedPrevious)
    {
        IndirectReservoirData selectedSample = outR;
        float3 selectedRadiancePrev;
        float3 selectedContributionPrev;
        float selectedTargetLumPrev;
        temporalP = ReevaluateIndirectReservoirAtSurface(combinedPrevSurface, selectedSample, selectedRadiancePrev, selectedContributionPrev, selectedTargetLumPrev)
            ? selectedTargetLumPrev
            : 0.0;
        if (temporalP > 0.0 && !IsIndirectSampleVisibleAtSurface(combinedPrevSurface, selectedSample))
            temporalP = 0.0;
    }
    pi = selectedPrevious ? temporalP : selectedTargetPdf;
    piSum += temporalP * max(combinedPrevCandidate.sampleCount, 0.0);

    float normalizationNumerator = pi;
    float normalizationDenominator = selectedTargetPdf * piSum;
    FinalizeIndirectReservoir(outR, normalizationNumerator, normalizationDenominator);
    bool outputFinite = all(isfinite(outR.secondaryPosition)) &&
        all(isfinite(outR.secondaryNormal)) && all(isfinite(outR.radiance)) &&
        all(isfinite(outR.contribution)) &&
        isfinite(outR.proposalPdf) && isfinite(outR.targetLum) &&
        isfinite(outR.weightSum) && isfinite(outR.selectedWeight) && isfinite(outR.sampleCount);
    if (!outputFinite)
    {
        RestirTelemetryCount(RESTIR_COUNTER_GI_TEMPORAL_NONFINITE_OUTPUT, id.x);
        RestirTelemetryCountCritical(RESTIR_COUNTER_CRITICAL_NONFINITE);
    }
    if (id.x == RestirTelemetrySelectedPixel())
        WriteIndirectReservoirTelemetry(
            1u,
            RESTIR_STAGE_GI_TEMPORAL,
            outputFinite ? RESTIR_REASON_NONE : RESTIR_REASON_NONFINITE_RESERVOIR,
            id.x,
            outR,
            float4(selectedPrevious ? 1.0 : 0.0, selectedTargetPdf, piSum, normalizationDenominator));
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
        ReSTIRDebugData[3] = float4(
            combinedPrevious ? 1.0 : 0.0,
            combinedPrevTargetPdf,
            combinedPrevOriginalProposalPdf,
            combinedPrevCandidate.proposalPdf);
        ReSTIRDebugData[4] = float4(
            combinedPrevJacobian,
            temporalP,
            combinedPrevCandidate.sampleCount,
            0.0);
    }
    IndirectReservoirs[outIdx] = outR;
}
