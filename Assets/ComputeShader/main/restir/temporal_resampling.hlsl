#pragma once

// Depends on: global.hlsl, trace.hlsl, bxdf.hlsl, reservoir.hlsl

bool ReevaluatePrevReservoir(
    HitData hd,
    DirectLightReservoirData prev,
    float3 cameraPos,
    out DirectLightSample result)
{
    RayHit hit;
    hit.position = hd.position; hit.distance = hd.distance;
    hit.normal = hd.normal;
    hit.geometryNormal = hd.geometryNormal; hit.mode = hd.mode;
    hit.material.albedo = hd.albedo; hit.material.emission = hd.emission;
    hit.material.emissionIntensity = hd.emissionIntensity;
    hit.material.roughness = hd.roughness; hit.material.metallic = hd.metallic;
    hit.material.alpha = hd.alpha; hit.material.ior = hd.ior;
    hit.should_break = false;
    float3 V = normalize(cameraPos - hd.position);

    float sourcePdf = prev.proposalPdf;
    DirectLightSample temp = (DirectLightSample)0;
    bool ok = false;
    if (prev.lightType == 1u)
        ok = ReevaluateSunDirectLightSample(hit, V, float3(1,1,1), prev.direction, sourcePdf, temp);
    else if (prev.lightType == 2u)
    {
        PointLightData light = LoadPointLight(prev.lightIndex);
        float3 oldReceiverPosition = prev.receiverPosition;
        float3 oldSamplePosition = prev.origin + prev.direction * prev.maxDist;
        float3 sp = ReprojectPointLightDiskPosition(
            light, oldReceiverPosition, hd.position, oldSamplePosition);
        ok = ReevaluatePointLightDirectSample(hit, V, float3(1,1,1), prev.lightIndex, sp, sourcePdf, temp);
    }
    ok = ok && IsValidDirectLightSample(temp);
    if (ok) result = temp;
    else result = (DirectLightSample)0;
    return ok;
}

bool IsHistoryLightAvailable(DirectLightReservoirData sample)
{
    if (sample.lightType == 1u)
        return _DirectionalLightColor.a > 0.0;
    return sample.lightType == 2u && sample.lightIndex < (uint)max(_PointLightsCount, 0);
}

[numthreads(64, 1, 1)]
void kernel_temporal_resampling(uint3 id : SV_DispatchThreadID)
{
    uint pixelCount = _ScreenWidth * _ScreenHeight;
    if (id.x >= pixelCount) return;

    uint curIdx = _RestirInitialReservoirOffset + id.x;
    uint outIdx = _RestirTemporalReservoirOffset + id.x;

    DirectLightReservoirData cur = DirectLightReservoirs[curIdx];
    DirectLightReservoirs[outIdx] = cur;
    bool currentReservoirValid = IsReservoirValid(cur);
    if (!currentReservoirValid)
        RestirTelemetryCount(RESTIR_COUNTER_DI_TEMPORAL_INVALID_CURRENT, id.x);

    HitData hdCur = _RestirGbuffer[id.x];
    if (hdCur.distance >= 1e19)
    {
        RestirTelemetryCount(RESTIR_COUNTER_DI_TEMPORAL_INVALID_CURRENT, id.x);
        WriteDirectReservoirTelemetry(
            5u, RESTIR_STAGE_DI_TEMPORAL, RESTIR_REASON_INVALID_SURFACE, id.x,
            cur, 0.0, 0.0);
        return;
    }
    if (cur.sampleCount == 0u)
        return;

    uint2 pixel = uint2(id.x % _ScreenWidth, id.x / _ScreenWidth);
    _Pixel = pixel;

    // A sample-independent restart mixes two unbiased estimators. It bounds
    // temporal correlation without clipping radiance or selecting by visibility.
    // A single light has no light-selection reuse benefit. Keep fresh RIS draws
    // there, so its soft-shadow visibility does not become persistent noise.
    uint lightCount = (uint)max(_PointLightsCount,0) + (_DirectionalLightColor.a > 0 ? 1u : 0u);
    RNG_SeedPixel(rng, pixel, _FrameCount, 2u);
    if (RNG_Next(rng) < (lightCount <= 1u ? 1.0 : 0.25)) return;

    // Motion-vector reprojection
    float4 prevClip = mul(_RestirPreviousViewProjection, float4(hdCur.position, 1.0));
    if (prevClip.w <= 1e-6)
    {
        RestirTelemetryCount(RESTIR_COUNTER_DI_TEMPORAL_REPROJECTION_OOB, id.x);
        WriteDirectReservoirTelemetry(
            5u, RESTIR_STAGE_DI_TEMPORAL, RESTIR_REASON_REPROJECTION_OOB, id.x,
            cur, 0.0, 0.0);
        return;
    }
    float2 prevUV = prevClip.xy / prevClip.w;
    if (any(prevUV < -1.0) || any(prevUV > 1.0))
    {
        RestirTelemetryCount(RESTIR_COUNTER_DI_TEMPORAL_REPROJECTION_OOB, id.x);
        WriteDirectReservoirTelemetry(
            5u, RESTIR_STAGE_DI_TEMPORAL, RESTIR_REASON_REPROJECTION_OOB, id.x,
            cur, 0.0, 0.0);
        return;
    }
    float2 prevScreen = prevUV * 0.5 + 0.5;
    int2 prevPx = clamp(
        int2(prevScreen * float2(_ScreenWidth, _ScreenHeight)),
        int2(0, 0),
        int2((int)_ScreenWidth - 1, (int)_ScreenHeight - 1));
    uint prevPxIdx = (uint)(prevPx.y * (int)_ScreenWidth + prevPx.x);

    DirectLightReservoirData prev = DirectLightReservoirs[_RestirPrevReservoirOffset + prevPxIdx];
    if (prev.sampleCount == 0u)
    {
        RestirTelemetryCount(RESTIR_COUNTER_DI_TEMPORAL_INVALID_HISTORY, id.x);
        WriteDirectReservoirTelemetry(
            5u, RESTIR_STAGE_DI_TEMPORAL, RESTIR_REASON_INVALID_HISTORY, id.x,
            cur, 0.0, float4((float)prevPx.x, (float)prevPx.y, 0.0, 0.0));
        return;
    }
    HitData hdPrev = _RestirGbufferPrevious[prevPxIdx];
    if (hdPrev.distance >= 1e19)
    {
        RestirTelemetryCount(RESTIR_COUNTER_DI_TEMPORAL_INVALID_HISTORY, id.x);
        WriteDirectReservoirTelemetry(
            5u, RESTIR_STAGE_DI_TEMPORAL, RESTIR_REASON_INVALID_HISTORY, id.x,
            cur, 0.0, float4((float)prevPx.x, (float)prevPx.y, 0.0, 0.0));
        return;
    }

    // Compatibility check
    float3 prevN = hdPrev.normal;
    float3 curN = hdCur.normal;
    float3 cameraPosition = _CameraToWorld._m03_m13_m23;
    if (dot(curN, cameraPosition - hdCur.position) < 0.0) curN = -curN;
    if (dot(prevN, _RestirPreviousCameraPosition - hdPrev.position) < 0.0) prevN = -prevN;
    if (!IsTemporalCompatible(hdCur.position, curN, hdCur.mode,
                              hdPrev.position, prevN, hdPrev.mode))
    {
        RestirTelemetryCount(RESTIR_COUNTER_DI_TEMPORAL_INCOMPATIBLE, id.x);
        WriteDirectReservoirTelemetry(
            5u, RESTIR_STAGE_DI_TEMPORAL, RESTIR_REASON_INCOMPATIBLE_SURFACE, id.x,
            cur, 0.0, float4((float)prevPx.x, (float)prevPx.y, 0.0, 0.0));
        return;
    }

    // A zero-weight history still represents its proposal draws in the MIS
    // denominator. Only its candidate weight is zero.
    DirectLightSample prevSample = (DirectLightSample)0;
    bool previousCandidateValid = IsReservoirValid(prev) &&
        IsHistoryLightAvailable(prev) &&
        ReevaluatePrevReservoir(hdCur, prev, cameraPosition, prevSample) &&
        IsValidDirectLightSample(prevSample);
    if (!previousCandidateValid)
        RestirTelemetryCount(RESTIR_COUNTER_DI_TEMPORAL_REEVALUATION_REJECTED, id.x);

    // Rebuild the history candidate's effective weight on the current surface.
    // Using the previous surface's raw weightSum here biases selection toward
    // stale history and freezes the temporal pattern.
    uint currentM = min(cur.sampleCount, RESTIR_DI_MAX_RESERVOIR_SAMPLES);
    uint previousM = min(
        max(prev.sampleCount, 1u),
        RESTIR_DI_MAX_RESERVOIR_SAMPLES - currentM);
    if (previousM == 0u)
    {
        WriteDirectReservoirTelemetry(
            5u, RESTIR_STAGE_DI_TEMPORAL, RESTIR_REASON_INVALID_HISTORY, id.x,
            cur, 0.0, float4((float)prevPx.x, (float)prevPx.y, 0.0, 0.0));
        return;
    }
    if (prev.sampleCount > previousM)
        RestirTelemetryCount(RESTIR_COUNTER_DI_TEMPORAL_M_CAPPED, id.x);

    float curW = currentReservoirValid ? cur.weightSum : 0.0;
    float prevW = previousCandidateValid
        ? prev.selectedWeight * prevSample.targetLum * (float)previousM : 0.0;
    float combinedWS = curW + prevW;
    uint combinedSC = currentM + previousM;


    DirectLightReservoirData outR = (DirectLightReservoirData)0;
    if (currentReservoirValid)
        outR = cur;
    bool selectedPrevious = false;
    if (prevW > 0.0 && (!currentReservoirValid || RNG_Next(rng) * combinedWS < prevW))
    {
        selectedPrevious = true;
        outR.origin = prevSample.origin;
        outR.maxDist = prevSample.maxDist;
        outR.direction = prevSample.direction;
        outR.targetLum = prevSample.targetLum;
        outR.contribution = prevSample.contribution;
        outR.proposalPdf = prevSample.proposalPdf;
        outR.lightType = prevSample.lightType;
        outR.lightIndex = prevSample.lightIndex;
    }
    outR.weightSum = combinedWS;
    outR.sampleCount = combinedSC;
    outR.receiverPosition = hdCur.position;

    float selectedTargetPdf = outR.targetLum;
    DirectLightSample selectedAtPrevious = (DirectLightSample)0;
    float temporalP = selectedTargetPdf > 0.0 &&
        ReevaluatePrevReservoir(hdPrev, outR, _RestirPreviousCameraPosition, selectedAtPrevious)
        ? selectedAtPrevious.targetLum : 0.0;
    float pi = selectedPrevious ? temporalP : selectedTargetPdf;
    float piSum = selectedTargetPdf * (float)currentM + temporalP * (float)previousM;
    outR.selectedWeight = ComputeDirectBiasCorrectedWeight(combinedWS, selectedTargetPdf, pi, piSum);
    DirectLightReservoirs[outIdx] = outR;
    RestirTelemetryCount(RESTIR_COUNTER_DI_TEMPORAL_HISTORY_COMBINED, id.x);
    if (selectedPrevious)
        RestirTelemetryCount(RESTIR_COUNTER_DI_TEMPORAL_HISTORY_SELECTED, id.x);

    bool outputFinite = isfinite(outR.weightSum) && isfinite(outR.selectedWeight) &&
        all(isfinite(outR.origin)) && all(isfinite(outR.direction)) && all(isfinite(outR.contribution));
    if (!outputFinite)
    {
        RestirTelemetryCount(RESTIR_COUNTER_DI_TEMPORAL_NONFINITE_OUTPUT, id.x);
        RestirTelemetryCountCritical(RESTIR_COUNTER_CRITICAL_NONFINITE);
    }
    WriteDirectReservoirTelemetry(
        5u, RESTIR_STAGE_DI_TEMPORAL,
        outputFinite ? RESTIR_REASON_NONE : RESTIR_REASON_NONFINITE_RESERVOIR,
        id.x, outR,
        float4(selectedPrevious ? 1.0 : 0.0, curW, prevW, combinedWS),
        float4((float)combinedSC, outR.selectedWeight, pi, piSum));

    if (id.x == _RestirDebugPixelIndex)
    {
        ReSTIRDebugData[0] = float4(
            selectedPrevious ? 1.0 : 0.0,
            curW,
            prevW,
            combinedWS);
        ReSTIRDebugData[1] = float4(pi, piSum, (float)currentM, (float)previousM);
    }
}
