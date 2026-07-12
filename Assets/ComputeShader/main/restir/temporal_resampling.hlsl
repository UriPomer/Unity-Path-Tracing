#pragma once

// Depends on: global.hlsl, trace.hlsl, bxdf.hlsl, reservoir.hlsl

bool ReevaluatePrevReservoir(
    HitData hd,
    DirectLightReservoirData prev,
    out DirectLightSample result)
{
    RayHit hit;
    hit.position = hd.position; hit.distance = hd.distance;
    hit.normal = hd.normal; hit.mode = hd.mode;
    hit.material.albedo = hd.albedo; hit.material.emission = hd.emission;
    hit.material.emissionIntensity = hd.emissionIntensity;
    hit.material.roughness = hd.roughness; hit.material.metallic = hd.metallic;
    hit.material.alpha = hd.alpha; hit.material.ior = hd.ior;
    hit.should_break = false;
    float3 cameraPos = float3(_CameraToWorld._m03, _CameraToWorld._m13, _CameraToWorld._m23);
    float3 V = normalize(cameraPos - hd.position);

    float sourcePdf = max(prev.proposalPdf, 1e-6);
    DirectLightSample temp = (DirectLightSample)0;
    bool ok = false;
    if (prev.lightType == 1u)
        ok = ReevaluateSunDirectLightSample(hit, V, float3(1,1,1), prev.direction, sourcePdf, temp);
    else if (prev.lightType == 2u)
    {
        float3 sp = prev.origin + prev.direction * prev.maxDist;
        ok = ReevaluatePointLightDirectSample(hit, V, float3(1,1,1), prev.lightIndex, sp, sourcePdf, temp);
    }
    if (ok) result = temp;
    else result = (DirectLightSample)0;
    return ok;
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
    if (!IsReservoirValid(cur))
    {
        RestirTelemetryCount(RESTIR_COUNTER_DI_TEMPORAL_INVALID_CURRENT, id.x);
        return;
    }

    HitData hdCur = _RestirGbuffer[id.x];
    if (hdCur.distance >= 1e19)
    {
        RestirTelemetryCount(RESTIR_COUNTER_DI_TEMPORAL_INVALID_CURRENT, id.x);
        return;
    }

    // Motion-vector reprojection
    float4 prevClip = mul(_RestirPreviousViewProjection, float4(hdCur.position, 1.0));
    if (prevClip.w <= 1e-6)
    {
        RestirTelemetryCount(RESTIR_COUNTER_DI_TEMPORAL_REPROJECTION_OOB, id.x);
        return;
    }
    float2 prevUV = prevClip.xy / prevClip.w;
    if (any(prevUV < -1.0) || any(prevUV > 1.0))
    {
        RestirTelemetryCount(RESTIR_COUNTER_DI_TEMPORAL_REPROJECTION_OOB, id.x);
        return;
    }
    float2 prevScreen = prevUV * 0.5 + 0.5;
    int2 prevPx = clamp(
        int2(prevScreen * float2(_ScreenWidth, _ScreenHeight)),
        int2(0, 0),
        int2((int)_ScreenWidth - 1, (int)_ScreenHeight - 1));
    uint prevPxIdx = (uint)(prevPx.y * (int)_ScreenWidth + prevPx.x);

    DirectLightReservoirData prev = DirectLightReservoirs[_RestirPrevReservoirOffset + prevPxIdx];
    if (!IsReservoirValid(prev))
    {
        RestirTelemetryCount(RESTIR_COUNTER_DI_TEMPORAL_INVALID_HISTORY, id.x);
        return;
    }
    HitData hdPrev = _RestirGbufferPrevious[prevPxIdx];
    if (hdPrev.distance >= 1e19)
    {
        RestirTelemetryCount(RESTIR_COUNTER_DI_TEMPORAL_INVALID_HISTORY, id.x);
        return;
    }

    // Compatibility check
    float3 prevN = hdPrev.normal;
    float3 curN = hdCur.normal;
    if (dot(curN, cur.surfaceNormal) < 0.0) curN = -curN;
    if (dot(prevN, prev.surfaceNormal) < 0.0) prevN = -prevN;
    if (!IsTemporalCompatible(hdCur.position, curN, hdCur.mode,
                              hdPrev.position, prevN, hdPrev.mode))
    {
        RestirTelemetryCount(RESTIR_COUNTER_DI_TEMPORAL_INCOMPATIBLE, id.x);
        return;
    }

    // Re-evaluate prev reservoir's sample at current surface
    DirectLightSample prevSample = (DirectLightSample)0;
    if (!ReevaluatePrevReservoir(hdCur, prev, prevSample))
    {
        RestirTelemetryCount(RESTIR_COUNTER_DI_TEMPORAL_REEVALUATION_REJECTED, id.x);
        return;
    }
    if (!IsValidDirectLightSample(prevSample))
    {
        RestirTelemetryCount(RESTIR_COUNTER_DI_TEMPORAL_REEVALUATION_REJECTED, id.x);
        return;
    }

    // Rebuild the history candidate's effective weight on the current surface.
    // Using the previous surface's raw weightSum here biases selection toward
    // stale history and freezes the temporal pattern.
    float curW = cur.weightSum;
    float prevW = prev.selectedWeight * prevSample.targetLum * max((float)prev.sampleCount, 1.0);
    float combinedWS = curW + prevW;
    uint combinedSC = cur.sampleCount + max(prev.sampleCount, 1u);

    uint2 pixel = uint2(id.x % _ScreenWidth, id.x / _ScreenWidth);
    RNG_SeedPixel(rng, pixel, _FrameCount);

    DirectLightReservoirData outR = cur;
    bool selectedPrevious = false;
    if (RNG_Next(rng) * combinedWS < prevW)
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
    outR.surfaceNormal = curN;
    outR.selectedWeight = ComputeMISWeight(combinedWS, outR.targetLum, combinedSC);
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
    RestirTelemetryWriteRecord(
        5u,
        RESTIR_STAGE_DI_TEMPORAL,
        outputFinite ? RESTIR_REASON_NONE : RESTIR_REASON_NONFINITE_RESERVOIR,
        id.x,
        float4(outR.origin, outR.maxDist),
        float4(outR.direction, outR.targetLum),
        float4(outR.contribution, outR.weightSum),
        float4(outR.surfaceNormal, outR.proposalPdf),
        float4(selectedPrevious ? 1.0 : 0.0, curW, prevW, combinedWS),
        float4((float)combinedSC, outR.selectedWeight, (float)prevPx.x, (float)prevPx.y));

    if (id.x == _RestirDebugPixelIndex)
    {
        ReSTIRDebugData[0] = float4(
            selectedPrevious ? 1.0 : 0.0,
            curW,
            prevW,
            combinedWS);
    }
}
