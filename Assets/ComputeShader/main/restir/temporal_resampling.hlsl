#pragma once

// Depends on: global.hlsl, trace.hlsl, bxdf.hlsl, reservoir.hlsl

uint _RestirTemporalReservoirOffset;
uint _RestirPrevReservoirOffset;
StructuredBuffer<HitData> _RestirGbufferPrevious;
float4x4 _RestirPreviousViewProjection;

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
    if (!IsReservoirValid(cur)) return;

    HitData hdCur = _RestirGbuffer[id.x];
    if (hdCur.distance >= 1e19) return;

    // Motion-vector reprojection
    float4 prevClip = mul(_RestirPreviousViewProjection, float4(hdCur.position, 1.0));
    if (prevClip.w <= 1e-6) return;
    float2 prevUV = prevClip.xy / prevClip.w;
    if (any(prevUV < -1.0) || any(prevUV > 1.0)) return;
    float2 prevScreen = prevUV * 0.5 + 0.5;
    int2 prevPx = clamp(
        int2(prevScreen * float2(_ScreenWidth, _ScreenHeight)),
        int2(0, 0),
        int2((int)_ScreenWidth - 1, (int)_ScreenHeight - 1));
    uint prevPxIdx = (uint)(prevPx.y * (int)_ScreenWidth + prevPx.x);

    DirectLightReservoirData prev = DirectLightReservoirs[_RestirPrevReservoirOffset + prevPxIdx];
    if (!IsReservoirValid(prev)) return;
    HitData hdPrev = _RestirGbufferPrevious[prevPxIdx];
    if (hdPrev.distance >= 1e19) return;

    // Compatibility check
    float3 prevN = hdPrev.normal;
    float3 curN = hdCur.normal;
    if (dot(curN, cur.surfaceNormal) < 0.0) curN = -curN;
    if (dot(prevN, prev.surfaceNormal) < 0.0) prevN = -prevN;
    if (!IsTemporalCompatible(hdCur.position, curN, hdCur.mode,
                              hdPrev.position, prevN, hdPrev.mode))
        return;

    // Re-evaluate prev reservoir's sample at current surface
    DirectLightSample prevSample = (DirectLightSample)0;
    if (!ReevaluatePrevReservoir(hdCur, prev, prevSample)) return;
    if (!IsValidDirectLightSample(prevSample)) return;

    // MIS combine: M_cur * W_cur + M_prev * W_prev
    float curW = cur.weightSum * (float)max(cur.sampleCount, 1u);
    float prevW = prev.weightSum * (float)max(prev.sampleCount, 1u);
    float combinedWS = curW + prevW;
    uint combinedSC = cur.sampleCount + max(prev.sampleCount, 1u);

    uint2 pixel = uint2(id.x % _ScreenWidth, id.x / _ScreenWidth);
    RNG_SeedPixel(rng, pixel, _FrameCount);

    DirectLightReservoirData outR = cur;
    if (RNG_Next(rng) * combinedWS < prevW)
    {
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
}
