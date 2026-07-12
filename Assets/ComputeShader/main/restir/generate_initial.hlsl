#pragma once

// Depends on: global.hlsl, trace.hlsl, bxdf.hlsl, function.hlsl, reservoir.hlsl

[numthreads(64, 1, 1)]
void kernel_generate_initial(uint3 id : SV_DispatchThreadID)
{
    uint pixelCount = _ScreenWidth * _ScreenHeight;
    if (id.x >= pixelCount) return;
    DirectLightReservoirs[_RestirInitialReservoirOffset + id.x] = (DirectLightReservoirData)0;

    HitData hd = _RestirGbuffer[id.x];
    if (hd.distance >= 1e19)
    {
        RestirTelemetryCount(RESTIR_COUNTER_DI_INITIAL_INVALID_SURFACE, id.x);
        WriteDirectReservoirTelemetry(
            4u, RESTIR_STAGE_DI_INITIAL, RESTIR_REASON_INVALID_SURFACE, id.x,
            (DirectLightReservoirData)0, 0.0, 0.0);
        return;
    }

    uint2 pixel = uint2(id.x % _ScreenWidth, id.x / _ScreenWidth);
    RNG_SeedPixel(rng, pixel, _FrameCount);
    _Pixel = pixel;

    RayHit hit;
    hit.position = hd.position; hit.distance = hd.distance;
    hit.normal = hd.normal; hit.mode = hd.mode;
    hit.material.albedo = hd.albedo; hit.material.emission = hd.emission;
    hit.material.emissionIntensity = hd.emissionIntensity;
    hit.material.roughness = hd.roughness;
    hit.material.metallic = hd.metallic;
    hit.material.alpha = hd.alpha; hit.material.ior = hd.ior;
    hit.should_break = false;

    float3 cameraPos = _CameraToWorld._m03_m13_m23;
    float3 V = normalize(cameraPos - hd.position);
    float3 surfaceNormal = GetDirectLightSurfaceNormal(hit, V);

    DirectLightSample selected = (DirectLightSample)0;
    float weightSum = 0.0;
    bool hasSelected = false;
    uint cCount = max(_RestirCandidateCount, 1u);
    DirectLightReservoirData emptyReservoir = (DirectLightReservoirData)0;
    emptyReservoir.surfaceNormal = surfaceNormal;
    emptyReservoir.sampleCount = cCount;
    DirectLightReservoirs[_RestirInitialReservoirOffset + id.x] = emptyReservoir;

    // Use the same candidate domain as the regular wavefront direct-light path.
    // ReSTIR changes how candidates are retained, not which lights are eligible.
    bool hasSun = _DirectionalLightColor.a > 0.0;
    uint pointLightCount;
    uint pointLightOffset;
    bool useCulledList;
    bool hasPointLights = GetPointLightCandidateRange(pointLightCount, pointLightOffset, useCulledList);
    uint candidatePoolCount = (hasSun ? 1u : 0u) + (hasPointLights ? pointLightCount : 0u);
    if (candidatePoolCount == 0u)
    {
        RestirTelemetryCount(RESTIR_COUNTER_DI_INITIAL_INVALID_PROPOSAL, id.x);
        WriteDirectReservoirTelemetry(
            4u, RESTIR_STAGE_DI_INITIAL, RESTIR_REASON_INVALID_PROPOSAL_PDF, id.x,
            emptyReservoir, 0.0, float4(0.0, (float)cCount, 0.0, 0.0));
        return;
    }
    float proposalPdf = rcp((float)candidatePoolCount);

    for (uint i = 0u; i < cCount; i++)
    {
        uint candidateIndex = min(uint(RNG_Next(rng) * candidatePoolCount), candidatePoolCount - 1u);
        DirectLightSample s = (DirectLightSample)0;
        bool ok = SampleDirectLightCandidate(
            hit,
            V,
            float3(1, 1, 1),
            hasSun,
            pointLightOffset,
            useCulledList,
            candidateIndex,
            proposalPdf,
            s);
        if (!ok || !IsValidDirectLightSample(s)) continue;

        float w = s.targetLum / max(s.proposalPdf, 1e-6);
        weightSum += w;
        if (!hasSelected || RNG_Next(rng) * weightSum < w)
        {
            selected = s;
            hasSelected = true;
        }
    }

    if (!hasSelected)
    {
        RestirTelemetryCount(RESTIR_COUNTER_DI_INITIAL_INVALID_PROPOSAL, id.x);
        WriteDirectReservoirTelemetry(
            4u, RESTIR_STAGE_DI_INITIAL, RESTIR_REASON_ZERO_TARGET, id.x,
            emptyReservoir, 0.0, float4((float)candidatePoolCount, (float)cCount, 0.0, 0.0));
        return;
    }

    DirectLightReservoirData r;
    r.origin = selected.origin;
    r.maxDist = selected.maxDist;
    r.direction = selected.direction;
    r.targetLum = selected.targetLum;
    r.contribution = selected.contribution;
    r.weightSum = weightSum;
    r.surfaceNormal = surfaceNormal;
    r.proposalPdf = selected.proposalPdf;
    r.lightType = selected.lightType;
    r.lightIndex = selected.lightIndex;
    r.sampleCount = cCount;
    r.selectedWeight = ComputeMISWeight(weightSum, selected.targetLum, cCount);
    DirectLightReservoirs[_RestirInitialReservoirOffset + id.x] = r;
    RestirTelemetryCount(RESTIR_COUNTER_DI_INITIAL_ACCEPTED, id.x);
    WriteDirectReservoirTelemetry(
        4u, RESTIR_STAGE_DI_INITIAL, RESTIR_REASON_NONE, id.x, r,
        float4((float)r.lightType, (float)r.lightIndex, (float)r.sampleCount, r.selectedWeight),
        float4((float)candidatePoolCount, (float)cCount, 0.0, 0.0));
}
