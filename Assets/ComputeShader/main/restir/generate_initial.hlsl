#pragma once

// Depends on: global.hlsl, trace.hlsl, bxdf.hlsl, function.hlsl, reservoir.hlsl

StructuredBuffer<HitData> _RestirGbuffer;
StructuredBuffer<LightDataPacked> _RestirLightData;
uint _RestirLightCount;
uint _RestirInitialReservoirOffset;
uint _RestirCandidateCount;

[numthreads(64, 1, 1)]
void kernel_generate_initial(uint3 id : SV_DispatchThreadID)
{
    uint pixelCount = _ScreenWidth * _ScreenHeight;
    if (id.x >= pixelCount) return;
    DirectLightReservoirs[_RestirInitialReservoirOffset + id.x] = (DirectLightReservoirData)0;

    HitData hd = _RestirGbuffer[id.x];
    if (hd.distance >= 1e19) return;

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

    // Find total CDF for light selection
    float totalCdf = 0.0;
    for (uint li = 0u; li < _RestirLightCount; li++)
    {
        float c = _RestirLightData[li].cdf;
        if (c > totalCdf) totalCdf = c;
    }

    if (totalCdf <= 0.0) return;

    for (uint i = 0u; i < cCount; i++)
    {
        // Weighted light selection via CDF binary search (linear scan for small N)
        float rnd = RNG_Next(rng) * totalCdf;
        uint lightIdx = 0u;
        for (uint si = 0u; si < _RestirLightCount; si++)
        {
            if (_RestirLightData[si].cdf >= rnd && _RestirLightData[si].power > 0.0)
            {
                lightIdx = si;
                break;
            }
            lightIdx = si;
        }

        LightDataPacked light = _RestirLightData[lightIdx];
        if (light.power <= 0.0) continue;

        float lightPickPdf = light.power / max(totalCdf, 1e-6);
        float proposalPdf = lightPickPdf;

        DirectLightSample s = (DirectLightSample)0;
        bool ok = false;
        if (light.lightType == 1u)
            ok = BuildSunDirectLightSample(hit, V, float3(1,1,1), proposalPdf, s);
        else if (light.lightType == 2u)
            ok = BuildPointLightDirectSample(hit, V, float3(1,1,1), light.originalIndex, proposalPdf, s);
        if (!ok || !IsValidDirectLightSample(s)) continue;

        float w = s.targetLum / max(s.proposalPdf, 1e-6);
        weightSum += w;
        if (!hasSelected || RNG_Next(rng) * weightSum < w)
        {
            selected = s;
            hasSelected = true;
        }
    }

    if (!hasSelected) return;

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
}
