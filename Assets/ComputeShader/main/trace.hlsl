#pragma once

#include "global.hlsl"
#include "bxdf.hlsl"

float3 SampleSkybox(Ray ray)
{
    float3 dir = normalize(ray.dir);
    float theta = acos(dir.y) / -PI;
    float phi   = atan2(dir.x, -dir.z) / -PI * 0.5;
    float3 envColor = _SkyboxTexture.SampleLevel(
        sampler_SkyboxTexture,
        float2(phi, theta),
        0
    ).xyz;
    //
    // float3 sunDir = normalize(_InverseDirectionalLight);
    // float cosA = saturate(dot(dir, sunDir));
    // float disk = pow(cosA, 50);
    // float3 sunColor = disk
    //                 * _DirectionalLightColor.rgb
    //                 * _DirectionalLightColor.a;

    return envColor * _SkyboxIntensity;
}

// trace a ray and returns hit immediately (for shadow rays)
bool TraceHit(Ray ray, float targetDist)
{
    return IntersectTlasFast(ray, targetDist);
}

// trace a ray and detect nearest hit
RayHit Trace(Ray ray)
{
    RayHit bestHit = GenRayHit();
    IntersectTlas(ray, bestHit);
    return bestHit;
}

#define DIRECTIONAL_LIGHT_SAMPLE 1
#define POINT_LIGHT_SAMPLES 1

float2 SampleDisk(float u1, float u2)
{
    float r     = sqrt(u1);
    float theta = 2.0 * PI * u2;
    return float2(r * cos(theta), r * sin(theta));
}

struct PointLightData
{
    float3 position;
    float  range;
    float3 color;
    float  intensity;
    float  sourceRadius;
};

PointLightData LoadPointLight(uint lightIdx)
{
    float4 lightPosRange = _PointLights[lightIdx * 3];
    float4 lightColorIntensity = _PointLights[lightIdx * 3 + 1];
    float4 lightMeta = _PointLights[lightIdx * 3 + 2];

    PointLightData light;
    light.position = lightPosRange.xyz;
    light.range = lightPosRange.w;
    light.color = lightColorIntensity.rgb;
    light.intensity = lightColorIntensity.a;
    light.sourceRadius = lightMeta.x;
    return light;
}

float GetPointLightRangeAttenuation(float distanceToLight, float lightRange)
{
    if (lightRange <= 0.0 || distanceToLight >= lightRange)
        return 0.0;

    float x = saturate(distanceToLight / lightRange);
    float fade = 1.0 - x * x * x * x;
    return fade * fade;
}

float3 SampleSunLightOnce(RayHit hit, float3 viewDir, float3 lightDir, float3 lightColor)
{
    float3 up = abs(lightDir.y) < 0.99 ? float3(0,1,0) : float3(1,0,0);
    float3 right = normalize(cross(up, lightDir));
    float3 up2 = cross(lightDir, right);
    float2 d = SampleDisk(RNG_Next(rng), RNG_Next(rng)) * _SunAngularRadius;
    float3 sampleDir = normalize(lightDir + d.x * right + d.y * up2);

    Ray shadowRay = GenRay(hit.position + hit.normal * 1e-5, sampleDir);
    if (TraceHit(shadowRay, 1e20))
        return 0;

    float NdotL = saturate(dot(hit.normal, sampleDir));
    if (NdotL <= 0.0)
        return 0;

    float3 f_brdf;
    float dummyPdf;
    EvaluateBXDF_GivenDir(hit, viewDir, sampleDir, /*out*/ f_brdf, /*out*/ dummyPdf);

    float cosA = saturate(dot(viewDir, sampleDir));
    float diskV = pow(cosA, _SunFocus);
    return diskV * lightColor * f_brdf * NdotL;
}

float3 AccumulateSunLight(
    RayHit hit,
    float3 viewDir
)
{
    if (_DirectionalLightColor.a <= 0.0)
        return 0;

    float3 L0 = normalize(_InverseDirectionalLight);
    float3 color = _DirectionalLightColor.rgb * _DirectionalLightColor.a;

#if DIRECTIONAL_LIGHT_SAMPLE == 1
    return SampleSunLightOnce(hit, viewDir, L0, color);
#else
    float3 accum = float3(0,0,0);
    for (int i = 0; i < DIRECTIONAL_LIGHT_SAMPLE; ++i)
        accum += SampleSunLightOnce(hit, viewDir, L0, color);
    return accum / float(DIRECTIONAL_LIGHT_SAMPLE);
#endif
}

float3 AccumulatePointLightSoft(
    RayHit hit,
    float3 V,
    uint   lightIdx,
    float  inv_select_pdf
)
{
    PointLightData light = LoadPointLight(lightIdx);
    if (light.intensity <= 0.0 || light.range <= 0.0) return 0;

    float3 toCenter= light.position - hit.position;
    float  distC   = length(toCenter);
    float rangeAtten = GetPointLightRangeAttenuation(distC, light.range);
    if (rangeAtten <= 0.0) return 0;
    float3 L0      = toCenter / max(distC, 1e-6);

    float3 upRef = abs(L0.y) < 0.99 ? float3(0,1,0) : float3(1,0,0);
    float3 right = normalize(cross(upRef, L0));
    float3 up2   = cross(L0, right);

    float3 Le = light.color * light.intensity;
    float radius = max(light.sourceRadius, 0.0);
    float3 accum = 0;

    [unroll]
    for (int s = 0; s < POINT_LIGHT_SAMPLES; ++s)
    {
        float3 samplePos = light.position;
        if (radius > 1e-4)
        {
            float2 d  = SampleDisk(RNG_Next(rng), RNG_Next(rng)) * radius;
            samplePos = light.position + d.x * right + d.y * up2;
        }

        float3 toSample = samplePos - hit.position;
        float  distS    = length(toSample);
        float3 Ls       = toSample / max(distS, 1e-6);

        if (TraceHit(GenRay(hit.position + hit.normal * 1e-5, Ls), distS))
            continue;

        float3 f_brdf; float dummyPdf;
        EvaluateBXDF_GivenDir(hit, V, Ls, /*out*/ f_brdf, /*out*/ dummyPdf);
        float NdotL = saturate(dot(hit.normal, Ls));
        if (NdotL <= 0.0) continue;

        float3 contrib;
        if (radius <= 1e-4)
        {
            contrib = Le * (f_brdf * NdotL) / max(distS * distS, 1e-6);
        }
        else
        {
            float pdf_area = 1.0 / (PI * radius * radius);
            float3 nLight = normalize(hit.position - light.position);
            float  cosThetaPrime = saturate(dot(nLight, -Ls));
            if (cosThetaPrime <= 1e-6) continue;

            float geom = cosThetaPrime / max(distS * distS, 1e-6);
            contrib = Le * (f_brdf * NdotL) * (geom / pdf_area);
        }

        contrib *= rangeAtten;
        accum += contrib * inv_select_pdf;
    }

    return accum / float(POINT_LIGHT_SAMPLES);
}
uint2 GetTileIndex(uint2 pixelCoord)
{
    return pixelCoord / TILE_SIZE;
}

float3 GetDirectLightContribution(RayHit hit, float3 V)
{
    float3 lightContribution = AccumulateSunLight(hit, V);

    if (_PointLightsCount <= 0)
        return lightContribution;

    uint lightCount = (uint)_PointLightsCount;
    uint lightOffset = 0;
    bool useCulledList = false;

    bool useLightCulling = _TileCount.x > 0 && _TileCount.y > 0;
    if (useLightCulling)
    {
        uint2 tileIndex = GetTileIndex((uint2)_Pixel);
        if (tileIndex.x < _TileCount.x && tileIndex.y < _TileCount.y)
        {
            uint tileId = tileIndex.y * _TileCount.x + tileIndex.x;
            uint2 tileData = _TileData[tileId];
            if (tileData.x > 0)
            {
                lightCount = tileData.x;
                lightOffset = tileData.y;
                useCulledList = true;
            }
            else
            {
                return lightContribution;
            }
        }
    }

#if POINT_LIGHT_SAMPLES == 1
    float u = RNG_Next(rng);
    uint sampleIndex = min(uint(u * lightCount), lightCount - 1);
    uint lightIndex = useCulledList ? _LightCullingData[lightOffset + sampleIndex] : sampleIndex;
    lightContribution += AccumulatePointLightSoft(hit, V, lightIndex, (float)lightCount);
#else
    int sampleCount = min(POINT_LIGHT_SAMPLES, (int)lightCount);
    float inv_select_pdf = lightCount / float(sampleCount);
    for (int i = 0; i < sampleCount; ++i)
    {
        float u = RNG_Next(rng);
        uint sampleIndex = min(uint(u * lightCount), lightCount - 1);
        uint lightIndex = useCulledList ? _LightCullingData[lightOffset + sampleIndex] : sampleIndex;
        lightContribution += AccumulatePointLightSoft(hit, V, lightIndex, inv_select_pdf);
    }
#endif

    return lightContribution;
}

void GenerateShadowRays(RayHit hit, float3 V, float3 throughput, uint pixelIndex)
{
    bool hasSun = _DirectionalLightColor.a > 0.0;
    bool hasPointLights = _PointLightsCount > 0;
    if (!hasSun && !hasPointLights) return;

    if (hasSun)
    {
        // Sun shadow ray
        float3 L0 = normalize(_InverseDirectionalLight);
        float3 up = abs(L0.y) < 0.99 ? float3(0,1,0) : float3(1,0,0);
        float3 right = normalize(cross(up, L0));
        float3 up2 = cross(L0, right);
        float2 d = SampleDisk(RNG_Next(rng), RNG_Next(rng)) * _SunAngularRadius;
        float3 sampleDir = normalize(L0 + d.x * right + d.y * up2);
        float NdotL = saturate(dot(hit.normal, sampleDir));
        if (NdotL > 0.0)
        {
            float3 f_brdf;
            float dummyPdf;
            EvaluateBXDF_GivenDir(hit, V, sampleDir, /*out*/ f_brdf, /*out*/ dummyPdf);

            float3 color = _DirectionalLightColor.rgb * _DirectionalLightColor.a;
            float diskV = pow(saturate(dot(V, sampleDir)), _SunFocus);
            float3 illum = throughput * diskV * color * f_brdf * NdotL;

            uint idx;
            InterlockedAdd(BufferSizes[CurBounce].shadowRays, 1, idx);
            ShadowRayData sr;
            sr.origin = hit.position + hit.normal * 1e-5;
            sr.direction = sampleDir;
            sr.maxDist = 1e20;
            sr.illumination = illum;
            sr.pixelIndex = pixelIndex;
            ShadowRaysBuffer[idx] = sr;
        }
    }

    if (hasPointLights)
    {
        // Sample one point light every bounce, matching the pre-wavefront direct-light path.
        bool useLightCulling = _TileCount.x > 0 && _TileCount.y > 0;
        uint lightIdx;
        float invSelectPdf;

        if (useLightCulling)
        {
            uint2 tileIndex = GetTileIndex((uint2)_Pixel);
            if (tileIndex.x < _TileCount.x && tileIndex.y < _TileCount.y)
            {
                uint tileId = tileIndex.y * _TileCount.x + tileIndex.x;
                uint2 tileData = _TileData[tileId];
                uint tileLightCount = tileData.x;
                uint tileLightOffset = tileData.y;
                if (tileLightCount == 0) return;

                float lu = RNG_Next(rng);
                uint sampleIndex = min(uint(lu * tileLightCount), tileLightCount - 1);
                lightIdx = _LightCullingData[tileLightOffset + sampleIndex];
                invSelectPdf = (float)tileLightCount;
            }
            else return;
        }
        else
        {
            float lu = RNG_Next(rng);
            lightIdx = min(uint(lu * _PointLightsCount), (uint)_PointLightsCount - 1);
            invSelectPdf = (float)_PointLightsCount;
        }

        PointLightData light = LoadPointLight(lightIdx);
        if (light.intensity <= 0.0 || light.range <= 0.0) return;

        float3 toCenter = light.position - hit.position;
        float distC = length(toCenter);
        float rangeAtten = GetPointLightRangeAttenuation(distC, light.range);
        if (rangeAtten <= 0.0) return;
        float3 L0 = toCenter / max(distC, 1e-6);
        float3 upRef = abs(L0.y) < 0.99 ? float3(0,1,0) : float3(1,0,0);
        float3 right = normalize(cross(upRef, L0));
        float3 up2 = cross(L0, right);
        float radius = max(light.sourceRadius, 0.0);
        float3 samplePos = light.position;
        if (radius > 1e-4)
        {
            float2 d = SampleDisk(RNG_Next(rng), RNG_Next(rng)) * radius;
            samplePos = light.position + d.x * right + d.y * up2;
        }
        float3 toSample = samplePos - hit.position;
        float distS = length(toSample);
        float3 Ls = toSample / max(distS, 1e-6);

        float NdotL = saturate(dot(hit.normal, Ls));
        if (NdotL <= 0.0) return;

        float3 f_brdf; float dummyPdf;
        EvaluateBXDF_GivenDir(hit, V, Ls, /*out*/ f_brdf, /*out*/ dummyPdf);

        float3 Le = light.color * light.intensity;
        float3 illum;
        if (radius <= 1e-4)
        {
            illum = throughput * Le * (f_brdf * NdotL) / max(distS * distS, 1e-6);
        }
        else
        {
            float pdf_area = 1.0 / (PI * radius * radius);
            float3 nLight = normalize(hit.position - light.position);
            float cosThetaPrime = saturate(dot(nLight, -Ls));
            if (cosThetaPrime <= 1e-6) return;
            float geom = cosThetaPrime / max(distS * distS, 1e-6);

            illum = throughput * Le * (f_brdf * NdotL) * (geom / pdf_area);
        }
        illum *= rangeAtten * invSelectPdf;

        uint idx;
        InterlockedAdd(BufferSizes[CurBounce].shadowRays, 1, idx);
        ShadowRayData sr;
        sr.origin = hit.position + hit.normal * 1e-5;
        sr.direction = Ls;
        sr.maxDist = distS;
        sr.illumination = illum;
        sr.pixelIndex = pixelIndex;
        ShadowRaysBuffer[idx] = sr;
    }
}
