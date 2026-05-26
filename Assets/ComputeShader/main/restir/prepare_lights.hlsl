#pragma once

// Depends on: global.hlsl, trace.hlsl (PointLightData, LoadPointLight etc.)

RWStructuredBuffer<LightDataPacked> _LightDataPacked;
uint _LightDataPackedCount;

[numthreads(64, 1, 1)]
void kernel_prepare_lights(uint3 id : SV_DispatchThreadID)
{
    if (id.x >= _LightDataPackedCount) return;
    LightDataPacked p = (LightDataPacked)0;

    if (id.x == 0u)
    {
        // Sun light always at slot 0
        p.direction = normalize(_InverseDirectionalLight);
        p.color = _DirectionalLightColor.rgb;
        p.intensity = _DirectionalLightColor.a;
        p.lightType = 1u;
        p.power = dot(p.color, float3(0.2126, 0.7152, 0.0722)) * p.intensity;
        p.range = 1e20;
        p.sourceRadius = 0.0;
        p.cdf = p.power;
    }
    else
    {
        uint pi = id.x - 1u;
        if (pi >= (uint)_PointLightsCount)
        {
            p.power = 0.0;
        }
        else
        {
            float4 pr = _PointLights[pi * 3];
            float4 ci = _PointLights[pi * 3 + 1];
            float4 m  = _PointLights[pi * 3 + 2];
            p.position = pr.xyz; p.range = pr.w;
            p.color = ci.rgb; p.intensity = ci.a;
            p.sourceRadius = m.x;
            p.direction = float3(0, 0, 0);
            p.lightType = 2u; p.originalIndex = pi;
            p.power = dot(ci.rgb, float3(0.2126, 0.7152, 0.0722)) * ci.a;
        }
        p.cdf = p.power;
    }
    _LightDataPacked[id.x] = p;
}
