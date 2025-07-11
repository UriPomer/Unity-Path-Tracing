#pragma once

#include "global.hlsl"


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

    float3 sunDir = normalize(_InverseDirectionalLight);
    float cosA = saturate(dot(dir, sunDir));
    float disk = pow(cosA, 50);
    float3 sunColor = disk
                    * _DirectionalLightColor.rgb
                    * _DirectionalLightColor.a;

    return envColor + sunColor;
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

float3 EvaluateBRDF(RayHit hit, float3 V, float3 L, out float3 f_spec, out float3 f_diffuse)
{
    float NdotL = saturate(dot(hit.normal, L));
    float3 H    = normalize(V + L);

    float3 F0;
    F0 = lerp(float3(0.04,0.04,0.04), hit.material.albedo, hit.material.metallic);
    float3 F = SchlickFresnel( saturate(dot(H,V)), F0 );
    float  D = DistributionGGX(hit.normal, H, hit.material.roughness);
    float  G = SmithG( saturate(dot(hit.normal, V)), hit.material.roughness )
             * SmithG( saturate(dot(hit.normal, L)), hit.material.roughness );
    f_spec = (D * G * F) / max(4.0 * dot(hit.normal, V) * dot(hit.normal, L), 1e-4);

    float3 kd      = (1.0 - hit.material.metallic) * hit.material.albedo;
    f_diffuse = kd / PI;

    return (f_diffuse + f_spec) * NdotL;
}

float3 GetLightContribution(RayHit hit, float3 V)
{
    float3 lightContribution = 0.0;

    // —— 环境（方向光）——
    if (_DirectionalLightColor.a > 0.0)
    {
        // L 是表面到光源的方向
        float3 L = normalize(_InverseDirectionalLight);
        // 阴影测试
        Ray shadowRay = GenRay(hit.position + hit.normal * 1e-5, L);
        if (!TraceHit(shadowRay, 1.#INF))
        {
            // 用完整 BRDF 评估
            float3 f_spec, f_diff;
            float3 brdfCos = EvaluateBRDF(hit, V, L, f_spec, f_diff);
            lightContribution += brdfCos * _DirectionalLightColor.rgb * _DirectionalLightColor.a;
        }
    }
    
    // —— 点光源合集 —— 
    for (int i = 0; i < _PointLightsCount; ++i)
    {
        float4 lightPos   = _PointLights[i * 2];
        float4 lightColor = _PointLights[i * 2 + 1];
        if (lightColor.a <= 0.0) continue;

        float3 toLight = lightPos.xyz - hit.position;
        float  dist    = length(toLight);
        float3 L       = toLight / dist;
        // 阴影测试（只检到 lightPos 距离）
        Ray shadowRay = GenRay(hit.position + hit.normal * 1e-5, L);
        if (!TraceHit(shadowRay, dist))
        {
            // 距离衰减
            float  d      = max(0.0, dist - lightPos.w);
            float  decay  = pow(0.2, d * d);

            // 完整 BRDF 评估
            float3 f_spec, f_diff;
            float3 brdfCos = EvaluateBRDF(hit, V, L, f_spec, f_diff);

            lightContribution += brdfCos * lightColor.rgb * lightColor.a * decay;
        }
    }

    return lightContribution;
}