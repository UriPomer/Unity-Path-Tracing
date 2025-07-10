#ifndef BXDF
#define BXDF

// refer to: https://github.com/HummaWhite/ZillumGL/blob/main/src/shader/material.shader
float DielectricFresnel(float cosTi, float eta)
{
    cosTi = clamp(cosTi, -1.0, 1.0);
    if (cosTi < 0.0)
    {
        eta = 1.0 / eta;
        cosTi = -cosTi;
    }

    float sinTi = sqrt(1.0 - cosTi * cosTi);
    float sinTt = sinTi / eta;
    if (sinTt >= 1.0)
        return 1.0;

    float cosTt = sqrt(1.0 - sinTt * sinTt);

    float rPa = (cosTi - eta * cosTt) / (cosTi + eta * cosTt);
    float rPe = (eta * cosTi - cosTt) / (eta * cosTi + cosTt);
    return (rPa * rPa + rPe * rPe) * 0.5;
}

// Schlick Fresnel 近似
float3 SchlickFresnel(float cosTheta, float3 F0)
{
    //return F0 + (1.0 - F0) * pow(abs(1.0 - cosTheta), 5.0);
    return lerp(F0, 1.0, pow(abs(1.0 - cosTheta), 5.0));
}

// Smith GGX shadowing-masking function
float SmithG(float NDotV, float alphaG)
{
    float a = alphaG * alphaG;
    float b = NDotV * NDotV;
    return (2.0 * NDotV) / (NDotV + sqrt(a + b - a * b));
}

float DistributionGGX(float3 normal, float3 halfVec, float roughness)
{
    float NdotH = saturate(dot(normal, halfVec));
        float alpha = roughness * roughness;
    
    float alpha2 = alpha * alpha;
    float NdotH2 = NdotH * NdotH;
    float denom = NdotH2 * (alpha2 - 1.0) + 1.0;
    return alpha2 / (PI * denom * denom);
}


void SpecReflModel(
    RayHit hit, float3 V, float3 L, float3 H,
    out float3 f_brdf_cos,  // 输出 BRDF·cos
    out float   pdf         // 输出本事件 pdf
)
{
    float NdotL = saturate(dot(hit.normal, L));
    float NdotV = saturate(dot(hit.normal, V));
    float NdotH = saturate(dot(hit.normal, H));
    float VdotH = saturate(dot(V, H));

    float3 F0 = lerp(float3(0.04,0.04,0.04), hit.material.albedo, hit.material.metallic);

    float alpha = max(hit.material.roughness * hit.material.roughness, 1e-4);

    float3 F = SchlickFresnel(VdotH, F0);
    float  D = DistributionGGX(hit.normal, H, sqrt(alpha)); // 你的 DistributionGGX 里直接用 roughness，传入 sqrt(α)
    float  G = SmithG(NdotV, sqrt(alpha)) * SmithG(NdotL, sqrt(alpha));

    f_brdf_cos = F * G * D * NdotL / max(4.0 * NdotV, 1e-4);
    pdf = NdotH * D / max(4.0 * VdotH, 1e-4);
}


void SpecRefrModel(RayHit hit, float3 V, float3 L, float3 H, inout float3 energy)
{
    float NdotL = abs(dot(hit.normal, L));
    float F = DielectricFresnel(dot(V, H), hit.material.ior);
    float G = SmithG(NdotL, hit.material.roughness);
    energy *= pow(hit.material.albedo, 0.5) * (1.0 - hit.material.metallic) *
        (1.0 - F) * G;
}


#endif