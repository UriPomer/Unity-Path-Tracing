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

void SpecReflModel(RayHit hit, float3 V, float3 L, float3 H, inout float3 energy)
{
    float NdotL = abs(dot(hit.normal, L));
    //float NdotV = abs(dot(hit.norm, -V));
    float3 specColor = lerp(0.04, hit.material.albedo, hit.material.metallic);
    float3 F = SchlickFresnel(dot(L, H), specColor);
    //float D = DistributionGGX(hit.norm, H, hit.mat.roughness);
    //float G = GeometrySmith(hit.norm, -V, L, hit.mat.roughness);
    float G = SmithG(NdotL, hit.material.roughness);
    energy *= F * G;
}

void SpecRefrModel(RayHit hit, float3 V, float3 L, float3 H, inout float3 energy)
{
    float NdotL = abs(dot(hit.normal, L));
    //float NdotV = abs(dot(-hit.norm, -V));
    float F = DielectricFresnel(dot(V, H), hit.material.ior);
    //float D = DistributionGGX(hit.norm, H, hit.mat.roughness);
    float G = SmithG(NdotL, hit.material.roughness);
    //float eta2 = hit.mat.ior * hit.mat.ior;
    energy *= pow(hit.material.albedo, 0.5) * (1.0 - hit.material.metallic) *
        (1.0 - F) * G;
}


#endif