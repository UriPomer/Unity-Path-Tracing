
#ifndef BXDF
#define BXDF

// refer to: https://github.com/HummaWhite/ZillumGL/blob/main/src/shader/material.shader
float DielectricFresnel(float cosTi, float ior)
{
    cosTi = clamp(cosTi, -1.0, 1.0);
    if (cosTi < 0.0)
    {
        ior = 1.0 / ior;
        cosTi = -cosTi;
    }

    float sinTi = sqrt(1.0 - cosTi * cosTi);
    float sinTt = sinTi / ior;
    if (sinTt >= 1.0)
        return 1.0;

    float cosTt = sqrt(1.0 - sinTt * sinTt);

    float rPa = (cosTi - ior * cosTt) / (cosTi + ior * cosTt);
    float rPe = (ior * cosTi - cosTt) / (ior * cosTi + cosTt);
    return (rPa * rPa + rPe * rPe) * 0.5;
}

// Schlick Fresnel 近似
float3 SchlickFresnel(float cosTheta, float3 F0)
{
    //return F0 + (1.0 - F0) * pow(1.0 - cosTheta, 5.0);
    return lerp(F0, 1.0, pow(1.0 - cosTheta, 5.0));
}

// Smith GGX shadowing-masking function
float SmithG(float NDotV, float alpha)
{
    float alpha2 = alpha * alpha;
    float b = NDotV * NDotV;
    return (2.0 * NDotV) / (NDotV + sqrt(alpha2 + b - alpha2 * b));
}

float GeometrySmith(float3 N, float3 V, float3 L, float alpha)
{
    float NdotV = saturate(dot(N, V));
    float NdotL = saturate(dot(N, L));
    return SmithG(NdotV, alpha) * SmithG(NdotL, alpha);
}

float DistributionGGX(float3 normal, float3 halfVec, float alpha)
{
    float NdotH = saturate(dot(normal, halfVec));
    
    float alpha2 = alpha * alpha;
    float NdotH2 = NdotH * NdotH;
    float denom = NdotH2 * (alpha2 - 1.0) + 1.0;
    return alpha2 / (PI * denom * denom);
}


void SpecReflModel(
    RayHit hit, float3 V, float3 L, float3 H,
    out half3 f_brdf,
    out half pdf)
{
    half NdotL = saturate(dot(hit.normal, L));
    half NdotV = saturate(dot(hit.normal, V));
    half NdotH = saturate(dot(hit.normal, H));
    half VdotH = saturate(dot(V, H));

    float3 F0 = lerp(float3(0.04,0.04,0.04), hit.material.albedo, hit.material.metallic);

    float alpha = hit.material.roughness * hit.material.roughness;
    float3 F = SchlickFresnel(VdotH, F0);
    float  D = DistributionGGX(hit.normal, H, alpha);
    float  G = GeometrySmith(hit.normal, V, L, alpha);
    f_brdf = F * G * D / max(4.0 * NdotV * NdotL, 1e-4);
    
    pdf = NdotH * D / max(4.0 * VdotH, 1e-4);
}

bool SkipTransparent(Material mat)
{
    float f = DielectricFresnel(0.2, mat.ior);
    float r = mat.roughness * mat.roughness;
    return RNG_Next(rng) < (1.0 - f) * (1.0 - mat.metallic) * (1.0 - r);
}

void SpecRefrModel(RayHit hit, float3 V, float3 L, float3 H, inout float3 energy)
{
    float NdotL = abs(dot(hit.normal, L));
    float F = DielectricFresnel(dot(V, H), hit.material.ior);
    float alpha = hit.material.roughness * hit.material.roughness;
    float G = SmithG(NdotL, alpha);
    energy *= pow(hit.material.albedo, 0.5) * (1.0 - hit.material.metallic) *
        (1.0 - F) * G;
}

float3 SampleGGXVNDF(float3 N, float3 V, float alpha, float2 Xi)
{
    float3 up        = abs(N.z) < 0.999 ? float3(0,0,1) : float3(1,0,0);
    float3 tangentX  = normalize(cross(up, N));
    float3 tangentY  = cross(N, tangentX);

    float3 Vh = normalize(float3(alpha * dot(V, tangentX),
                                 alpha * dot(V, tangentY),
                                 dot(V, N)));
    
    float lensq = Vh.x*Vh.x + Vh.y*Vh.y;
    float3 T1   = lensq > 0.0
                  ? float3(-Vh.y, Vh.x, 0) / sqrt(lensq)
                  : float3(1, 0, 0);
    float3 T2   = cross(Vh, T1);

    float r   = sqrt(Xi.x);
    float phi = 2.0 * PI * Xi.y;
    float t1  = r * cos(phi);
    float t2  = r * sin(phi);

    float s = 0.5 * (1.0 + Vh.z);
    t2      = lerp(sqrt(max(0, 1.0 - t1*t1)), t2, s);

    float3 Nh = t1 * T1 + t2 * T2 + sqrt(max(0, 1.0 - t1*t1 - t2*t2)) * Vh;

    float3 H  = normalize(float3(alpha * Nh.x,
                                 alpha * Nh.y,
                                 max(0, Nh.z)));
    return normalize(H.x * tangentX + H.y * tangentY + H.z * N);
}

void EvaluateBXDFWithDotAndPDF(RayHit hit, inout Ray ray, out float3 f_brdf)
{
    float3 V = -ray.dir;
    float roulette = RNG_Next(rng);
    f_brdf = 0;
    float pdf;
    float3 rayOutDir;
    if (hit.mode >= 3.0)
    {
        if (dot(ray.dir, hit.normal) > 0.0)
            hit.normal = -hit.normal;
        // dielectric workflow
        float alpha = SmoothnessToPhongAlpha(hit.material.roughness);
        hit.normal = normalize(lerp(
            hit.normal,
            SampleReflectionDirectionSphere(hit.normal,alpha),
            hit.material.roughness * hit.material.roughness
        ));
        rayOutDir = normalize(reflect(ray.dir, hit.normal));
        float3 H = normalize(V + rayOutDir);
        float fresnel = DielectricFresnel(dot(H, V), hit.material.ior);
        float reflChance = 1.0 - (1.0 - fresnel) * (1.0 - hit.material.metallic);
        if (roulette < reflChance)
        {
            float3 brdfCos;
            float  microPdf;
            SpecReflModel(hit, V, rayOutDir, H, brdfCos, microPdf);
            pdf = reflChance * microPdf;
            float NdotL = saturate(dot(hit.normal, rayOutDir));
            pdf = max(pdf, 1e-5);
            f_brdf = brdfCos * NdotL / pdf;
        }
        else
        {
            rayOutDir = normalize(refract(ray.dir, hit.normal, 1.0 / hit.material.ior));
            float3 refrWeight = 1.0;
            SpecRefrModel(hit, V, rayOutDir, H, /*inout*/ refrWeight);
            pdf = clamp(1.0 - reflChance, 1e-3, 1.0);
            f_brdf = refrWeight / pdf;
        }
    }
    else
    {
        float3 F0 = lerp(float3(0.04,0.04,0.04), hit.material.albedo, hit.material.metallic);
        float  specProb = saturate(max(max(F0.x, F0.y), F0.z));
        float  diffProb = 1.0 - specProb;
        // Calculate chances of diffuse and specular reflection
        if (roulette < specProb)
        {
            if (hit.material.roughness < 1e-4)
            {
                rayOutDir = reflect(-V, hit.normal);
                pdf = max(specProb, 1e-3);
                f_brdf = F0 / pdf;
            }
            else
            {
                float alpha = hit.material.roughness * hit.material.roughness;
                float2 xi = float2(RNG_Next(rng), RNG_Next(rng));
                float3 H = SampleGGXVNDF(hit.normal, V, alpha, xi);
                rayOutDir = normalize(reflect(-V, H));

                float3 f_spec; float microPdf;
                SpecReflModel(hit, V, rayOutDir, H, f_spec, microPdf);

                pdf = max(specProb * microPdf, 1e-4);
                float NdotL = saturate(dot(hit.normal, rayOutDir));
                f_brdf = f_spec * NdotL / pdf;
            }
        }
        else
        {
            rayOutDir = normalize(SampleHemisphere(hit.normal));

            float NdotL = saturate(dot(hit.normal, rayOutDir));
            float diffusePdf = NdotL / PI;

            float3 f_diffuse = hit.material.albedo / PI;
            pdf = max(diffusePdf * diffProb, 1e-4);
            f_brdf = f_diffuse * NdotL / pdf;
        }
    }
    ray.dir = rayOutDir;
}

void EvaluateBXDF_GivenDir(RayHit hit, float3 V, float3 L, out float3 f_brdf, out float pdf)
{
    f_brdf = 0.0;
    pdf    = 0.0;

    float3 N = hit.normal;
    float  NdotL;

    if (hit.mode >= 3.0)
    {
        if (dot(V, N) < 0.0) N = -N;

        NdotL = saturate(dot(N, L));
        if (NdotL <= 0.0) return;
        float3 H = normalize(V + L);

        float3 brdfSpec; float pdfSpec;
        SpecReflModel(hit, V, L, H, /*out*/ brdfSpec, /*out*/ pdfSpec);

        float fresnel    = DielectricFresnel(dot(H, V), hit.material.ior);
        float reflChance = 1.0 - (1.0 - fresnel) * (1.0 - hit.material.metallic);

        f_brdf = brdfSpec;
        pdf    = reflChance * max(pdfSpec, 0.0);
        return;
    }
    {
        NdotL = saturate(dot(N, L));
        if (NdotL <= 0.0) return;

        float3 F0 = lerp(float3(0.04, 0.04, 0.04), hit.material.albedo, hit.material.metallic);
        float  specProb = saturate(max(max(F0.x, F0.y), F0.z));
        float  diffProb = 1.0 - specProb;

        float3 f_diff = hit.material.albedo / PI;
        float  pdf_d  = NdotL / PI;
        float3 f_spec;
        float  pdf_s;

        if (hit.material.roughness < 1e-4)
        {
            f_spec = 0.0;
            pdf_s  = 0.0;
        }
        else
        {
            float3 H = normalize(V + L);

            float3 brdfSpec; float microPdf;
            SpecReflModel(hit, V, L, H, /*out*/ brdfSpec, /*out*/ microPdf);

            f_spec = brdfSpec;
            pdf_s  = microPdf;
        }

        f_brdf = f_diff + f_spec;
        pdf    = diffProb * max(pdf_d, 0.0) + specProb * max(pdf_s, 0.0);
    }
}


#endif