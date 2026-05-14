
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

float3 SchlickFresnel(float cosTheta, float3 F0)
{
    return lerp(F0, 1.0, pow(1.0 - cosTheta, 5.0));
}

// Schlick weight:
//   w(u) = (1 - u)^5
// Common in Disney diffuse / Schlick Fresnel style terms.
// It makes the response stronger near grazing angles and weaker near normal incidence.
float SchlickWeight(float u)
{
    float m = saturate(1.0 - u);
    float m2 = m * m;
    return m2 * m2 * m;
}

// Dielectric normal-incidence reflectance:
//   F0 = ((eta - 1) / (eta + 1))^2
// where eta is the index of refraction (IOR).
float DielectricF0(float ior)
{
    float a = (ior - 1.0) / (ior + 1.0);
    return a * a;
}

float GetDielectricF0(Material mat)
{
    // For non-metals we want a reasonable dielectric F0.
    // Physically, we would use:
    //   F0 = ((ior - 1) / (ior + 1))^2
    //
    // But this project historically stores many opaque materials with IOR = 1.0.
    // That gives:
    //   F0 = 0
    // which removes almost all specular highlight from dielectrics and makes the image look gray/flat.
    //
    // So this function behaves like:
    //   F0_dielectric = max(0.04, ((ior - 1) / (ior + 1))^2)
    // and for near-default IOR values we explicitly fall back to 0.04.
    //
    // 0.04 is the common PBR default for many dielectrics, roughly corresponding to IOR ~ 1.5.
    float ior = max(mat.ior, 1.0);
    float dielectricF0 = DielectricF0(ior);
    if (ior <= 1.01)
        dielectricF0 = 0.04;
    return max(dielectricF0, 0.04);
}

// Blend between dielectric F0 and metallic colored F0:
//   F0_material = lerp(F0_dielectric, baseColor, metallic)
//
// metallic = 0:
//   F0 = dielectric scalar, diffuse is allowed
// metallic = 1:
//   F0 = albedo/baseColor, diffuse should vanish
float3 GetMaterialF0(Material mat)
{
    float dielectricF0 = GetDielectricF0(mat);
    return lerp(float3(dielectricF0, dielectricF0, dielectricF0), mat.albedo, mat.metallic);
}

void GetOpaqueLobeWeights(Material mat, out float specProb, out float diffProb)
{
    // These are sampling probabilities, not exact energy-conservation equations.
    // We use them for BSDF lobe selection in Russian roulette:
    //
    //   w_spec = luminance(F0)
    //   w_diff = (1 - metallic) * luminance(baseColor)
    //
    //   p_spec = w_spec / (w_spec + w_diff)
    //   p_diff = w_diff / (w_spec + w_diff)
    //
    // Intuition:
    // - brighter/specular materials should sample the specular lobe more often
    // - non-metal colored materials should still spend samples on diffuse
    // - metals push probability toward specular because their diffuse term should disappear
    //
    // This is a practical importance-sampling heuristic to reduce variance.
    float3 F0 = GetMaterialF0(mat);
    float specWeight = saturate(dot(F0, LUM));
    float diffWeight = saturate((1.0 - mat.metallic) * dot(mat.albedo, LUM));
    float sum = specWeight + diffWeight;

    if (sum <= 1e-4)
    {
        specProb = 0.5;
        diffProb = 0.5;
        return;
    }

    specProb = specWeight / sum;
    diffProb = diffWeight / sum;
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

float3 EvaluateDisneyDiffuse(Material mat, float NdotV, float NdotL, float LdotH)
{
    float rough = mat.roughness * mat.roughness;
    float fd90 = 0.5 + 2.0 * rough * LdotH * LdotH;
    float lightScatter = lerp(1.0, fd90, SchlickWeight(NdotL));
    float viewScatter = lerp(1.0, fd90, SchlickWeight(NdotV));
    return (1.0 - mat.metallic) * mat.albedo * (INV_PI * lightScatter * viewScatter);
}

void SpecReflModel(
    RayHit hit, float3 V, float3 L, float3 H,
    out float3 f_brdf,
    out float pdf)
{
    float NdotL = saturate(dot(hit.normal, L));
    float NdotV = saturate(dot(hit.normal, V));
    float NdotH = saturate(dot(hit.normal, H));
    float VdotH = saturate(dot(V, H));

    float3 F0 = GetMaterialF0(hit.material);
    float alpha = max(hit.material.roughness * hit.material.roughness, 1e-4);
    float3 F = SchlickFresnel(VdotH, F0);
    float D = DistributionGGX(hit.normal, H, alpha);
    float G = GeometrySmith(hit.normal, V, L, alpha);
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
    energy *= pow(hit.material.albedo, 0.5) * (1.0 - hit.material.metallic) * (1.0 - F) * G;
}

float3 SampleGGXVNDF(float3 N, float3 V, float alpha, float2 Xi)
{
    float3 up = abs(N.z) < 0.999 ? float3(0, 0, 1) : float3(1, 0, 0);
    float3 tangentX = normalize(cross(up, N));
    float3 tangentY = cross(N, tangentX);

    float3 Vh = normalize(float3(alpha * dot(V, tangentX),
                                 alpha * dot(V, tangentY),
                                 dot(V, N)));

    float lensq = Vh.x * Vh.x + Vh.y * Vh.y;
    float3 T1 = lensq > 0.0
              ? float3(-Vh.y, Vh.x, 0) / sqrt(lensq)
              : float3(1, 0, 0);
    float3 T2 = cross(Vh, T1);

    float r = sqrt(Xi.x);
    float phi = 2.0 * PI * Xi.y;
    float t1 = r * cos(phi);
    float t2 = r * sin(phi);

    float s = 0.5 * (1.0 + Vh.z);
    t2 = lerp(sqrt(max(0, 1.0 - t1 * t1)), t2, s);

    float3 Nh = t1 * T1 + t2 * T2 + sqrt(max(0, 1.0 - t1 * t1 - t2 * t2)) * Vh;

    float3 H = normalize(float3(alpha * Nh.x,
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

        float alpha = SmoothnessToPhongAlpha(hit.material.roughness);
        hit.normal = normalize(lerp(
            hit.normal,
            SampleReflectionDirectionSphere(hit.normal, alpha),
            hit.material.roughness * hit.material.roughness
        ));
        rayOutDir = normalize(reflect(ray.dir, hit.normal));
        float3 H = normalize(V + rayOutDir);
        float fresnel = DielectricFresnel(dot(H, V), hit.material.ior);
        float reflChance = 1.0 - (1.0 - fresnel) * (1.0 - hit.material.metallic);
        if (roulette < reflChance)
        {
            float3 brdfCos;
            float microPdf;
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
            SpecRefrModel(hit, V, rayOutDir, H, refrWeight);
            pdf = clamp(1.0 - reflChance, 1e-3, 1.0);
            f_brdf = refrWeight / pdf;
        }
    }
    else
    {
        float specProb, diffProb;
        GetOpaqueLobeWeights(hit.material, specProb, diffProb);

        if (roulette < specProb)
        {
            if (hit.material.roughness < 1e-4)
            {
                rayOutDir = reflect(-V, hit.normal);
                float NdotL = saturate(dot(hit.normal, rayOutDir));
                if (NdotL <= 0.0)
                {
                    ray.dir = rayOutDir;
                    f_brdf = 0.0;
                    return;
                }

                float3 F0 = GetMaterialF0(hit.material);
                float3 F = SchlickFresnel(saturate(dot(hit.normal, V)), F0);
                pdf = max(specProb, 1e-3);
                f_brdf = F / pdf;
            }
            else
            {
                float alpha = max(hit.material.roughness * hit.material.roughness, 1e-4);
                float2 xi = float2(RNG_Next(rng), RNG_Next(rng));
                float3 H = SampleGGXVNDF(hit.normal, V, alpha, xi);
                rayOutDir = normalize(reflect(-V, H));
                float NdotL = saturate(dot(hit.normal, rayOutDir));
                if (NdotL <= 0.0)
                {
                    ray.dir = rayOutDir;
                    f_brdf = 0.0;
                    return;
                }

                float3 f_spec;
                float microPdf;
                SpecReflModel(hit, V, rayOutDir, H, f_spec, microPdf);

                pdf = max(specProb * microPdf, 1e-4);
                f_brdf = f_spec * NdotL / pdf;
            }
        }
        else
        {
            rayOutDir = normalize(SampleHemisphere(hit.normal));

            float NdotV = saturate(dot(hit.normal, V));
            float NdotL = saturate(dot(hit.normal, rayOutDir));
            float diffusePdf = NdotL / PI;
            float3 H = normalize(V + rayOutDir);
            float LdotH = saturate(dot(rayOutDir, H));
            float3 f_diffuse = EvaluateDisneyDiffuse(hit.material, NdotV, NdotL, LdotH);

            pdf = max(diffusePdf * diffProb, 1e-4);
            f_brdf = f_diffuse * NdotL / pdf;
        }
    }

    ray.dir = rayOutDir;
}

void EvaluateBXDF_GivenDir(RayHit hit, float3 V, float3 L, out float3 f_brdf, out float pdf)
{
    f_brdf = 0.0;
    pdf = 0.0;

    float3 N = hit.normal;
    float NdotL;

    if (hit.mode >= 3.0)
    {
        if (dot(V, N) < 0.0)
            N = -N;

        NdotL = saturate(dot(N, L));
        if (NdotL <= 0.0)
            return;

        hit.normal = N;
        float3 H = normalize(V + L);
        float3 brdfSpec;
        float pdfSpec;
        SpecReflModel(hit, V, L, H, brdfSpec, pdfSpec);

        float fresnel = DielectricFresnel(dot(H, V), hit.material.ior);
        float reflChance = 1.0 - (1.0 - fresnel) * (1.0 - hit.material.metallic);

        f_brdf = brdfSpec;
        pdf = reflChance * max(pdfSpec, 0.0);
        return;
    }

    NdotL = saturate(dot(N, L));
    float NdotV = saturate(dot(N, V));
    if (NdotL <= 0.0 || NdotV <= 0.0)
        return;

    float specProb, diffProb;
    GetOpaqueLobeWeights(hit.material, specProb, diffProb);

    float3 H = normalize(V + L);
    float LdotH = saturate(dot(L, H));
    float3 f_diff = EvaluateDisneyDiffuse(hit.material, NdotV, NdotL, LdotH);
    float pdf_d = NdotL / PI;
    float3 f_spec = 0.0;
    float pdf_s = 0.0;

    if (hit.material.roughness >= 1e-4)
    {
        float3 brdfSpec;
        float microPdf;
        SpecReflModel(hit, V, L, H, brdfSpec, microPdf);
        f_spec = brdfSpec;
        pdf_s = microPdf;
    }

    f_brdf = f_diff + f_spec;
    pdf = diffProb * max(pdf_d, 0.0) + specProb * max(pdf_s, 0.0);
}

#endif
