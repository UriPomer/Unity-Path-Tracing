
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

bool IsLambertianMaterial(Material mat)
{
    return mat.metallic < 0.001 && mat.roughness >= 0.999;
}

void GetOpaqueLobeWeights(Material mat, out float specProb, out float diffProb)
{
    if (IsLambertianMaterial(mat))
    {
        specProb = 0.0;
        diffProb = 1.0;
        return;
    }

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

    // Very rough dielectrics behave much more like diffuse reflectors than glossy mirrors.
    // Without this boost, rough walls still spend too many samples on GGX VNDF specular picks,
    // which then frequently reflect below the hemisphere and collapse into zero-throughput tails.
    if (mat.metallic < 0.999)
    {
        float roughDiffuseBoost = saturate(mat.roughness * mat.roughness);
        diffProb = lerp(diffProb, 1.0, roughDiffuseBoost);
        specProb = 1.0 - diffProb;
    }
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

float3 EvaluateOpaqueDiffuse(Material mat, float NdotV, float NdotL, float LdotH)
{
    return IsLambertianMaterial(mat)
        ? mat.albedo * INV_PI
        : EvaluateDisneyDiffuse(mat, NdotV, NdotL, LdotH);
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
    f_brdf = F * G * D / (4.0 * NdotV * NdotL);

    pdf = NdotH * D / (4.0 * VdotH);
}

float GGXVNDFReflectionPdf(RayHit hit, float3 V, float3 H)
{
    float NdotV = saturate(dot(hit.normal, V));
    float VdotH = dot(V, H);
    if (NdotV <= 0.0 || VdotH <= 0.0)
        return 0.0;
    float alpha = max(hit.material.roughness * hit.material.roughness, 1e-4);
    float D = DistributionGGX(hit.normal, H, alpha);
    return D * SmithG(NdotV, alpha) / (4.0 * NdotV);
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

void EvaluateBXDF_GivenDir(RayHit hit, float3 V, float3 L, out float3 f_brdf, out float pdf);

void EvaluateOpaqueBXDF_GivenDir(RayHit hit, float3 V, float3 L, out float3 f_brdf, out float pdf);

void SampleOpaqueBXDF(RayHit hit, inout Ray ray,
    out float3 throughput, out bool sampledSpecular, out float zeroReasonCode)
{
    float3 V = -ray.dir;
    throughput = 0.0;
    sampledSpecular = false;
    zeroReasonCode = 0.0;
    float roulette = RNG_Next(rng);
    float specProbability, diffuseProbability;
    GetOpaqueLobeWeights(hit.material, specProbability, diffuseProbability);
    sampledSpecular = roulette < specProbability;
    if (sampledSpecular && hit.material.roughness < 1e-4)
    {
        ray.dir = reflect(-V, hit.normal);
        if (dot(hit.normal, ray.dir) > 0.0)
            throughput = SchlickFresnel(saturate(dot(hit.normal, V)), GetMaterialF0(hit.material)) / specProbability;
        return;
    }
    if (sampledSpecular)
    {
        float alpha = max(hit.material.roughness * hit.material.roughness, 1e-4);
        float3 H = SampleGGXVNDF(hit.normal, V, alpha, RNG_Next2(rng));
        ray.dir = normalize(reflect(-V, H));
    }
    else ray.dir = normalize(SampleHemisphere(hit.normal));
    float3 value;
    float pdf;
    // The same marginal BSDF/PDF pair is used by PT and ReSTIR proposals.
    EvaluateOpaqueBXDF_GivenDir(hit, V, ray.dir, value, pdf);
    if (pdf > 0.0) throughput = value * saturate(dot(hit.normal, ray.dir)) / pdf;
    if (all(throughput <= 0.0)) zeroReasonCode = 2.0;
}

void EvaluateBXDFWithDotAndPDFDetailed(RayHit hit, inout Ray ray,
    out float3 throughput, out bool sampledSpecular, out float zeroReasonCode)
{
    float3 V = -ray.dir;
    throughput = 0.0;
    sampledSpecular = false;
    zeroReasonCode = 0.0;
    // Alpha coverage is a BSDF mixture with a straight-through delta event.
    if (hit.mode == 2.0 && RNG_Next(rng) >= saturate(hit.material.alpha))
    {
        throughput = 1.0;
        sampledSpecular = true;
        return;
    }
    if (hit.mode < 3.0)
    {
        SampleOpaqueBXDF(hit, ray, throughput, sampledSpecular, zeroReasonCode);
        return;
    }
    float roulette = RNG_Next(rng);
    if (hit.mode >= 3.0)
    {
        bool exiting = dot(V, hit.normal) < 0.0;
        float3 N = exiting ? -hit.normal : hit.normal;
        float eta = exiting ? rcp(hit.material.ior) : hit.material.ior;
        float3 H = N;
        bool smooth = hit.material.roughness < 1e-4;
        if (!smooth)
            H = SampleGGXVNDF(N, V, max(hit.material.roughness * hit.material.roughness, 1e-4),
                RNG_Next2(rng));
        float F = DielectricFresnel(dot(V, H), eta);
        float reflectProbability = lerp(F, 1.0, hit.material.metallic);
        sampledSpecular = true;
        if (roulette < reflectProbability)
        {
            ray.dir = normalize(reflect(-V, H));
            if (dot(N, ray.dir) <= 0.0) return;
            if (smooth)
            {
                throughput = lerp(F.xxx, hit.material.albedo, hit.material.metallic) / reflectProbability;
                return;
            }
        }
        else
        {
            float3 direction = refract(-V, H, rcp(eta));
            if (dot(direction, direction) == 0.0 || dot(N, direction) >= 0.0) return;
            ray.dir = normalize(direction);
            if (smooth || eta == 1.0)
            {
                throughput = sqrt(max(hit.material.albedo, 0.0)) / (eta * eta);
                return;
            }
        }
        float3 value;
        float pdf;
        EvaluateBXDF_GivenDir(hit, V, ray.dir, value, pdf);
        if (pdf > 0.0) throughput = value * abs(dot(N, ray.dir)) / pdf;
        return;
    }

}

void EvaluateBXDFWithDotAndPDF(RayHit hit, inout Ray ray, out float3 f_brdf)
{
    bool sampledSpecular;
    float zeroReasonCode;
    EvaluateBXDFWithDotAndPDFDetailed(hit, ray, f_brdf, sampledSpecular, zeroReasonCode);
}

void EvaluateBXDF_GivenDir(RayHit hit, float3 V, float3 L, out float3 f_brdf, out float pdf)
{
    f_brdf = 0.0;
    pdf = 0.0;

    float3 N = hit.normal;
    float NdotL;

    if (hit.mode >= 3.0)
    {
        if (hit.material.roughness < 1e-4) return; // Delta distributions have no continuous density.
        bool exiting = dot(V, N) < 0.0;
        if (exiting) N = -N;
        float eta = exiting ? rcp(hit.material.ior) : hit.material.ior;
        float NV = dot(N, V), NL = dot(N, L);
        if (NV <= 0.0 || NL == 0.0) return;
        bool reflection = NL > 0.0;
        float3 halfVector = reflection ? V + L : V + eta * L;
        if (dot(halfVector, halfVector) == 0.0) return;
        float3 H = normalize(halfVector);
        if (dot(N, H) < 0.0) H = -H;
        float VH = dot(V, H), LH = dot(L, H);
        if (VH <= 0.0 || LH * NL <= 0.0) return;
        float alpha = max(hit.material.roughness * hit.material.roughness, 1e-4);
        float D = DistributionGGX(N, H, alpha);
        float G1 = SmithG(NV, alpha);
        float G = G1 * SmithG(abs(NL), alpha);
        float F = DielectricFresnel(VH, eta);
        float probability = lerp(F, 1.0, hit.material.metallic);
        if (reflection)
        {
            f_brdf = lerp(F.xxx, hit.material.albedo, hit.material.metallic) * D * G / (4.0 * NV * NL);
            pdf = probability * D * G1 / (4.0 * NV);
        }
        else
        {
            float denominator = VH + eta * LH;
            denominator *= denominator;
            if (denominator <= 0.0) return;
            float T = (1.0 - probability);
            f_brdf = sqrt(max(hit.material.albedo, 0.0)) * T * D * G * abs(VH * LH) / (NV * abs(NL) * denominator);
            pdf = T * D * G1 * VH / NV * eta * eta * abs(LH) / denominator;
        }
        return;
    }

    EvaluateOpaqueBXDF_GivenDir(hit, V, L, f_brdf, pdf);
    if (hit.mode == 2.0)
    {
        f_brdf *= saturate(hit.material.alpha);
        pdf *= saturate(hit.material.alpha);
    }
}

void EvaluateOpaqueBXDF_GivenDir(RayHit hit, float3 V, float3 L, out float3 f_brdf, out float pdf)
{
    f_brdf = 0.0;
    pdf = 0.0;
    float3 N = hit.normal;
    float NdotL;
    NdotL = saturate(dot(N, L));
    float NdotV = saturate(dot(N, V));
    if (NdotL <= 0.0 || NdotV <= 0.0)
        return;

    float specProb, diffProb;
    GetOpaqueLobeWeights(hit.material, specProb, diffProb);

    float3 H = normalize(V + L);
    float LdotH = saturate(dot(L, H));
    float3 f_diff = EvaluateOpaqueDiffuse(hit.material, NdotV, NdotL, LdotH);
    float pdf_d = NdotL / PI;
    float3 f_spec = 0.0;
    float pdf_s = 0.0;

    if (hit.material.roughness >= 1e-4 && !IsLambertianMaterial(hit.material))
    {
        float3 brdfSpec;
        float microPdf;
        SpecReflModel(hit, V, L, H, brdfSpec, microPdf);
        f_spec = brdfSpec;
        pdf_s = GGXVNDFReflectionPdf(hit, V, H);
    }

    f_brdf = f_diff + f_spec;
    pdf = diffProb * max(pdf_d, 0.0) + specProb * max(pdf_s, 0.0);
}

#endif
