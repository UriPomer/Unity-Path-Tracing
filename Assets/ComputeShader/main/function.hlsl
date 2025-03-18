#ifndef FUNCTION
#define FUNCTION

#include "global.hlsl"
#include "bxdf.hlsl"

/*
 * 传入面的索引idx，交点处通过插值得到的uv坐标data，法线纹理索引normIdx，uv坐标uv
 * 返回局部坐标的法线
 */
float3 GetNormal(int idx, float2 data, int normIdx, float2 uv)
{
    float3 norm0 = _Normals[_Indices[idx]];
    float3 norm1 = _Normals[_Indices[idx + 1]];
    float3 norm2 = _Normals[_Indices[idx + 2]];
    float3 norm = norm1 * data.x + norm2 * data.y + norm0 * (1.0 - data.x - data.y);    //插值得到法线
    float4 tangent0 = _Tangents[_Indices[idx]];
    float4 tangent1 = _Tangents[_Indices[idx + 1]];
    float4 tangent2 = _Tangents[_Indices[idx + 2]];
    float4 tangent = tangent1 * data.x + tangent2 * data.y + tangent0 * (1.0 - data.x - data.y);    //插值得到切线
    //tangent.w = tangent0.w;
    if (normIdx >= 0)
    {
        float3 binorm = normalize(cross(norm, tangent.xyz)) * tangent.w;    //计算副法线，也是一个切线，tangent.w通常是1或-1，为切线的方向，保持正交
        float3x3 TBN = float3x3(
            norm,
            binorm,
            tangent.xyz
        );  // 切线空间矩阵
        float3 normTS = _NormalTextures.SampleLevel(sampler_NormalTextures, float3(uv, normIdx), 0.0).xyz * 2.0 - 1.0;
        return mul(normTS, TBN);   //将法线从切线空间变换到物体的局部坐标系
    }
    else
    {
        return norm;
    }
}

float sdot(float3 x, float3 y, float f = 1.0f)
{
    return saturate(dot(x, y) * f);
}

float rand()
{
    float result = frac(sin(_Seed / 100.0f * dot(_Pixel, float2(12.9898f, 78.233f))) * 43758.5453f); // Fraction part
    _Seed += 1.0f;
    return result;
}

bool SkipTransparent(Material mat)
{
    float f = DielectricFresnel(0.2, mat.ior);
    float r = mat.roughness * mat.roughness;
    return rand() < (1.0 - f) * (1.0 - mat.metallic) * (1.0 - r);
}

float SmoothnessToPhongAlpha(float s)
{
    return pow(1000.0f, s * s);
}

float3x3 GetTangentSpace(float3 normal)
{
    // Choose a helper vector for the cross product
    float3 helper = float3(1, 0, 0);
    if (abs(normal.x) > 0.99f)
        helper = float3(0, 0, 1);

    // Generate vectors
    float3 tangent = normalize(cross(normal, helper));
    float3 binormal = normalize(cross(normal, tangent));
    return float3x3(tangent, binormal, normal);
}

float3 SampleReflectionDirectionSphere(float3 normal, float alpha)
{
    // Spherical coordinate to cartesian
    float cosTheta = pow(rand(), 1.0f / (alpha + 1.0f));
    float sinTheta = sqrt(1.0f - cosTheta * cosTheta);
    float phi = 2 * PI * rand();
    float3 tangentSpaceDir = float3(cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta);

    // Transform direction to world space
    return mul(tangentSpaceDir, GetTangentSpace(normal));
}

float3 SampleHemisphere4(float3 norm)
{
    float2 rand1 = rand();
    float theta = rand1.x * PI_TWO;
    float phi = acos(1.0 - 2.0 * rand1.y);
    float3 v = float3(sin(phi) * cos(theta), sin(phi) * sin(theta), cos(phi));
    return v * sign(dot(v, norm));
}

Ray GenRayByID(float2 pixelCoord)
{
    uint width, height;
    _Result.GetDimensions(width, height);

    float2 screenPos = (pixelCoord + 0.5f) / float2(width, height);
    float2 ndcPos = screenPos * 2.0f - 1.0f;  // NDC 坐标

    float4 clipPos = float4(ndcPos, 1.0f, 1.0f);
    float3 viewPos = mul(_CameraInverseProjection, clipPos).xyz;
    float3 worldDir = mul((float3x3)_CameraToWorld, normalize(viewPos));

    Ray ray;
    ray.origin = mul(_CameraToWorld, float4(0.0f, 0.0f, 0.0f, 1.0f)).xyz;
    //TODO: 随机生成光线方向
    
    ray.dir = normalize(worldDir);
    ray.energy = float3(1.0f, 1.0f, 1.0f);
    ray.invDir = 1.0f / ray.dir;

    return ray;
}

#endif
