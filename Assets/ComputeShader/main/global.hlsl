
#ifndef GLOBAL
#define GLOBAL
#include <HLSLSupport.cginc>

struct Ray
{
    float3 origin;
    float3 dir;
    float3 invDir;
};

struct Material
{
    half3 albedo;
    half3 emission;
    half emissionIntensity;
    half roughness;
    half metallic;
    half alpha;
    float ior;
};

// 光源剔除相关常量
#define TILE_SIZE 16
#define MAX_LIGHTS_PER_TILE 32

struct RayHit
{
    float distance;
    float3 position;
    float3 normal;
    Material material;
    float mode;
    bool should_break;
};

RWTexture2D<float4> _Result;
float2 _Resolution;
int _TraceDepth;
float4x4 _CameraToWorld;
float4x4 _CameraInverseProjection;
Texture2D<float4> _SkyboxTexture;
SamplerState sampler_SkyboxTexture;
float3 _InverseDirectionalLight;
float4 _DirectionalLightColor;
float _SunFocus;
float _SunAngularRadius;
float _SkyboxIntensity;
StructuredBuffer<float4> _PointLights;
int _PointLightsCount;
uint _FrameCount;

StructuredBuffer<uint> _LightCullingData;  // 每个tile的光源索引列表
StructuredBuffer<uint2> _TileData;         // 每个tile的光源数量和起始偏移
uint2 _TileCount;                          // 屏幕分块数量 (x, y)
float _CameraNear;                         // 相机近平面
float _CameraFar;                          // 相机远平面

struct RNG
{
    uint state;
};
// Keep RNG as shader-internal state so Unity does not reflect it as a kernel parameter.
static RNG rng;

static const float PI = 3.14159265f;
static const float INV_PI = 0.318309886f;
#define PI_TWO          6.28318530717958623198
const float3 LUM = float3(0.2126, 0.7152, 0.0722);

float2 _Pixel;
// float _Seed;

struct BLASNode
{
    float3 boundMax;
    float3 boundMin;
    int primitiveEndIdx;
    int Index;  // Child Index Or Primitive Start Index
};
StructuredBuffer<BLASNode> _BNodes;

struct TLASNode
{
    float3 boundMax;
    float3 boundMin;
    int transformIdx;
    int materialIdx;
    int Index;  // Child Index Or BLAS Root Index
};
StructuredBuffer<TLASNode> _TLASNodes;

/// Debug ///
uint _TLASNodesCount;
uint _BNodesCount;

bool _OnlyDrawAlbedo;
bool _OnlyDrawNormals;
bool _OnlyDrawDepth;
bool _UseDirectLightReservoirForPrimaryDirect;
bool _ShowDirectLightReservoirDifference;

/// Debug

struct MaterialData
{
    float4 color;
    float3 emission;
    float emissionIntensity;
    float metallic;
    float smoothness;
    float ior;
    float mode;
    int albedoIdx;
    int emitIdx;
    int metalIdx;
    int normIdx;
    int roughIdx;
};
StructuredBuffer<MaterialData> _Materials;

StructuredBuffer<float3> _Vertices;
StructuredBuffer<int> _Indices;
StructuredBuffer<float3> _Normals;
StructuredBuffer<float4> _Tangents;
StructuredBuffer<float2> _UVs;
StructuredBuffer<float4x4> _Transforms;

Texture2DArray<float4> _AlbedoTextures;
SamplerState sampler_AlbedoTextures;
Texture2DArray<float4> _EmitTextures;
SamplerState sampler_EmitTextures;
Texture2DArray<float4> _MetallicTextures;
SamplerState sampler_MetallicTextures;
Texture2DArray<float4> _NormalTextures;
SamplerState sampler_NormalTextures;
Texture2DArray<float4> _RoughnessTextures;
SamplerState sampler_RoughnessTextures;

float2 _PixelOffset;

// ==================== Multi-Pass Data Structures ====================

struct RayData
{
    float3 origin;
    float3 direction;
    uint   pixelIndex;
    uint   rngState;
    float3 throughput;
    float  lastPdf;
};

struct PathContribution
{
    float3 L;
    float3 throughput;
};

struct BufferSizeData
{
    int traceRays;
    int shadowRays;
};

struct HitData
{
    float3 position;          float  distance;
    float3 normal;            float  mode;
    float3 albedo;            float  emissionIntensity;
    float3 emission;          float  roughness;
    float  metallic;          float  alpha;
    float  ior;
};

struct ShadowRayData
{
    float3 origin;       float  maxDist;
    float3 direction;    uint   pixelIndex;
    float3 illumination; float  selectPdf;
};

struct DirectLightReservoirData
{
    float3 origin;       float  maxDist;
    float3 direction;    float  targetLum;
    float3 contribution; float  weightSum;
    uint   lightType;    uint   lightIndex;
    uint   sampleCount;  float  selectedWeight;
};

globallycoherent RWStructuredBuffer<BufferSizeData> BufferSizes;
RWStructuredBuffer<RayData>          GlobalRays;
RWStructuredBuffer<RayData>          GlobalRays2;
RWStructuredBuffer<HitData>          GlobalHits;
RWStructuredBuffer<ShadowRayData>    ShadowRaysBuffer;
RWStructuredBuffer<DirectLightReservoirData> DirectLightReservoirs;
RWStructuredBuffer<float3>            DirectLightReservoirDifference;
RWStructuredBuffer<PathContribution> GlobalColors;
RWStructuredBuffer<uint>              IndirectArgs;

int  CurBounce;
uint _ScreenWidth;
uint _ScreenHeight;

// ==================== End Multi-Pass ====================

Material GenMaterial(half3 baseColor, half3 emission, half emissionIntensity,
    half metallic, half smoothness, half alpha, float ior,
    int4 indices = -1, half2 uv = 0.0)
{
    if (indices.x >= 0)
    {
        half4 color = _AlbedoTextures.SampleLevel(sampler_AlbedoTextures, float3(uv, indices.x), 0.0);
        baseColor = baseColor * color.rgb;
        alpha = alpha * color.a;
    }
    if (indices.y >= 0)
    {
        half4 metallicRoughness = _MetallicTextures.SampleLevel(sampler_MetallicTextures, float3(uv, indices.y), 0.0);
        metallic = metallicRoughness.r;
        smoothness = metallicRoughness.a;
    }
    if (indices.w >= 0)
    {
        smoothness = _RoughnessTextures.SampleLevel(sampler_RoughnessTextures, float3(uv, indices.w), 0.0).x;
        smoothness = 1.0 - smoothness;
    }
    Material mat;
    mat.alpha = alpha;
    mat.albedo = baseColor;
    mat.metallic = metallic;
    if (indices.z >= 0)
    {
        emission = emission * _EmitTextures.SampleLevel(sampler_EmitTextures, float3(uv, indices.z), 0.0).xyz;
    }
    mat.emissionIntensity = emissionIntensity;
    mat.emission = emission;
    mat.roughness = 1.0 - smoothness;
    mat.ior = ior;
    return mat;
}

RayHit GenRayHit()
{
    RayHit hit;
    hit.position = float3(0.0f, 0.0f, 0.0f);
    hit.distance = 1.#INF;
    hit.normal = float3(0.0f, 0.0f, 0.0f);
    hit.material = GenMaterial(float3(0.0f, 0.0f, 0.0f), float3(0.0f, 0.0f, 0.0f), 0, 0.0f, 0.0f, 1.0f, 1.0f);
    hit.mode = 0.0f;
    hit.should_break = false;
    return hit;
}

Ray GenRay(float3 origin, float3 dir)
{
    Ray ray;
    ray.origin = origin;
    ray.dir = dir;
    ray.invDir = 1.0f / ray.dir;
    return ray;
}


#endif
