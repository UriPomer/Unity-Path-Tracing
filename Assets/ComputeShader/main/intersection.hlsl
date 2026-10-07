#ifndef INTERSECTION
#define INTERSECTION

#include "global.hlsl"
#include "function.hlsl"

#define BVHTREE_RECURSE_SIZE 64 // BVHSAH bounds BLAS depth to 32 and TLAS depth to 63.

/*
 *把光线的起点和方向变换到局部坐标系，然后返回新的光线
 */
Ray PrepareTreeEnterRay(Ray ray, int transformIdx)
{
    float4x4 worldToLocal = _Transforms[transformIdx * 2 + 1];
    float3 origin = mul(worldToLocal, float4(ray.origin, 1.0));     // 把光线的起点变换到局部坐标系
    float3 dir = mul(worldToLocal, float4(ray.dir, 0.0));    // 把光线的方向变换到局部坐标系 ， 但不进行normalize，使得三角形求交的t与世界坐标完全相同，所以PrepareTreeEnterHit不需要变换Distance
    Ray local = GenRay(origin, dir);
    local.coneSpread = ray.coneSpread;
    return local;
}

//这里的u和v就是三角形的两个顶点的uv坐标，t是光线与三角形的交点
bool IntersectTriangle(Ray ray, int triangleBase,
    inout float t, inout float u, inout float v
)
{
    float3 v0 = _Triangles[triangleBase];
    float3 e1 = _Triangles[triangleBase + 1];
    float3 e2 = _Triangles[triangleBase + 2];
    float3 pvec = cross(ray.dir, e2);
    float det = dot(e1, pvec);
    if (abs(det) < 1e-8)
        return false;
    float detInv = 1.0 / det;
    float3 tvec = ray.origin - v0;
    u = dot(tvec, pvec) * detInv;
    if(u < 0.0 || u > 1.0)
        return false;
    float3 qvec = cross(tvec, e1);
    v = dot(ray.dir, qvec) * detInv;
    if(v < 0.0 || v + u > 1.0)
        return false;
    t = dot(e2, qvec) * detInv;
    return true;
}

inline float IntersectBox(Ray ray, float3 pMax, float3 pMin)
{
    // reference: https://github.com/knightcrawler25/GLSL-PathTracer/blob/master/src/shaders/common/intersection.glsl
    // reference: https://medium.com/@bromanz/another-view-on-the-classic-ray-aabb-intersection-algorithm-for-bvh-traversal-41125138b525
    float3 f = (pMax - ray.origin) * ray.invDir;
    float3 n = (pMin - ray.origin) * ray.invDir;
    float3 tMax = max(f, n);
    float3 tMin = min(f, n);
    float dstNear = max(tMin.x, max(tMin.y, tMin.z));
    float dstFar = min(tMax.x, min(tMax.y, tMax.z));
    bool hit = dstNear <= dstFar && dstFar >= 0.0;
    return hit ? dstNear : 1.#INF;
}

inline float RayBoundingBoxDst(const Ray ray, float3 boxMin, float3 boxMax)
{
    float3 tMin = (boxMin - ray.origin) * ray.invDir;
    float3 tMax = (boxMax - ray.origin) * ray.invDir;
    float3 t1   = min(tMin, tMax);
    float3 t2   = max(tMin, tMax);
    float  tNear = max(max(t1.x, t1.y), t1.z);
    float  tFar  = min(min(t2.x, t2.y), t2.z);
    if (tFar >= tNear && tFar > 0.0f)
    {
        return tNear > 0.0f ? tNear : 0.0f;
    }
    return -1.0f;
}

// Visibility only needs coverage; normals and other material textures cannot affect it.
half SurfaceAlpha(MaterialData mat, half2 uv)
{
    half alpha = mat.color.a;
    if (mat.albedoIdx >= 0)
        alpha *= _AlbedoTextures.SampleLevel(sampler_AlbedoTextures, float3(uv, mat.albedoIdx), 0.0).a;
    return alpha;
}

float2 TriangleUV(int triIndexBase, float u, float v)
{
    float2 uv0 = _UVs[_Indices[triIndexBase]];
    float2 uv1 = _UVs[_Indices[triIndexBase + 1]];
    float2 uv2 = _UVs[_Indices[triIndexBase + 2]];
    return uv1 * u + uv2 * v + uv0 * (1.0 - u - v);
}

/*
 *与BLAS树中的三角形面求交
 */
float2 TriangleUVFootprint(Ray ray, int triangleBase, int transformIdx, float distance)
{
    float3x3 localToWorld = (float3x3)_Transforms[transformIdx*2];
    uint i0 = _Indices[triangleBase], i1 = _Indices[triangleBase+1], i2 = _Indices[triangleBase+2];
    float3 e1 = mul(localToWorld,_Vertices[i1]-_Vertices[i0]);
    float3 e2 = mul(localToWorld,_Vertices[i2]-_Vertices[i0]);
    float3 n = cross(e1,e2); float area = length(n);
    if (area < 1e-12) return 0.0;
    n /= area;
    float2 duv1 = _UVs[i1]-_UVs[i0], duv2 = _UVs[i2]-_UVs[i0];
    float3 a = cross(e2,n)/area, b = cross(n,e1)/area;
    float3 worldDirection = normalize(mul(localToWorld,ray.dir));
    float width = distance*ray.coneSpread / max(abs(dot(n,worldDirection)),1e-4);
    return width * float2(length(a*duv1.x+b*duv2.x),length(a*duv1.y+b*duv2.y));
}

void IntersectBlasTree(Ray ray, inout RayHit bestHit, int startIdx, int materialIdx, int transformIdx)
{
    int stack[BVHTREE_RECURSE_SIZE];
    int stackPtr = 0;
    int primitiveIdx;
    int closestTriangle = -1;
    float2 closestBarycentrics = 0.0;
    stack[stackPtr] = startIdx;
    while (stackPtr >= 0 && stackPtr < BVHTREE_RECURSE_SIZE)
    {
        int idx = stack[stackPtr--];    //模拟栈
        BLASNode node = _BNodes[idx];   //获取当前BLAS节点

        float dst = IntersectBox(ray, node.boundMax, node.boundMin);    // 和BLAS的包围盒求交
        bool leaf = node.primitiveEndIdx >= 0;
        if (dst < bestHit.distance)
        {
            if (leaf)
            {
                // 遍历BLAS中的每一个面
                for (primitiveIdx = node.Index; primitiveIdx < node.primitiveEndIdx; primitiveIdx++)
                {
                    int triIndexBase = primitiveIdx * 3;
                    float t, u, v;
                    if (IntersectTriangle(ray, triIndexBase, t, u, v))    //与面求交
                    {
                        if (t > 0.0 && t < bestHit.distance)
                        {
                            MaterialData mat = _Materials[materialIdx];
                            if (mat.mode == 1.0 && SurfaceAlpha(mat, TriangleUV(triIndexBase, u, v)) < 0.5)
                                continue;
                            bestHit.distance = t;
                            closestTriangle = triIndexBase;
                            closestBarycentrics = float2(u, v);
                        }
                    }
                }
            }
            else
            {
                int childIndexA = node.Index;
                int childIndexB = node.Index + 1;
                BLASNode childA = _BNodes[childIndexA];
                BLASNode childB = _BNodes[childIndexB];

                float dstA = RayBoundingBoxDst(ray, childA.boundMin, childA.boundMax);
                float dstB = RayBoundingBoxDst(ray, childB.boundMin, childB.boundMax);

                bool hitA = dstA >= 0.0f && dstA < bestHit.distance;
                bool hitB = dstB >= 0.0f && dstB < bestHit.distance;

                if (!hitA && !hitB)
                    continue;

                if (hitA && hitB) {
                    bool isNearestA      = dstA <= dstB;
                    int  childIndexNear  = isNearestA ? childIndexA : childIndexB;
                    int  childIndexFar   = isNearestA ? childIndexB : childIndexA;
                    stack[++stackPtr]    = childIndexFar;
                    stack[++stackPtr]    = childIndexNear;
                }
                else if (hitA) {
                    stack[++stackPtr] = childIndexA;
                }
                else {
                    stack[++stackPtr] = childIndexB;
                }
            }
        }
    }
    if (closestTriangle < 0) return;
    MaterialData mat = _Materials[materialIdx];
    bestHit.materialIndex = materialIdx;
    float2 uv = TriangleUV(closestTriangle, closestBarycentrics.x, closestBarycentrics.y);
    float3 v0 = _Vertices[_Indices[closestTriangle]];
    float3 v1 = _Vertices[_Indices[closestTriangle + 1]];
    float3 v2 = _Vertices[_Indices[closestTriangle + 2]];
    bestHit.position = ray.origin + bestHit.distance * ray.dir;
    float2 uvFootprint = TriangleUVFootprint(ray,closestTriangle,transformIdx,bestHit.distance);
    bestHit.normal = GetNormal(closestTriangle, closestBarycentrics, mat.normIdx, uv, uvFootprint);
    bestHit.geometryNormal = normalize(cross(v1 - v0, v2 - v0));
    bestHit.material = GenMaterial(mat.color.rgb, mat.emission, mat.emissionIntensity,
        mat.metallic, mat.smoothness, mat.color.a, mat.ior,
        int4(mat.albedoIdx, mat.metalIdx, mat.emitIdx, mat.roughIdx), uv, uvFootprint);
    bestHit.mode = mat.mode;
}

/*
 *判断是否与BLAS树中的三角形面相交
 */
// Threaded visibility traversal visits each overlapping leaf once. Escape
// links end at this BLAS root's boundary; no per-ray traversal stack is needed.
float IntersectBlasVisibility(Ray ray, int startIdx, float targetDist, int materialIdx, bool transmitAlpha)
{
    float transmittance = 1.0;
    int idx = startIdx;
    while (idx >= 0)
    {
        BLASNode node = _BNodes[idx];
        idx = node.escapeIndex;
        float distance = RayBoundingBoxDst(ray,node.boundMin,node.boundMax);
        if (distance < 0.0 || distance >= targetDist) continue;
        if (node.primitiveEndIdx < 0) { idx = node.Index; continue; }
        for (int primitiveIdx = node.Index; primitiveIdx < node.primitiveEndIdx; primitiveIdx++)
        {
            int triIndexBase = primitiveIdx * 3;
            float t, u, v;
            if (IntersectTriangle(ray, triIndexBase, t, u, v))
            {
                if (t > 0.0 && t < targetDist)
                {
                    MaterialData mat = _Materials[materialIdx];
                    if (mat.mode == 1.0 || (transmitAlpha && mat.mode == 2.0))
                    {
                        half alpha = SurfaceAlpha(mat, TriangleUV(triIndexBase, u, v));
                        if (mat.mode == 1.0 && alpha < 0.5) continue;
                        if (mat.mode == 2.0)
                        {
                            transmittance *= 1.0 - saturate(alpha);
                            if (transmittance > 0.0) continue;
                        }
                    }
                    return 0.0;
                }
            }
        }
    }
    return transmittance;
}

void IntersectTlas(Ray ray, inout RayHit bestHit)
{
    if (_TLASNodesCount == 0u) return;
    int stack[BVHTREE_RECURSE_SIZE];
    int stackIndex = 0;
    stack[0] = 0;                         // 根始终是 0

    while (stackIndex >= 0)
    {
        int idx = stack[stackIndex--];
        TLASNode n = _TLASNodes[idx];

        // ray、bounds都是世界坐标
        if (IntersectBox(ray, n.boundMax, n.boundMin) < bestHit.distance)
        {
            if (n.transformIdx >= 0)               // ---------- 叶子 ----------
            {
                Ray localRay = PrepareTreeEnterRay(ray, n.transformIdx);
                RayHit localHit = GenRayHit();
                localHit.distance = bestHit.distance;
                IntersectBlasTree(localRay, localHit, n.Index, n.materialIdx, n.transformIdx);
                if (localHit.distance < bestHit.distance)
                {
                    bestHit = localHit;
                    bestHit.instanceIndex = n.transformIdx;
                    bestHit.position = ray.origin + ray.dir * localHit.distance;
                    float3x3 worldToLocal = (float3x3)_Transforms[n.transformIdx * 2 + 1];
                    bestHit.normal = normalize(mul(localHit.normal, worldToLocal));
                    bestHit.geometryNormal = normalize(mul(localHit.geometryNormal, worldToLocal));
                }
            }
            else
            {
                int left  = n.Index;
                int right = n.Index + 1;

                float dstL = RayBoundingBoxDst(ray,
                                               _TLASNodes[left].boundMin,
                                               _TLASNodes[left].boundMax);
                float dstR = RayBoundingBoxDst(ray,
                                               _TLASNodes[right].boundMin,
                                               _TLASNodes[right].boundMax);

                bool hitL = dstL >= 0.0f;
                bool hitR = dstR >= 0.0f;

                if (!hitL && !hitR)
                    continue;

                if (hitL && hitR) {
                    bool swap     = dstR < dstL;
                    int  nearIdx  = swap ? right : left;
                    int  farIdx   = swap ? left  : right;
                    float nearDst = swap ? dstR   : dstL;
                    float farDst  = swap ? dstL   : dstR;

                    if (farDst < bestHit.distance) stack[++stackIndex] = farIdx;
                    if (nearDst< bestHit.distance) stack[++stackIndex] = nearIdx;
                }
                else if (hitL) {
                    if (dstL < bestHit.distance) stack[++stackIndex] = left;
                }
                else {
                    if (dstR < bestHit.distance) stack[++stackIndex] = right;
                }
            }
        }
    }
}

float TraceVisibility(Ray ray, float targetDist, bool transmitAlpha)
{
    if (_TLASNodesCount == 0u) return 1.0;
    float transmittance = 1.0;
    int idx = 0;
    while (idx >= 0)
    {
        TLASNode node = _TLASNodes[idx];
        idx = node.escapeIndex;
        float distance = RayBoundingBoxDst(ray,node.boundMin,node.boundMax);
        if (distance < 0.0 || distance >= targetDist) continue;
        if (node.transformIdx < 0) { idx = node.Index; continue; }
        Ray localRay = PrepareTreeEnterRay(ray,node.transformIdx);
        transmittance *= IntersectBlasVisibility(localRay,node.Index,targetDist,node.materialIdx,transmitAlpha);
        if (transmittance <= 0.0) return 0.0;
    }
    return transmittance;
}

bool IntersectTlasFast(Ray ray, float targetDist)
{
    return TraceVisibility(ray, targetDist, false) <= 0.0;
}

#endif
