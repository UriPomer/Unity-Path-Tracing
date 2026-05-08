#ifndef INTERSECTION
#define INTERSECTION

#include "global.hlsl"
#include "function.hlsl"

#define BVHTREE_RECURSE_SIZE 32

/*
 *把光线的起点和方向变换到局部坐标系，然后返回新的光线
 */
Ray PrepareTreeEnterRay(Ray ray, int transformIdx)
{
    float4x4 worldToLocal = _Transforms[transformIdx * 2 + 1];
    float3 origin = mul(worldToLocal, float4(ray.origin, 1.0));     // 把光线的起点变换到局部坐标系
    float3 dir = mul(worldToLocal, float4(ray.dir, 0.0));    // 把光线的方向变换到局部坐标系 ， 但不进行normalize，使得三角形求交的t与世界坐标完全相同，所以PrepareTreeEnterHit不需要变换Distance
    return GenRay(origin, dir);
}

/*
 *变换长度为targetDist的向量到局部坐标系，然后返回新的长度
 */
float PrepareTreeEnterTargetDistance(float targetDist, int transformIdx, float3 rayDir)
{
    float4x4 worldToLocal = _Transforms[transformIdx * 2 + 1];
    if (!isfinite(targetDist))
    {
        return targetDist;
    }
    float3 vecWorld = rayDir * targetDist;  
    float3 vecLocal = mul(worldToLocal, float4(vecWorld, 0.0)).xyz;
    return vecLocal;
}

/*
 *将RayHit hit的属性变换到局部坐标系
 */
void PrepareTreeEnterHit(Ray rayLocal, inout RayHit hit, int transformIdx)
{
    if (isfinite(hit.distance))
    {
        float4x4 worldToLocal = _Transforms[transformIdx * 2 + 1];
        hit.position = mul(worldToLocal, float4(hit.position, 1.0)).xyz;
        // hit.distance = length(hit.position - rayLocal.origin);
        hit.normal = normalize(mul(worldToLocal, float4(hit.normal, 0.0)).xyz);
    }
}

/*
 *将RayHit hit的属性变换到世界坐标系
 */
void PrepareTreeExit(Ray rayWorld, inout RayHit hit, int transformIdx)
{
    if (isfinite(hit.distance))
    {
        float4x4 localToWorld = _Transforms[transformIdx * 2];
        hit.position = mul(localToWorld, float4(hit.position, 1.0)).xyz;
        hit.distance = length(hit.position - rayWorld.origin);
        hit.normal = normalize(mul(localToWorld, float4(hit.normal, 0.0)).xyz);
    }
}

/*
 *与地面相交，数据存储在bestHit中
 */
void IntersectGround(Ray ray, inout RayHit bestHit, float yVal = 0.0)
{
    float t = (yVal - ray.origin.y) / ray.dir.y;
    if(t > 0.0 && t < bestHit.distance)
    {
        bestHit.position = ray.origin + t * ray.dir;
        bestHit.distance = t;
        bestHit.normal = float3(0.0, 1.0, 0.0);
        bestHit.material = GenMaterial(float3(1.0, 1.0, 1.0), float3(0.0, 0.0, 0.0), 0, 0.0, 0.0, 1.0, 1.0);
    }
}

/*
 *通过targetDist快速判断光线是否与地面相交
 */
bool IntersectGroundFast(Ray ray, float targetDist, float yVal = 0.0)
{
    float t = -(ray.origin.y - yVal) / ray.dir.y;
    if (t > 0.0 && t < targetDist)
        return true;
    return false;
}

//这里的u和v就是三角形的两个顶点的uv坐标，t是光线与三角形的交点
bool IntersectTriangle(Ray ray, float3 v0, float3 v1, float3 v2,
    inout float t, inout float u, inout float v
)
{
    float3 e1 = v1 - v0;
    float3 e2 = v2 - v0;
    float3 pvec = cross(ray.dir, e2);
    float det = dot(e1, pvec);
    if (det < 1e-8)
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

/*
 *判断光线是否与盒子相交
 */
bool IntersectBox2(Ray ray, float3 pMax, float3 pMin)
{
    // reference: https://github.com/knightcrawler25/GLSL-PathTracer/blob/master/src/shaders/common/intersection.glsl
    // reference: https://medium.com/@bromanz/another-view-on-the-classic-ray-aabb-intersection-algorithm-for-bvh-traversal-41125138b525
    float3 f = (pMax - ray.origin) * ray.invDir;
    float3 n = (pMin - ray.origin) * ray.invDir;
    float3 tMax = max(f, n);
    float3 tMin = min(f, n);
    float t0 = max(tMin.x, max(tMin.y, tMin.z));
    float t1 = min(tMax.x, min(tMax.y, tMax.z));
    return t1 >= t0;
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

/*
 *与BLAS树中的三角形面求交
 */
void IntersectBlasTree(Ray ray, inout RayHit bestHit, int startIdx, int materialIdx)
{
    int stack[BVHTREE_RECURSE_SIZE];
    int stackPtr = 0;
    int primitiveIdx;
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
                    float3 v0 = _Vertices[_Indices[triIndexBase]];
                    float3 v1 = _Vertices[_Indices[triIndexBase + 1]];
                    float3 v2 = _Vertices[_Indices[triIndexBase + 2]];
                    float t, u, v;
                    if (IntersectTriangle(ray, v0, v1, v2, t, u, v))    //与面求交
                    {
                        float2 uv0 = _UVs[_Indices[triIndexBase]];
                        float2 uv1 = _UVs[_Indices[triIndexBase + 1]];
                        float2 uv2 = _UVs[_Indices[triIndexBase + 2]];
                        if (t > 0.0 && t < bestHit.distance)    //距离更近且不为负数
                        {
                            MaterialData mat = _Materials[materialIdx];
                            float3 hitPos = ray.origin + t * ray.dir;
                            float2 uv = uv1 * u + uv2 * v + uv0 * (1.0 - u - v);    //插值uv
                            float3 norm = GetNormal(triIndexBase, float2(u, v), mat.normIdx, uv);
                            Material mats = GenMaterial(
                                mat.color.rgb, mat.emission, mat.emissionIntensity, mat.metallic, mat.smoothness, mat.color.a, mat.ior,
                                int4(mat.albedoIdx, mat.metalIdx, mat.emitIdx, mat.roughIdx), uv
                            );
                            if (mat.mode == 1.0 && mats.alpha < 1.0 ||
                                    (mat.mode > 1.0 && SkipTransparent(mats)))   // 如果这是一个透明材质，那么则忽略它与光线的相交
                                continue;
                            bestHit.distance = t;
                            bestHit.position = hitPos;
                            bestHit.normal = norm;
                            bestHit.material = mats;
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
}

/*
 *判断是否与BLAS树中的三角形面相交
 */
bool IntersectBlasTreeFast(Ray ray, int startIdx, float targetDist, int materialIdx)
{
    int stack[BVHTREE_RECURSE_SIZE];
    int stackPtr = 0;
    int primitiveIdx;
    stack[stackPtr] = startIdx;
    while (stackPtr >= 0 && stackPtr < BVHTREE_RECURSE_SIZE)
    {
        int idx = stack[stackPtr--];
        BLASNode node = _BNodes[idx];
        // check if ray intersect with bounding box
        float dst = IntersectBox(ray, node.boundMax, node.boundMin);
        bool leaf = node.primitiveEndIdx >= 0;
        if (dst < targetDist)
        {
            if (leaf)
            {
                for (primitiveIdx = node.Index; primitiveIdx < node.primitiveEndIdx; primitiveIdx++)
                {
                    int triIndexBase = primitiveIdx * 3;
                    float3 v0 = _Vertices[_Indices[triIndexBase]];
                    float3 v1 = _Vertices[_Indices[triIndexBase + 1]];
                    float3 v2 = _Vertices[_Indices[triIndexBase + 2]];
                    float2 uv0 = _UVs[_Indices[triIndexBase]];
                    float2 uv1 = _UVs[_Indices[triIndexBase + 1]];
                    float2 uv2 = _UVs[_Indices[triIndexBase + 2]];
                    float t, u, v;
                    if (IntersectTriangle(ray, v0, v1, v2, t, u, v))
                    {
                        if (t > 0.0 && t < targetDist)
                        {
                            MaterialData mat = _Materials[materialIdx];
                            float2 uv = uv1 * u + uv2 * v + uv0 * (1.0 - u - v);
                            Material mats = GenMaterial(
                                mat.color.rgb, mat.emission, mat.emissionIntensity, mat.metallic, mat.smoothness, mat.color.a, mat.ior,
                                int4(mat.albedoIdx, mat.metalIdx, mat.emitIdx, mat.roughIdx), uv
                            );
                            if (mat.mode == 1.0 && mats.alpha < 1.0 ||
                                    (mat.mode > 1.0 && SkipTransparent(mats)))
                                continue;
                            return true;
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

                bool hitA = dstA >= 0.0f && dstA < targetDist;
                bool hitB = dstB >= 0.0f && dstB < targetDist;

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
    return false;
}

void IntersectTlas(Ray ray, inout RayHit bestHit)
{
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
                PrepareTreeEnterHit(localRay, bestHit, n.transformIdx);
                IntersectBlasTree(localRay, bestHit,
                                  n.Index, n.materialIdx);
                PrepareTreeExit(ray, bestHit, n.transformIdx);
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

bool IntersectTlasFast(Ray ray, float targetDist)
{
    int stack[BVHTREE_RECURSE_SIZE];
    int stackPtr = 0;
    stack[0] = 0;

    while (stackPtr >= 0)
    {
        int idx = stack[stackPtr--];
        TLASNode n = _TLASNodes[idx];

        float dst = IntersectBox(ray, n.boundMax, n.boundMin);

        if (dst < targetDist)
        {
            if (n.transformIdx >= 0)               // 叶子
            {
                // float localDist = PrepareTreeEnterTargetDistance(targetDist, n.transformIdx, ray.dir);
                Ray   localRay  = PrepareTreeEnterRay(ray, n.transformIdx);
                if (IntersectBlasTreeFast(localRay, n.Index, targetDist, n.materialIdx))
                    return true;
            }
            else
            {
                int left  = n.Index;
                int right = n.Index + 1;

                float dstL = RayBoundingBoxDst(ray,
                                               _TLASNodes[left ].boundMin,
                                               _TLASNodes[left ].boundMax);
                float dstR = RayBoundingBoxDst(ray,
                                               _TLASNodes[right].boundMin,
                                               _TLASNodes[right].boundMax);

                bool hitL = dstL >= 0.0f && dstL < targetDist;
                bool hitR = dstR >= 0.0f && dstR < targetDist;

                if (!hitL && !hitR)
                    continue;

                if (hitL && hitR) {
                    bool swap    = dstR < dstL;
                    int  nearIdx = swap ? right : left;
                    int  farIdx  = swap ? left  : right;
                    stack[++stackPtr] = farIdx;
                    stack[++stackPtr] = nearIdx;
                }
                else if (hitL) {
                    stack[++stackPtr] = left;
                }
                else { // hitR
                    stack[++stackPtr] = right;
                }
            }
        }
    }
    return false;
}

#endif
