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
    float4x4 worldToLocal = AffineInverse(_Transforms[transformIdx]);
    float3 origin = mul(worldToLocal, float4(ray.origin, 1.0)).xyz;     // 把光线的起点变换到局部坐标系
    float3 dir = normalize(mul(worldToLocal, float4(ray.dir, 0.0)).xyz);    // 把光线的方向变换到局部坐标系
    return GenRay(origin, dir);
}

/*
 *变换长度为targetDist的向量到局部坐标系，然后返回新的长度
 */
float PrepareTreeEnterTargetDistance(float targetDist, int transformIdx)
{
    float4x4 worldToLocal = AffineInverse(_Transforms[transformIdx]);
    if (targetDist >= 1.#INF)
    {
        return targetDist;
    }
    else
    {
        // 变换长度为targetDist的向量到局部坐标系，然后返回新的长度
        float3 dir = mul(worldToLocal, float4(targetDist, 0.0, 0.0, 0.0)).xyz;
        return length(dir);
    }
}

/*
 *将RayHit hit的属性变换到局部坐标系
 */
void PrepareTreeEnterHit(Ray rayLocal, inout RayHit hit, int transformIdx)
{
    float4x4 worldToLocal = AffineInverse(_Transforms[transformIdx]);
    if (hit.distance < 1.#INF)
    {
        hit.position = mul(worldToLocal, float4(hit.position, 1.0)).xyz;
        hit.distance = length(hit.position - rayLocal.origin);
    }
}

/*
 *将RayHit hit的属性变换到世界坐标系
 */
void PrepareTreeExit(Ray rayWorld, inout RayHit hit, int transformIdx)
{
    float4x4 localToWorld = _Transforms[transformIdx];
    if (hit.distance < 1.#INF)
    {
        hit.position = mul(localToWorld, float4(hit.position, 1.0)).xyz;
        hit.distance = length(hit.position - rayWorld.origin);
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
        bestHit.material = GenMaterial(float3(1.0, 1.0, 1.0), float3(0.0, 0.0, 0.0), 0.0, 0.0, 1.0, 1.0);
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
bool IntersectBox1(Ray ray, float3 pMax, float3 pMin)
{
    float t0 = 0.0;
    float t1 = 1.#INF;
    float invRayDir, tNear, tFar;
    for (int i = 0; i < 3; i++)
    {
        invRayDir = 1.0 / ray.dir[i];
        tNear = (pMin[i] - ray.origin[i]) * invRayDir;
        tFar = (pMax[i] - ray.origin[i]) * invRayDir;
        t0 = max(t0, tNear);
        t1 = min(t1, tFar);
        if (t0 > t1)
        {
            return false;
        }
    }
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

/*
 *判断光线是否与盒子相交，同时考虑光线的双向
 */
bool IntersectBox3(Ray ray, RayHit bestHit, float3 pMax, float3 pMin)
{
    bool intersectForward = IntersectBox2(ray, pMax, pMin);
    bool intersectBackward = bestHit.distance < 1.#INF ? IntersectBox2(GenRay(bestHit.position, -ray.dir), pMax, pMin) : true;
    return intersectForward && intersectBackward;
}

float RayBoundingBoxDst(Ray ray, float3 boxMin, float3 boxMax)
{
    float3 tMin = (boxMin - ray.origin) * ray.invDir;
    float3 tMax = (boxMax - ray.origin) * ray.invDir;
    float3 t1 = min(tMin, tMax);
    float3 t2 = max(tMin, tMax);
    float tNear = max(max(t1.x, t1.y), t1.z);
    float tFar = min(min(t2.x, t2.y), t2.z);

    bool hit = tFar >= tNear && tFar > 0;
    float dst = hit ? tNear > 0 ? tNear : 0 : 1.#INF;
    return dst;
};

/*
 *与BLAS树中的三角形面求交
 */
void IntersectBlasTree(Ray ray, inout RayHit bestHit, int startIdx, int transformIdx)
{
    int stack[BVHTREE_RECURSE_SIZE];
    int stackPtr = 0;
    int primitiveIdx;
    stack[stackPtr] = startIdx;
    float4x4 localToWorld = _Transforms[transformIdx];
    while (stackPtr >= 0 && stackPtr < BVHTREE_RECURSE_SIZE)
    {
        int idx = stack[stackPtr--];    //模拟栈
        BLASNode node = _BNodes[idx];   //获取当前BLAS节点

        bool hit = IntersectBox2(ray, node.boundMax, node.boundMin);    // 和BLAS的包围盒求交
        bool leaf = node.primitiveStartIdx >= 0;
        if (hit)
        {
            if (leaf)
            {
                // 遍历BLAS中的每一个面
                for (primitiveIdx = node.primitiveStartIdx; primitiveIdx < node.primitiveEndIdx; primitiveIdx++)
                {
                    // 根据之前得出的结论，这里的_Indices对应OrderedPrimitiveIndices，然后每一个BLASNode的primitiveStartIdx和primitiveEndIdx对应的是OrderedPrimitiveIndices的索引
                    // 然后再通过OrderedPrimitiveIndices的索引找到实际的面的索引
                    int i = primitiveIdx * 3;   // i是面的索引
                    float3 v0 = _Vertices[_Indices[i]];
                    float3 v1 = _Vertices[_Indices[i + 1]];
                    float3 v2 = _Vertices[_Indices[i + 2]];
                    float2 uv0 = _UVs[_Indices[i]];
                    float2 uv1 = _UVs[_Indices[i + 1]];
                    float2 uv2 = _UVs[_Indices[i + 2]];
                    float t, u, v;
                    if (IntersectTriangle(ray, v0, v1, v2, t, u, v))    //与面求交
                    {
                        if (t > 0.0 && t < bestHit.distance)    //距离更近且不为负数
                        {
                            MaterialData mat = _Materials[node.materialIdx];
                            float3 hitPos = ray.origin + t * ray.dir;
                            float2 uv = uv1 * u + uv2 * v + uv0 * (1.0 - u - v);    //插值uv
                            float3 norm = GetNormal(i, float2(u, v), mat.normIdx, uv);
                            Material mats = GenMaterial(
                                mat.color.rgb, mat.emission, mat.metallic, mat.smoothness, mat.color.a, mat.ior,
                                int4(mat.albedoIdx, mat.metalIdx, mat.emitIdx, mat.roughIdx), uv
                            );
                            if (mat.mode == 1.0 && mats.alpha < 1.0)    // 如果这是一个透明材质，那么则忽略它与光线的相交
                                continue;
                            bestHit.distance = t;
                            bestHit.position = hitPos;
                            bestHit.normal = normalize(mul(localToWorld, float4(norm, 0.0)).xyz);
                            bestHit.material = mats;
                        }
                    }
                }
            }
            else
            {
                int childIndexA = node.childIdx;
                int childIndexB = node.childIdx + 1;
                BLASNode childA = _BNodes[childIndexA];
                BLASNode childB = _BNodes[childIndexB];

                float dstA = RayBoundingBoxDst(ray, childA.boundMin, childA.boundMax);
                float dstB = RayBoundingBoxDst(ray, childB.boundMin, childB.boundMax);

                bool isNearestA = dstA <= dstB;
                float dstNear = isNearestA ? dstA : dstB;
                float dstFar = isNearestA ? dstB : dstA;
                int childIndexNear = isNearestA ? childIndexA : childIndexB;
                int childIndexFar = isNearestA ? childIndexB : childIndexA;

                if (dstFar < bestHit.distance) stack[++stackPtr] = childIndexFar;
                if (dstNear < bestHit.distance) stack[++stackPtr] = childIndexNear;
            }
        }
    }
}

/*
 *判断是否与BLAS树中的三角形面相交
 */
bool IntersectBlasTreeFast(Ray ray, int startIdx, float targetDist)
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
        bool hit = IntersectBox2(ray, node.boundMax, node.boundMin);
        bool leaf = node.primitiveStartIdx >= 0;
        if (hit)
        {
            if (leaf)
            {
                for (primitiveIdx = node.primitiveStartIdx; primitiveIdx < node.primitiveEndIdx; primitiveIdx++)
                {
                    int i = primitiveIdx * 3;
                    float3 v0 = _Vertices[_Indices[i]];
                    float3 v1 = _Vertices[_Indices[i + 1]];
                    float3 v2 = _Vertices[_Indices[i + 2]];
                    float2 uv0 = _UVs[_Indices[i]];
                    float2 uv1 = _UVs[_Indices[i + 1]];
                    float2 uv2 = _UVs[_Indices[i + 2]];
                    float t, u, v;
                    if (IntersectTriangle(ray, v0, v1, v2, t, u, v))
                    {
                        if (t > 0.0 && t < targetDist)
                        {
                            MaterialData mat = _Materials[node.materialIdx];
                            float2 uv = uv1 * u + uv2 * v + uv0 * (1.0 - u - v);
                            Material mats = GenMaterial(
                                mat.color.rgb, mat.emission, mat.metallic, mat.smoothness, mat.color.a, mat.ior,
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
                stack[++stackPtr] = node.childIdx;
                stack[++stackPtr] = node.childIdx + 1;
            }
        }
    }
    return false;
}

// 栈大小保持与 BLAS 相同
void IntersectTlas(Ray ray, inout RayHit bestHit)
{
    int stack[BVHTREE_RECURSE_SIZE];
    int sp = 0;
    stack[0] = 0;                         // 根始终是 0

    while (sp >= 0)
    {
        int idx = stack[sp--];
        MeshNode n = _MeshNodes[idx];

        // 统一世界坐标包围盒
        if (!IntersectBox2(ray, n.boundMax, n.boundMin))
            continue;

        if (n.childIdx < 0)               // ---------- 叶子 ----------
        {
            Ray localRay = PrepareTreeEnterRay(ray, n.transformIdx);
            PrepareTreeEnterHit(localRay, bestHit, n.transformIdx);
            IntersectBlasTree(localRay, bestHit,
                              n.rootIdx, n.transformIdx);
            PrepareTreeExit(ray, bestHit, n.transformIdx);
        }
        else                              // ---------- 内部 ----------
        {
            int left  = n.childIdx;
            int right = n.childIdx + 1;

            float dstL = RayBoundingBoxDst(ray,
                                           _MeshNodes[left ].boundMin,
                                           _MeshNodes[left ].boundMax);
            float dstR = RayBoundingBoxDst(ray,
                                           _MeshNodes[right].boundMin,
                                           _MeshNodes[right].boundMax);

            // 近 -> 远 压栈
            bool swap = dstR < dstL;
            int nearIdx = swap ? right : left;
            int farIdx  = swap ? left  : right;
            float nearDst = swap ? dstR : dstL;
            float farDst  = swap ? dstL : dstR;

            if (farDst  < bestHit.distance) stack[++sp] = farIdx;
            if (nearDst < bestHit.distance) stack[++sp] = nearIdx;
        }
    }
}


bool IntersectTlasFast(Ray ray, float targetDist)
{
    int stack[BVHTREE_RECURSE_SIZE];
    int sp = 0;
    stack[0] = 0;

    while (sp >= 0)
    {
        int idx = stack[sp--];
        MeshNode n = _MeshNodes[idx];

        if (!IntersectBox2(ray, n.boundMax, n.boundMin))
            continue;

        if (n.childIdx < 0)               // 叶子
        {
            float localDist = PrepareTreeEnterTargetDistance(targetDist, n.transformIdx);
            Ray   localRay  = PrepareTreeEnterRay(ray, n.transformIdx);
            if (IntersectBlasTreeFast(localRay, n.rootIdx, localDist))
                return true;
        }
        else                              // 内部
        {
            int left  = n.childIdx;
            int right = n.childIdx + 1;

            float dstL = RayBoundingBoxDst(ray,
                                           _MeshNodes[left ].boundMin,
                                           _MeshNodes[left ].boundMax);
            float dstR = RayBoundingBoxDst(ray,
                                           _MeshNodes[right].boundMin,
                                           _MeshNodes[right].boundMax);

            bool  swap = dstR < dstL;
            int   nearIdx = swap ? right : left;
            int   farIdx  = swap ? left  : right;
            float nearDst = min(dstL, dstR);
            float farDst  = max(dstL, dstR);

            if (farDst  < targetDist) stack[++sp] = farIdx;
            if (nearDst < targetDist) stack[++sp] = nearIdx;
        }
    }
    return false;
}




#endif