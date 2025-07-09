using System;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;

/// <summary>
/// 把 20 条随机屏幕射线转换到 BLAS 局部空间并做层次包围盒相交测试。
/// 命中则射线呈红色并在命中点画小球；未命中则呈青色。
/// </summary>
public class RayIntersection_LocalAABB : MonoBehaviour
{
    public Camera cam;
    [Range(1, 100)] public int rayCount = 20;

    private void OnDrawGizmos()
    {
        if (!enabled || !RayTestUtils.EnsureCamera(cam)) return;
        Random.InitState(12345);

        List<RayTestUtils.MyRay> rays = RayTestUtils.GenerateWorldRays(cam, rayCount);

        var meshNodes  = BVHBuilder.GetMeshNodes();
        var bNodes     = BVHBuilder.GetBLASNodes();
        var transforms = BVHBuilder.GetTransforms();
        var vertices   = BVHBuilder.GetVertices();      // ★① 取顶点
        var indices    = BVHBuilder.GetIndices();       // ★① 取索引
        if (meshNodes==null||bNodes==null||transforms==null||
            vertices==null||indices==null) return;

        foreach (var ray in rays)
        {
            bool    hitAny   = false;
            Vector3 hitPoint = default;
            float   closest  = float.PositiveInfinity;

            for (int i = 0; i < meshNodes.Count; ++i)
            {
                var meshNode     = meshNodes[i];
                var localToWorld = transforms[meshNode.TransformIdx * 2];
                var worldToLocal = transforms.Count > meshNode.TransformIdx * 2 + 1
                    ? transforms[meshNode.TransformIdx * 2 + 1]
                    : localToWorld.inverse;

                RayTestUtils.MyRay localRay = WorldToLocalRay(ray, worldToLocal);

                if (TraverseBLAS(localRay, meshNode.NodeRootIdx, bNodes,
                        vertices, indices,
                        out float tLocal))
                {
                    Vector3 localHit = localRay.origin + localRay.dir * tLocal;
                    Vector3 worldHit = localToWorld.MultiplyPoint3x4(localHit);

                    float tWorld = Vector3.Dot(worldHit - ray.origin, ray.dir);
                    if (tWorld < closest)
                    {
                        closest  = tWorld;
                        hitPoint = worldHit;
                        hitAny   = true;
                    }
                }
            }

            RayTestUtils.DrawRay(ray, hitAny, hitPoint);
        }
    }

    private bool TraverseBLAS(
        RayTestUtils.MyRay ray,
        int rootIdx,
        IReadOnlyList<BLASNode> bnodes,
        IReadOnlyList<Vector3> vertices, IReadOnlyList<int> indices,   // ★② 新参数
        out float tHit)
    {
        tHit = float.PositiveInfinity;
        bool hitAny = false;

        Span<int> stack = stackalloc int[32];
        int sp = 0;
        stack[0] = rootIdx;

        while (sp >= 0)
        {
            int idx = stack[sp--];
            var node = bnodes[idx];

            if (!IntersectAABB(ray, node.BoundMin, node.BoundMax, out float _))
                continue;

            if (node.PrimitiveEndIdx < 0)          // 内部
            {
                stack[++sp] = node.Index;
                stack[++sp] = node.Index + 1;
            }
            else                                     // 叶子：遍历三角形
            {
                for (int p = node.Index; p < node.PrimitiveEndIdx; ++p) // ★③
                {
                    int tri = p * 3;
                    Vector3 v0 = vertices[indices[tri    ]];
                    Vector3 v1 = vertices[indices[tri + 1]];
                    Vector3 v2 = vertices[indices[tri + 2]];

                    if (RayTestUtils.IntersectTriangle(ray, v0, v1, v2, out float tLocal))
                    {
                        if (tLocal > 0f && tLocal < tHit)
                        {
                            tHit   = tLocal;
                            hitAny = true;
                        }
                    }
                }
            }
        }
        return hitAny;
    }
    
    private static bool IntersectAABB(RayTestUtils.MyRay r, Vector3 min, Vector3 max, out float tEnter)
    {
        float tmin = (min.x - r.origin.x) * r.invDir.x;
        float tmax = (max.x - r.origin.x) * r.invDir.x;
        if (tmin > tmax) (tmin, tmax) = (tmax, tmin);

        float tymin = (min.y - r.origin.y) * r.invDir.y;
        float tymax = (max.y - r.origin.y) * r.invDir.y;
        if (tymin > tymax) (tymin, tymax) = (tymax, tymin);

        if (tmin > tymax || tymin > tmax) { tEnter = 0; return false; }
        if (tymin > tmin) tmin = tymin;
        if (tymax < tmax) tmax = tymax;

        float tzmin = (min.z - r.origin.z) * r.invDir.z;
        float tzmax = (max.z - r.origin.z) * r.invDir.z;
        if (tzmin > tzmax) (tzmin, tzmax) = (tzmax, tzmin);

        if (tmin > tzmax || tzmin > tmax) { tEnter = 0; return false; }
        if (tzmin > tmin) tmin = tzmin;
        if (tzmax < tmax) tmax = tzmax;

        tEnter = tmin;
        return tmax >= 0;
    }

    private static RayTestUtils.MyRay WorldToLocalRay(in RayTestUtils.MyRay wr, Matrix4x4 worldToLocal)
    {
        Vector3 o = worldToLocal.MultiplyPoint3x4(wr.origin);
        Vector3 d = worldToLocal.MultiplyVector(wr.dir);
        return new RayTestUtils.MyRay
        {
            origin  = o,
            dir     = d,
            invDir  = new Vector3(
                1f / (Mathf.Abs(d.x) < 1e-8f ? 1e-8f : d.x),
                1f / (Mathf.Abs(d.y) < 1e-8f ? 1e-8f : d.y),
                1f / (Mathf.Abs(d.z) < 1e-8f ? 1e-8f : d.z))
        };
    }
}
