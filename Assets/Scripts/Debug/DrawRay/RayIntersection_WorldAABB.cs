using System;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;

public class RayIntersection_WorldAABB : MonoBehaviour
{
    public Camera cam;
    [Range(1,100)] public int rayCount = 20;

    private void OnDrawGizmos()
    {
        if (!enabled || !RayTestUtils.EnsureCamera(cam)) return;
        Random.InitState(12345);

        List<RayTestUtils.MyRay> rays = RayTestUtils.GenerateWorldRays(cam, rayCount);

        var meshNodes  = BVHBuilder.GetMeshNodes();
        var bNodes     = BVHBuilder.GetBLASNodes();
        var transforms = BVHBuilder.GetTransforms();
        var vertices   = BVHBuilder.GetVertices();
        var indices    = BVHBuilder.GetIndices();
        if (meshNodes == null || bNodes == null || transforms == null ||
            vertices == null || indices == null) return;

        foreach (var ray in rays)
        {
            bool   hitAny   = false;
            Vector3 hitPos  = default;
            float  closestT = float.PositiveInfinity;

            for (int m = 0; m < meshNodes.Count; ++m)
            {
                var meshNode     = meshNodes[m];
                var localToWorld = transforms[meshNode.TransformIdx * 2];

                TraverseBLAS_World(
                    ray,
                    meshNode.NodeRootIdx,
                    localToWorld,
                    bNodes,
                    vertices, indices,
                    ref hitAny,
                    ref hitPos,
                    ref closestT);
            }

            RayTestUtils.DrawRay(ray, hitAny, hitPos);
        }
    }

    private void TraverseBLAS_World(
        in RayTestUtils.MyRay ray,
        int rootIdx,
        Matrix4x4 l2w,
        IReadOnlyList<BLASNode> bnodes,
        IReadOnlyList<Vector3> vertices, IReadOnlyList<int> indices,   // ❶
        ref bool hitAny,
        ref Vector3 hitPosWorld,
        ref float closestT)
    {
        Span<int> stack = stackalloc int[32];
        int sp = 0;
        stack[0] = rootIdx;

        while (sp >= 0)
        {
            int idx = stack[sp--];
            var n = bnodes[idx];

            // —— 1. 把局部 AABB 变为世界轴对齐 AABB ——
            Vector3 localSize  = n.BoundMax - n.BoundMin;
            TransformUtils.TransformSize(l2w, localSize, out Vector3 worldSize);
            Vector3 localCenter = (n.BoundMin + n.BoundMax) * 0.5f;
            Vector3 worldCenter = l2w.MultiplyPoint3x4(localCenter);
            Vector3 worldMin    = worldCenter - 0.5f * worldSize;
            Vector3 worldMax    = worldCenter + 0.5f * worldSize;

            // —— 2. 世界射线 ↔ 世界 AABB ——
            if (!IntersectAABB_World(ray, worldMin, worldMax, out float _))
                continue;

            // —— 3. 内部 / 叶子 处理 ——
            if (n.PrimitiveStartIdx < 0)             // 内部
            {
                stack[++sp] = n.ChildIdx;
                stack[++sp] = n.ChildIdx + 1;
            }
            else                                     // 叶子：真正三角形求交
            {
                for (int p = n.PrimitiveStartIdx; p < n.PrimitiveEndIdx; ++p)
                {
                    int tri = p * 3;
                    Vector3 v0 = l2w.MultiplyPoint3x4(vertices[indices[tri]]);
                    Vector3 v1 = l2w.MultiplyPoint3x4(vertices[indices[tri + 1]]);
                    Vector3 v2 = l2w.MultiplyPoint3x4(vertices[indices[tri + 2]]);

                    if (RayTestUtils.IntersectTriangle(ray, v0, v1, v2, out float t))
                    {
                        if (t > 0.0f && t < closestT)
                        {
                            closestT     = t;
                            hitPosWorld  = ray.origin + ray.dir * t;
                            hitAny       = true;
                        }
                    }
                }
            }
        }
    }

    private static bool IntersectAABB_World(in RayTestUtils.MyRay r, Vector3 min, Vector3 max, out float tEnter)
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
}
