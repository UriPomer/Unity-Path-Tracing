using System;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;

[ExecuteAlways]
public class RayBLASIntersection_WorldAABB : MonoBehaviour
{
    public Camera cam;
    [Range(1,100)] public int rayCount = 20;
    public float rayLength = 10f;
    public Color missColor = Color.cyan;
    public Color hitColor  = Color.red;

    private void OnDrawGizmos()
    {
        if (!EnsureCamera()) return;
        Random.InitState(12345);

        // 1. 生成世界射线 ----------------------------------------------------
        List<WorldRay> rays = GenerateWorldRays();

        // 2. 取得 BVH & 变换 -------------------------------------------------
        var meshNodes  = BVHBuilder.GetMeshNodes();
        var bNodes     = BVHBuilder.GetBLASNodes();
        var transforms = BVHBuilder.GetTransforms();
        var vertices   = BVHBuilder.GetVertices();          // ❶ 新增
        var indices    = BVHBuilder.GetIndices();           // ❶ 新增
        if (meshNodes == null || bNodes == null || transforms == null ||
            vertices == null || indices == null) return;

        // 3. 每条射线遍历所有 BLAS -----------------------------------------
        foreach (var ray in rays)
        {
            bool   hitAny   = false;
            Vector3 hitPos  = default;
            float  closestT = float.PositiveInfinity;

            for (int m = 0; m < meshNodes.Count; ++m)
            {
                var meshNode     = meshNodes[m];
                var localToWorld = transforms[meshNode.TransformIdx * 2];

                // 遍历当前 mesh 的 BLAS，用世界射线求交
                TraverseBLAS_World(
                    ray,
                    meshNode.NodeRootIdx,
                    localToWorld,
                    bNodes,
                    vertices, indices,                // ❶ 传入顶点索引
                    ref hitAny,
                    ref hitPos,
                    ref closestT);
            }

            DrawRay(ray, hitAny, hitPos);
        }
    }

    #region —— 生成射线，与前版相同 ——
    private List<WorldRay> GenerateWorldRays()
    {
        int w = cam.pixelWidth, h = cam.pixelHeight;
        var list = new List<WorldRay>(rayCount);
        for (int i = 0; i < rayCount; ++i)
        {
            Vector2 pix = new(
                UnityEngine.Random.Range(0, w),
                UnityEngine.Random.Range(0, h));

            Vector3 dir = PixelToWorldDir(pix, w, h).normalized;
            list.Add(new WorldRay
            {
                origin = cam.transform.position,
                dir    = dir,
                invDir = new Vector3(
                    1f / (Mathf.Abs(dir.x) < 1e-8f ? 1e-8f : dir.x),
                    1f / (Mathf.Abs(dir.y) < 1e-8f ? 1e-8f : dir.y),
                    1f / (Mathf.Abs(dir.z) < 1e-8f ? 1e-8f : dir.z))
            });
        }
        return list;
    }
    private Vector3 PixelToWorldDir(Vector2 pixel, int w, int h)
    {
        Vector2 screen = (pixel + Vector2.one * 0.5f) / new Vector2(w, h);
        Vector2 ndc    = screen * 2f - Vector2.one;
        Vector4 clip   = new(ndc.x, ndc.y, 1f, 1f);
        Vector4 view4  = cam.projectionMatrix.inverse * clip;
        view4         /= Mathf.Max(view4.w, 1e-6f);
        Vector3 viewDir = view4;
        return (cam.cameraToWorldMatrix * new Vector4(viewDir.x, viewDir.y, viewDir.z, 0f));
    }
    #endregion

    #region —— BLAS 遍历（节点 AABB → 世界） ——
    private void TraverseBLAS_World(
        in WorldRay ray,
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

                    if (IntersectTriangle(ray, v0, v1, v2, out float t))
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
    
    private static bool IntersectTriangle(in WorldRay r,
        Vector3 v0, Vector3 v1, Vector3 v2, out float t)
    {
        const float EPS = 1e-6f;
        Vector3 e1 = v1 - v0;
        Vector3 e2 = v2 - v0;
        Vector3 p = Vector3.Cross(r.dir, e2);
        float det = Vector3.Dot(e1, p);
        if (Mathf.Abs(det) < EPS) { t = 0; return false; }
        float invDet = 1.0f / det;
        Vector3 s = r.origin - v0;
        float u = Vector3.Dot(s, p) * invDet;
        if (u < 0 || u > 1) { t = 0; return false; }
        Vector3 q = Vector3.Cross(s, e1);
        float v = Vector3.Dot(r.dir, q) * invDet;
        if (v < 0 || u + v > 1) { t = 0; return false; }
        t = Vector3.Dot(e2, q) * invDet;
        return t >= 0;
    }

    private static bool IntersectAABB_World(in WorldRay r, Vector3 min, Vector3 max, out float tEnter)
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
    #endregion

    #region —— 绘制结果 ——
    private void DrawRay(in WorldRay ray, bool hit, Vector3 hitPoint)
    {
        Color c = hit ? hitColor : missColor;
        Gizmos.color = c;
        Vector3 end = hit ? hitPoint : ray.origin + ray.dir * rayLength;
        Gizmos.DrawLine(ray.origin, end);
        if (hit) Gizmos.DrawSphere(hitPoint, 0.08f);

        Debug.DrawLine(ray.origin, end, c);
    }
    #endregion

    #region —— 数据结构 & 工具 ——
    private struct WorldRay { public Vector3 origin, dir, invDir; }

    private bool EnsureCamera() { if (cam == null) cam = Camera.main; return cam != null; }
    #endregion
}
