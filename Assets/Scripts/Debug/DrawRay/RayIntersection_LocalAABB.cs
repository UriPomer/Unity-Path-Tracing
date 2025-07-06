using System;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;

/// <summary>
/// 把 20 条随机屏幕射线转换到 BLAS 局部空间并做层次包围盒相交测试。
/// 命中则射线呈红色并在命中点画小球；未命中则呈青色。
/// </summary>
[ExecuteAlways]
public class RayIntersection_LocalAABB : MonoBehaviour
{
    public Camera cam;
    [Range(1, 100)] public int rayCount = 20;
    public float rayLength = 10f;
    public Color missColor = Color.cyan;
    public Color hitColor  = Color.red;

    #region ✦ 主流程：编辑器 & 运行时都可见 ✦
    private void OnDrawGizmos()
    {
        if (!EnsureCamera()) return;
        Random.InitState(12345);

        List<WorldRay> rays = GenerateWorldRays();

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

                LocalRay localRay = WorldToLocalRay(ray, worldToLocal);

                // ★② 传入 vertices / indices
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

            DrawRay(ray, hitAny, hitPoint);
        }
    }
    #endregion

    #region ✦ 生成世界射线 ✦
    private List<WorldRay> GenerateWorldRays()
    {
        int w = cam.pixelWidth, h = cam.pixelHeight;
        var list = new List<WorldRay>(rayCount);

        for (int i = 0; i < rayCount; ++i)
        {
            Vector2 pixel = new(
                UnityEngine.Random.Range(0, w),
                UnityEngine.Random.Range(0, h));

            Vector3 dir = PixelToWorldDir(pixel, w, h).normalized;
            list.Add(new WorldRay
            {
                origin = cam.transform.position,
                dir    = dir,
                invDir = new Vector3(
                    1f / (dir.x == 0 ? 1e-8f : dir.x),
                    1f / (dir.y == 0 ? 1e-8f : dir.y),
                    1f / (dir.z == 0 ? 1e-8f : dir.z))
            });
        }
        return list;
    }

    // 与 ComputeShader 同线性代数流程
    private Vector3 PixelToWorldDir(Vector2 pixel, int w, int h)
    {
        Vector2 screen   = (pixel + Vector2.one * 0.5f) / new Vector2(w, h);
        Vector2 ndc      = screen * 2f - Vector2.one;
        Vector4 clipPos  = new(ndc.x, ndc.y, 1f, 1f);
        Vector4 viewPos4 = cam.projectionMatrix.inverse * clipPos;
        viewPos4        /= Mathf.Max(viewPos4.w, 1e-6f);
        Vector3 viewDir  = viewPos4;

        return (cam.cameraToWorldMatrix * new Vector4(viewDir.x, viewDir.y, viewDir.z, 0f));
    }
    #endregion

    #region ✦ 射线–BLAS 相交 ✦
    private bool TraverseBLAS(
        LocalRay ray,
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

            if (node.PrimitiveStartIdx < 0)          // 内部
            {
                stack[++sp] = node.ChildIdx;
                stack[++sp] = node.ChildIdx + 1;
            }
            else                                     // 叶子：遍历三角形
            {
                for (int p = node.PrimitiveStartIdx; p < node.PrimitiveEndIdx; ++p) // ★③
                {
                    int tri = p * 3;
                    Vector3 v0 = vertices[indices[tri    ]];
                    Vector3 v1 = vertices[indices[tri + 1]];
                    Vector3 v2 = vertices[indices[tri + 2]];

                    if (IntersectTriangle(ray, v0, v1, v2, out float tLocal))
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
    
    private static bool IntersectTriangle(
        in LocalRay r, Vector3 v0, Vector3 v1, Vector3 v2, out float t)
    {
        const float EPS = 1e-6f;
        Vector3 e1 = v1 - v0;
        Vector3 e2 = v2 - v0;
        Vector3 p  = Vector3.Cross(r.dir, e2);
        float det  = Vector3.Dot(e1, p);
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

    // slab 法：返回是否相交 & 最近进入距离
    private static bool IntersectAABB(LocalRay r, Vector3 min, Vector3 max, out float tEnter)
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

    #region ✦ 可视化 ✦
    private void DrawRay(in WorldRay ray, bool hit, Vector3 hitPos)
    {
        hitColor.a = 0.3f;
        missColor.a = 0.3f;
        Color c = hit ? hitColor : missColor;
        Gizmos.color = c;

        Vector3 end = hit ? hitPos : ray.origin + ray.dir * rayLength;
        Gizmos.DrawLine(ray.origin, end);
        if (hit) Gizmos.DrawSphere(hitPos, 0.08f);

        Debug.DrawLine(ray.origin, end, c);
    }
    #endregion

    #region ✦ 工具结构体 ✦
    private struct WorldRay
    {
        public Vector3 origin;
        public Vector3 dir;
        public Vector3 invDir;
    }
    private struct LocalRay
    {
        public Vector3 origin;
        public Vector3 dir;
        public Vector3 invDir;
    }
    private static LocalRay WorldToLocalRay(in WorldRay wr, Matrix4x4 worldToLocal)
    {
        Vector3 o = worldToLocal.MultiplyPoint3x4(wr.origin);
        Vector3 d = worldToLocal.MultiplyVector(wr.dir);
        return new LocalRay
        {
            origin  = o,
            dir     = d,
            invDir  = new Vector3(
                1f / (Mathf.Abs(d.x) < 1e-8f ? 1e-8f : d.x),
                1f / (Mathf.Abs(d.y) < 1e-8f ? 1e-8f : d.y),
                1f / (Mathf.Abs(d.z) < 1e-8f ? 1e-8f : d.z))
        };
    }
    #endregion

    private bool EnsureCamera()
    {
        if (cam == null) cam = Camera.main;
        return cam != null;
    }
}
