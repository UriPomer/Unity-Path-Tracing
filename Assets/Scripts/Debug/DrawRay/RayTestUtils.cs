using System.Collections.Generic;
using UnityEngine;

public class RayTestUtils
{
    private static readonly Color missColor = Color.cyan;
    private static readonly Color hitColor  = Color.red;
    private static readonly float rayLength = 10f;
    public struct MyRay { public Vector3 origin, dir, invDir; }
    
    public static void DrawRay(in RayTestUtils.MyRay ray, bool hit, Vector3 hitPoint)
    {
        Color c = hit ? hitColor : missColor;
        c.a = 0.5f;
        Gizmos.color = c;
        Vector3 end = hit ? hitPoint : ray.origin + ray.dir * rayLength;
        Gizmos.DrawLine(ray.origin, end);
        if (hit) Gizmos.DrawSphere(hitPoint, 0.08f);

        Debug.DrawLine(ray.origin, end, c);
    }
    
    public static bool IntersectTriangle(
        in MyRay r, Vector3 v0, Vector3 v1, Vector3 v2, out float t)
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
    
    public static List<MyRay> GenerateWorldRays(Camera cam, int rayCount)
    {
        int w = cam.pixelWidth, h = cam.pixelHeight;
        var list = new List<MyRay>(rayCount);

        for (int i = 0; i < rayCount; ++i)
        {
            Vector2 pixel = new(
                UnityEngine.Random.Range(0, w),
                UnityEngine.Random.Range(0, h));

            Vector3 dir = PixelToWorldDir(cam, pixel, w, h).normalized;
            list.Add(new MyRay
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
    
    private static Vector3 PixelToWorldDir(Camera cam, Vector2 pixel, int w, int h)
    {
        Vector2 screen = (pixel + Vector2.one * 0.5f) / new Vector2(w, h);
        Vector2 ndc    = screen * 2f - Vector2.one;
        Vector4 clip   = new(ndc.x, ndc.y, 1f, 1f);
        Vector4 view4  = cam.projectionMatrix.inverse * clip;
        view4         /= Mathf.Max(view4.w, 1e-6f);
        Vector3 viewDir = view4;
        return (cam.cameraToWorldMatrix * new Vector4(viewDir.x, viewDir.y, viewDir.z, 0f));
    }
    
    public static bool EnsureCamera(Camera cam) { if (cam == null) cam = Camera.main; return cam != null; }
}