using UnityEngine;

/// <summary>
/// 将像素坐标转换成 Compute Shader 中的世界射线方向，
/// 并用 Gizmos + Debug.DrawRay 画出约 20 条箭头
/// </summary>
[ExecuteAlways]             // 运行中 & 编辑器模式都可见
public class RayDirectionGizmo : MonoBehaviour
{
    public Camera cam;       // 如留空则使用 Camera.main
    [Range(1, 100)]
    public int arrowCount = 20;
    public float rayLength = 5f;      // 箭头长度
    public Color gizmoColor = Color.cyan;

    // ---------- Gizmos（Scene 视图） ----------
    private void OnDrawGizmos()
    {
        if (!EnsureCamera()) return;
        Gizmos.color = gizmoColor;
        DrawRays(drawArrowHead: true, drawDebug: false);
    }

    // ---------- Debug.DrawRay（运行时 Game 视图） ----------
    private void Update()
    {
        if (!EnsureCamera()) return;
        DrawRays(drawArrowHead: false, drawDebug: true);
    }

    // 主逻辑：随机采样若干像素坐标 → 转换到世界空间方向 → 画线
    private void DrawRays(bool drawArrowHead, bool drawDebug)
    {
        int width = cam.pixelWidth;
        int height = cam.pixelHeight;
        Random.InitState(12345);      // 固定种子，方便调试

        for (int i = 0; i < arrowCount; ++i)
        {
            Vector2 pixel = new Vector2(
                Random.Range(0, width),
                Random.Range(0, height)
            );

            // 1. 复现 Compute Shader 的坐标变换
            Vector3 worldDir = PixelToWorldDir(pixel, width, height).normalized;
            Vector3 origin = cam.transform.position;
            Vector3 endPos = origin + worldDir * rayLength;

            // 2. 画射线
            if (drawDebug)
                Debug.DrawLine(origin, endPos, gizmoColor);

            if (drawArrowHead)
                DrawArrowHead(endPos, -worldDir, rayLength * 0.08f);
        }
    }

    // ---------- 与 HLSL 相同的像素→世界方向转换 ----------
    private Vector3 PixelToWorldDir(Vector2 pixel, int width, int height)
    {
        // （pixel + 0.5） / (w, h)
        Vector2 screenPos = (pixel + Vector2.one * 0.5f) /
                            new Vector2(width, height);

        // screen → NDC
        Vector2 ndc = screenPos * 2f - Vector2.one;

        // clip space
        Vector4 clipPos = new Vector4(ndc.x, ndc.y, 1f, 1f);

        // 乘逆投影矩阵：clip → view
        Matrix4x4 invProj = cam.projectionMatrix.inverse;
        Vector4 viewPos4 = invProj * clipPos;
        viewPos4 /= Mathf.Max(viewPos4.w, 1e-6f);   // 透视除法
        Vector3 viewDir = viewPos4;                 // w 已归一

        // 乘 CameraToWorld（cam.cameraToWorldMatrix）得到世界方向
        Vector3 worldDir = (cam.cameraToWorldMatrix *
                            new Vector4(viewDir.x, viewDir.y, viewDir.z, 0f));

        return worldDir;
    }

    // ---------- 画箭头尖 ----------
    private void DrawArrowHead(Vector3 tip, Vector3 dir, float size)
    {
        dir.Normalize();
        Vector3 right = Vector3.Cross(dir, Vector3.up);
        if (right.sqrMagnitude < 0.001f)
            right = Vector3.Cross(dir, Vector3.right);
        right.Normalize();
        Vector3 up = Vector3.Cross(dir, right);

        // 4 条小线段形成箭头尖
        Vector3 a = tip + (dir + right + up).normalized * size;
        Vector3 b = tip + (dir + right - up).normalized * size;
        Vector3 c = tip + (dir - right + up).normalized * size;
        Vector3 d = tip + (dir - right - up).normalized * size;

        Gizmos.DrawLine(tip, a);
        Gizmos.DrawLine(tip, b);
        Gizmos.DrawLine(tip, c);
        Gizmos.DrawLine(tip, d);
    }

    // ---------- 保证有相机 ----------
    private bool EnsureCamera()
    {
        if (cam == null) cam = Camera.main;
        return cam != null;
    }
}
