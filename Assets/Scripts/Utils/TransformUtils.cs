using UnityEngine;

public class TransformUtils
{
    public static void TransformSize(Matrix4x4 LocalToWorld, Vector3 LocalSize, out Vector3 WorldSize)
    {
        Vector3 half = LocalSize * 0.5f;

        Vector3 hx = new Vector3(half.x, 0f,     0f);
        Vector3 hy = new Vector3(0f,     half.y, 0f);
        Vector3 hz = new Vector3(0f,     0f,     half.z);

        Vector3 wx = LocalToWorld.MultiplyVector(hx);
        Vector3 wy = LocalToWorld.MultiplyVector(hy);
        Vector3 wz = LocalToWorld.MultiplyVector(hz);

        Vector3 worldHalf = new Vector3(
            Mathf.Abs(wx.x) + Mathf.Abs(wy.x) + Mathf.Abs(wz.x),
            Mathf.Abs(wx.y) + Mathf.Abs(wy.y) + Mathf.Abs(wz.y),
            Mathf.Abs(wx.z) + Mathf.Abs(wy.z) + Mathf.Abs(wz.z)
            
        );

        WorldSize = worldHalf * 2f;
    }
    
    public static void TransformBounds(
        Matrix4x4 localToWorld,
        Vector3 localMin,
        Vector3 localMax,
        out Vector3 worldMin,
        out Vector3 worldMax)
    {
        Vector3 localCenter = (localMin + localMax) * 0.5f;
        Vector3 half        = (localMax - localMin) * 0.5f;

        Vector3 hx = new Vector3(half.x, 0f,     0f);
        Vector3 hy = new Vector3(0f,     half.y, 0f);
        Vector3 hz = new Vector3(0f,     0f,     half.z);

        Vector3 wCenter = localToWorld.MultiplyPoint3x4(localCenter);

        Vector3 wx = localToWorld.MultiplyVector(hx);
        Vector3 wy = localToWorld.MultiplyVector(hy);
        Vector3 wz = localToWorld.MultiplyVector(hz);

        Vector3 worldHalf = new Vector3(
            Mathf.Abs(wx.x) + Mathf.Abs(wy.x) + Mathf.Abs(wz.x),
            Mathf.Abs(wx.y) + Mathf.Abs(wy.y) + Mathf.Abs(wz.y),
            Mathf.Abs(wx.z) + Mathf.Abs(wy.z) + Mathf.Abs(wz.z)
        );

        worldMin = wCenter - worldHalf;
        worldMax = wCenter + worldHalf;
    }
    
    public static void TransformBoundingBox(
        Matrix4x4 localToWorld, 
        Bounds localBounds, 
        out Bounds worldBounds)
    {
        Vector3[] corners = new Vector3[8];
        corners[0] = localBounds.min; // 左下前
        corners[1] = new Vector3(localBounds.min.x, localBounds.min.y, localBounds.max.z); // 左下后
        corners[2] = new Vector3(localBounds.min.x, localBounds.max.y, localBounds.min.z); // 左上前
        corners[3] = new Vector3(localBounds.min.x, localBounds.max.y, localBounds.max.z); // 左上后
        corners[4] = new Vector3(localBounds.max.x, localBounds.min.y, localBounds.min.z); // 右下前
        corners[5] = new Vector3(localBounds.max.x, localBounds.min.y, localBounds.max.z); // 右下后
        corners[6] = new Vector3(localBounds.max.x, localBounds.max.y, localBounds.min.z); // 右上前
        corners[7] = localBounds.max; // 右上后

        for (int i = 0; i < 8; i++)
        {
            corners[i] = localToWorld.MultiplyPoint3x4(corners[i]);
        }

        Vector3 newMin = corners[0];
        Vector3 newMax = corners[0];

        for (int i = 1; i < 8; i++)
        {
            newMin = Vector3.Min(newMin, corners[i]);
            newMax = Vector3.Max(newMax, corners[i]);
        }

        worldBounds = new Bounds((newMin + newMax) * 0.5f, newMax - newMin);
    }
}
