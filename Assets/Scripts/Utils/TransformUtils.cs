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
}
