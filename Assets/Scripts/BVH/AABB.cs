
using UnityEngine;

public class AABB
{
    public Vector3 min;
    public Vector3 max;
    public Vector3 extent;

    public AABB()
    {
        min = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
        max = new Vector3(float.MinValue, float.MinValue, float.MinValue);
        extent = max - min;
    }

    public AABB(Vector3 a, Vector3 b)
    {
        min = new Vector3(
            a.x < b.x ? a.x : b.x,
            a.y < b.y ? a.y : b.y,
            a.z < b.z ? a.z : b.z
        );
        max = new Vector3(
            a.x > b.x ? a.x : b.x,
            a.y > b.y ? a.y : b.y,
            a.z > b.z ? a.z : b.z
        );
        extent = max - min;
    }

    public AABB(Vector3 v0, Vector3 v1, Vector3 v2)
    {
        float minX = v0.x < v1.x ? v0.x : v1.x;
        float minY = v0.y < v1.y ? v0.y : v1.y;
        float minZ = v0.z < v1.z ? v0.z : v1.z;
        min = new Vector3(
            minX < v2.x ? minX : v2.x,
            minY < v2.y ? minY : v2.y,
            minZ < v2.z ? minZ : v2.z
        );

        float maxX = v0.x > v1.x ? v0.x : v1.x;
        float maxY = v0.y > v1.y ? v0.y : v1.y;
        float maxZ = v0.z > v1.z ? v0.z : v1.z;
        max = new Vector3(
            maxX > v2.x ? maxX : v2.x,
            maxY > v2.y ? maxY : v2.y,
            maxZ > v2.z ? maxZ : v2.z
        );

        extent = max - min;
    }
    
    public void Reset()
    {
        min = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
        max = new Vector3(float.MinValue, float.MinValue, float.MinValue);
        extent = Vector3.zero;
    }

    public void Extend(AABB volume)
    {
        min.x = volume.min.x < min.x ? volume.min.x : min.x;
        min.y = volume.min.y < min.y ? volume.min.y : min.y;
        min.z = volume.min.z < min.z ? volume.min.z : min.z;

        max.x = volume.max.x > max.x ? volume.max.x : max.x;
        max.y = volume.max.y > max.y ? volume.max.y : max.y;
        max.z = volume.max.z > max.z ? volume.max.z : max.z;

        extent = max - min;
    }

    public void Extend(Vector3 p)
    {
        min.x = p.x < min.x ? p.x : min.x;
        min.y = p.y < min.y ? p.y : min.y;
        min.z = p.z < min.z ? p.z : min.z;

        max.x = p.x > max.x ? p.x : max.x;
        max.y = p.y > max.y ? p.y : max.y;
        max.z = p.z > max.z ? p.z : max.z;

        extent = max - min;
    }

    public Vector3 Center()
    {
        return (min + max) * 0.5f;
    }

    public int MaxDimension()
    {
        int result = 0; // 0 for x, 1 for y, 2 for z
        if(extent.y > extent[result]) result = 1;
        if(extent.z > extent[result]) result = 2;
        return result;
    }

    public static AABB Combine(AABB v1, AABB v2)
    {
        AABB result = v1.Copy();
        result.Extend(v2);
        return result;
    }

    public Vector3 Offset(Vector3 p)
    {
        Vector3 o = p - min;
        if (max.x > min.x) o.x /= extent.x;
        if (max.y > min.y) o.y /= extent.y;
        if (max.z > min.z) o.z /= extent.z;
        return o;
    }

    public float SurfaceArea()
    {
        return 2.0f * (
            extent.x * extent.y +
            extent.x * extent.z +
            extent.y * extent.z
        );
    }

    public AABB Copy()
    {
        return new AABB(min, max);
    }
}
