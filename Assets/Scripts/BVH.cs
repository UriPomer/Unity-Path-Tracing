using System;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;


public struct BLASNode
{
    public Vector3 BoundMax;
    public Vector3 BoundMin;
    public int PrimitiveStartIdx;
    public int PrimitiveEndIdx;
    public int MaterialIdx;
    public int ChildIdx;

    public static int TypeSize = sizeof(float) * 3 * 2 + sizeof(int) * 4; }

/// <summary>
/// 含有SubMesh的包围盒、Transform索引
/// NodeRootIdx是这个Mesh的BLAS节点的起始索引
/// </summary>
public struct MeshNode
{
    public Vector3 BoundMax;
    public Vector3 BoundMin;
    public int TransformIdx;    // also the index of the object
    public int NodeRootIdx;
    public int  ChildIdx;

    public static int TypeSize = sizeof(float)*3*2+sizeof(int)*3;
}

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

    public AABB(Vector3 min, Vector3 max)
    {
        this.min = Vector3.Min(min, max);
        this.max = Vector3.Max(min, max);
        extent = this.max - this.min;
    }

    public AABB(Vector3 v0, Vector3 v1, Vector3 v2)
    {
        min = Vector3.Min(v0, Vector3.Min(v1, v2));
        max = Vector3.Max(v0, Vector3.Max(v1, v2));
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
        min = Vector3.Min(volume.min, min);
        max = Vector3.Max(volume.max, max);
        extent = max - min;
    }

    public void Extend(Vector3 p)
    {
        min = Vector3.Min(p, min);
        max = Vector3.Max(p, max);
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

/// <summary>
/// define BVH Type
/// </summary>
public enum BVHType
{
    SAH
}

/// <summary>
/// Abstract BVH class
/// </summary>
public abstract class BVH
{
    /// <summary>
    /// BVH tree node
    /// </summary>
    public class BVHNode
    {
        public AABB Bounds;
        public BVHNode LeftChild;
        public BVHNode RightChild;
        public int SplitAxis;
        public int PrimitiveStartIdx;
        public int PrimitiveEndIdx;

        public bool IsLeaf()
        {
            return (LeftChild == null) && (RightChild == null);
        }

        public static BVHNode CreateLeaf(int start, int count, AABB bounding)
        {
            BVHNode node = new BVHNode
            {
                Bounds = bounding,
                LeftChild = null,
                RightChild = null,
                SplitAxis = -1,
                PrimitiveStartIdx = start,
                PrimitiveEndIdx = start + count
            };
            return node;
        }

        public static BVHNode CreateParent(int splitAxis, BVHNode nodeLeft, BVHNode nodeRight)
        {
            BVHNode node = new BVHNode
            {
                Bounds = AABB.Combine(nodeLeft.Bounds, nodeRight.Bounds),
                LeftChild = nodeLeft,
                RightChild = nodeRight,
                SplitAxis = splitAxis,
                PrimitiveStartIdx = -1,
                PrimitiveEndIdx = -1
            };
            return node;
        }
    }

    // 只有AABB和中心点信息，而没有顶点信息
    protected struct PrimitiveInfo
    {
        public AABB Bounds;
        public Vector3 Center;
        public int PrimitiveIdx;
    }


    protected abstract BVHNode Build(
        List<PrimitiveInfo> faceInfo,
        int faceInfoStart, int faceInfoEnd
    );

    /// <summary>
    /// 将子网格的BVH节点转换为BLAS节点，并存到全局的BLAS节点列表和indices列表中
    /// </summary>
    public void FlattenBLAS(
        ref List<BLASNode> bnodes,
        ref List<MeshNode> meshNodes,
        int materialIdx,
        int objectTransformIdx,
        int globalPrimitiveBase
    )
    {
        int blasRootIdx = bnodes.Count;

        Queue<BVHNode> q = new Queue<BVHNode>();
        q.Enqueue(BVHRoot);
        while (q.Count > 0)
        {
            var n = q.Dequeue();
            bnodes.Add(new BLASNode
            {
                BoundMax         = n.Bounds.max,
                BoundMin         = n.Bounds.min,
                PrimitiveStartIdx= n.IsLeaf() ? n.PrimitiveStartIdx + globalPrimitiveBase : -1,
                PrimitiveEndIdx  = n.IsLeaf() ? n.PrimitiveEndIdx   + globalPrimitiveBase : -1,
                MaterialIdx      = n.IsLeaf() ? materialIdx : 0,
                ChildIdx         = n.IsLeaf() ? -1 : q.Count + bnodes.Count + 1
            });
            if (n.LeftChild  != null) q.Enqueue(n.LeftChild);
            if (n.RightChild != null) q.Enqueue(n.RightChild);
        }

        meshNodes.Add(new MeshNode
        {
            BoundMax     = BVHRoot.Bounds.max,
            BoundMin     = BVHRoot.Bounds.min,
            TransformIdx = objectTransformIdx,
            NodeRootIdx  = blasRootIdx
        });
        
        OrderedPrimitiveIndices.Clear();
    }
    
    // TODO : FlattenTLAS和 FlatBLAS好像干了一样的事情

    public void FlattenTLAS(
        ref List<MeshNode> dst,
        List<MeshNode> leafSrc,              // 原 meshNodes（局部包围盒）
        IReadOnlyList<Matrix4x4> trs)        // 对应 transform
    {
        dst.Clear();
        Queue<(BVHNode node, int parent, bool leftFlag)> q = new();
        q.Enqueue((BVHRoot, -1, true));          // 根

        while (q.Count > 0)
        {
            var (cur, parent, left) = q.Dequeue();
            int myIdx = dst.Count;

            MeshNode n;
            if (cur.IsLeaf())                    // ---------- 叶子 ----------
            {
                MeshNode src = leafSrc[cur.PrimitiveStartIdx];
                Matrix4x4 l2w = trs[src.TransformIdx];

                // 转到世界空间
                Vector3 wMin = l2w.MultiplyPoint3x4(src.BoundMin);
                Vector3 wMax = l2w.MultiplyPoint3x4(src.BoundMax);

                n = new MeshNode {
                    BoundMin     = wMin,
                    BoundMax     = wMax,
                    TransformIdx = src.TransformIdx,
                    NodeRootIdx  = src.NodeRootIdx,
                    ChildIdx     = -1
                };
            }
            else                                 // ---------- 内部 ----------
            {
                n = new MeshNode {
                    BoundMin     = cur.Bounds.min,
                    BoundMax     = cur.Bounds.max,
                    TransformIdx = -1,
                    NodeRootIdx  = -1,
                    ChildIdx     = -2          // 占位
                };
            }

            dst.Add(n);

            // 回填父节点 childIdx
            if (parent >= 0) {
                MeshNode p = dst[parent];
                if (left) p.ChildIdx = myIdx;    // 左孩子
                dst[parent] = p;
            }

            if (!cur.IsLeaf()) {
                q.Enqueue((cur.LeftChild , myIdx, true ));
                q.Enqueue((cur.RightChild, myIdx, false));
            }
        }
    }

    
    private static List<PrimitiveInfo> sharedInfos = new List<PrimitiveInfo>();
    // 将顶点和顶点对应的索引转换为PrimitiveInfo，存储AABB和中心点信息
    // 此处生成的PrimitiveInfo的PrimitiveIdx与顶点的索引的对应关系是，PrimitiveIdx = 顶点索引 / 3 取整
    protected List<PrimitiveInfo> CreatePrimitiveInfo(Vector3[] vertices, int[] indices)
    {
        sharedInfos.Clear();
        int triCount = indices.Length / 3;
        // 预分配容量，减少扩容
        if (sharedInfos.Capacity < triCount) sharedInfos.Capacity = triCount;

        for (int i = 0; i < triCount; i++)
        {
            int i0 = indices[i * 3], i1 = indices[i * 3 + 1], i2 = indices[i * 3 + 2];
            var bounds = new AABB(vertices[i0], vertices[i1], vertices[i2]);
            sharedInfos.Add(new PrimitiveInfo
            {
                Bounds = bounds,
                Center = bounds.Center(),
                PrimitiveIdx = i
            });
        }
        return sharedInfos;
    }

    
    /// <summary>
    /// 通过TLASRawNode和Transforms生成PrimitiveInfo
    /// 这里的rawNodes，通常情况下，场景中有几个物体，就有几个rawNodes，但如何一个mesh有多个submesh，那么这个mesh就会有多个rawNodes
    /// </summary>
    /// <param name="meshNodes"></param>
    /// <param name="transforms"></param>
    /// <returns></returns>
    protected List<PrimitiveInfo> CreatePrimitiveInfo
        (List<MeshNode> meshNodes, List<Matrix4x4> transforms)
    {
        sharedInfos.Clear();
        if (sharedInfos.Capacity < meshNodes.Count)
            sharedInfos.Capacity = meshNodes.Count;

        for (int i = 0; i < meshNodes.Count; i++)
        {
            var n = meshNodes[i];
            PrimitiveInfo info;
            info.Bounds = new AABB
            (
                transforms[n.TransformIdx].MultiplyPoint3x4(n.BoundMin),
                transforms[n.TransformIdx].MultiplyPoint3x4(n.BoundMax)
            );
            info.Center       = info.Bounds.Center();
            info.PrimitiveIdx = i;
            sharedInfos.Add(info);
        }
        return sharedInfos;
    }

    public BVHNode BVHRoot = null;

    protected static readonly List<int> s_SharedOrderedPrimitiveIndices = new List<int>(4096);
    public List<int> OrderedPrimitiveIndices { get; protected set; }

    public static BVH Construct(Vector3[] vertices, int[] indices, BVHType type)
    {
                return new BVHSAH(vertices, indices);
    }

    public static BVH Construct(List<MeshNode> rawNodes, List<Matrix4x4> transforms, BVHType type)
    {
                return new BVHSAH(rawNodes, transforms);
    }
}

/// <summary>
/// BVH with SAH
/// refer to https://www.pbr-book.org/3ed-2018/Primitives_and_Intersection_Acceleration/Bounding_Volume_Hierarchies
/// </summary>
public class BVHSAH : BVH
{
    private static readonly int nBuckets = 12;
    
    /// <summary>
    /// Info for SAH
    /// </summary>
    public struct SAHBucket
    {
        public int  Count;
        public AABB Bounds;

        [System.Runtime.CompilerServices.MethodImpl(
            System.Runtime.CompilerServices.MethodImplOptions.AggressiveInlining)]
        public void Reset()
        {
            Count = 0;

            if (Bounds == null)
                Bounds = new AABB();
            else
                Bounds.Reset();
        }
    }
    
    private static readonly SAHBucket[] bucketsCache = new SAHBucket[nBuckets];

    public BVHSAH(Vector3[] vertices,int[] indices)
    {
        OrderedPrimitiveIndices = s_SharedOrderedPrimitiveIndices;
        OrderedPrimitiveIndices.Clear();
        // generate face info
        var faceInfo = CreatePrimitiveInfo(vertices, indices);
        // build tree
        BVHRoot = Build(faceInfo, 0, faceInfo.Count);
    }

    public BVHSAH(List<MeshNode> rawNodes, List<Matrix4x4> transforms)
    {
        OrderedPrimitiveIndices = s_SharedOrderedPrimitiveIndices;
        OrderedPrimitiveIndices.Clear();
        // generate face info
        var faceInfo = CreatePrimitiveInfo(rawNodes, transforms);
        // build tree
        BVHRoot = Build(faceInfo, 0, faceInfo.Count);
    }

    protected override BVHNode Build(List<PrimitiveInfo> infos, int start, int end)
    {
        AABB bounds = new AABB();
        for (int i = start; i < end; i++) bounds.Extend(infos[i].Bounds);

        int count = end - start;
        if (count == 1)
        {
            int dst = OrderedPrimitiveIndices.Count;
            OrderedPrimitiveIndices.Add(infos[start].PrimitiveIdx);
            return BVHNode.CreateLeaf(dst, 1, bounds);
        }

        AABB centerBounds = new AABB();
        for (int i = start; i < end; i++) centerBounds.Extend(infos[i].Center);
        int axis = centerBounds.MaxDimension();

        if (Mathf.Approximately(centerBounds.extent[axis], 0f))
        {
            int dst = OrderedPrimitiveIndices.Count;
            for (int i = start; i < end; i++)
                OrderedPrimitiveIndices.Add(infos[i].PrimitiveIdx);
            return BVHNode.CreateLeaf(dst, count, bounds);
        }

        for (int i = 0; i < nBuckets; i++) bucketsCache[i].Reset();

        float invExtent = 1f / centerBounds.extent[axis];

        for (int i = start; i < end; i++)
        {
            int b = (int)((infos[i].Center[axis] - centerBounds.min[axis]) * invExtent * nBuckets);
            b = Mathf.Clamp(b, 0, nBuckets - 1);

            ref SAHBucket bucket = ref bucketsCache[b];
            bucket.Count++;
            bucket.Bounds.Extend(infos[i].Bounds);
        }

        Span<float> areaL = stackalloc float[nBuckets - 1];
        Span<float> areaR = stackalloc float[nBuckets - 1];
        Span<int>   cntL  = stackalloc int  [nBuckets - 1];
        Span<int>   cntR  = stackalloc int  [nBuckets - 1];

        AABB tmp = new AABB();
        int acc = 0;
        for (int i = 0; i < nBuckets - 1; i++)
        {
            acc += bucketsCache[i].Count;
            cntL[i] = acc;
            tmp.Extend(bucketsCache[i].Bounds);
            areaL[i] = tmp.SurfaceArea();
        }
        tmp.Reset(); acc = 0;
        for (int i = nBuckets - 1; --i >= 0;)
        {
            acc += bucketsCache[i + 1].Count;
            cntR[i] = acc;
            tmp.Extend(bucketsCache[i + 1].Bounds);
            areaR[i] = tmp.SurfaceArea();
        }

        float bestCost = float.MaxValue; int bestSplit = -1;
        float invSA = 1f / bounds.SurfaceArea();
        for (int i = 0; i < nBuckets - 1; i++)
        {
            if (cntL[i] == 0 || cntR[i] == 0) continue;
            float cost = 0.5f + (cntL[i] * areaL[i] + cntR[i] * areaR[i]) * invSA;
            if (cost < bestCost) { bestCost = cost; bestSplit = i; }
        }

        if (bestCost >= count)
        {
            int dst = OrderedPrimitiveIndices.Count;
            for (int i = start; i < end; i++)
                OrderedPrimitiveIndices.Add(infos[i].PrimitiveIdx);
            return BVHNode.CreateLeaf(dst, count, bounds);
        }

        infos.Sort(start, count, Comparer<PrimitiveInfo>.Create((a, b) =>
        {
            int ba = Mathf.Clamp(
                (int)((a.Center[axis] - centerBounds.min[axis]) * invExtent * nBuckets), 0, nBuckets - 1);
            int bb = Mathf.Clamp(
                (int)((b.Center[axis] - centerBounds.min[axis]) * invExtent * nBuckets), 0, nBuckets - 1);
            return ba.CompareTo(bb);
        }));

        int mid = start;
        for (int i = 0; i <= bestSplit; i++) mid += bucketsCache[i].Count;
        if (mid == start || mid == end) mid = (start + end) >> 1;   // 防御

        var left  = Build(infos, start, mid);
        var right = Build(infos, mid,   end);
        return BVHNode.CreateParent(axis, left, right);
    }

}
