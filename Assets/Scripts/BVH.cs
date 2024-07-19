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

    public static int TypeSize = sizeof(float) * 3 * 2 + sizeof(int) * 4;   //40 bytes
}

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
    // public int NodeEndIdx;

    public static int TypeSize = sizeof(float)*3*2+sizeof(int)*2;   //32 bytes
}

/// <summary>
/// TLAS node built with bvh
/// </summary>
public struct TLASNode
{
    public Vector3 BoundMax;
    public Vector3 BoundMin;
    public int MeshNodeStartIdx;
    // public int MeshNodeEndIdx;
    public int ChildIdx;

    public static int TypeSize = sizeof(float) * 3 * 2 + sizeof(int) * 2;   //32 bytes
}
/// <summary>
/// Bounding box
/// </summary>
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
    public class PrimitiveInfo
    {
        public AABB Bounds;
        public Vector3 Center;
        public int PrimitiveIdx;
    }


    protected abstract BVHNode Build(
        List<PrimitiveInfo> faceInfo,
        int faceInfoStart, int faceInfoEnd, int depth = 0
    );

    /// <summary>
    /// 将子网格的BVH节点转换为BLAS节点，并存到全局的BLAS节点列表和indices列表中
    /// </summary>
    /// <param name="indices"></param>
    /// <param name="bnodes"></param>
    /// <param name="meshNode"></param>
    /// <param name="subindices"></param>
    /// <param name="verticesIdxOffset"></param>
    /// <param name="materialIdx"></param>
    /// <param name="objectTransformIdx"></param>
    public void FlattenBLAS(
        ref List<int> indices, ref List<BLASNode> bnodes,
        ref List<MeshNode> meshNode, List<int> subindices,
        int verticesIdxOffset, int materialIdx, int objectTransformIdx
    )
    {
        int faceOffset = indices.Count / 3;  //面的数量
        int bnodesOffset = bnodes.Count;
        // add indices
        foreach (int faceId in OrderedFaceId)
        {
            indices.Add(subindices[faceId * 3] + verticesIdxOffset);
            indices.Add(subindices[faceId * 3 + 1] + verticesIdxOffset);
            indices.Add(subindices[faceId * 3 + 2] + verticesIdxOffset);
        }
        // add BLAS nodes
        Queue<BVHNode> nodes = new Queue<BVHNode>();
        nodes.Enqueue(BVHRoot);
        while (nodes.Count > 0)
        {
            var node = nodes.Dequeue();
            bnodes.Add(new BLASNode
            {
                BoundMax = node.Bounds.max,
                BoundMin = node.Bounds.min,
                PrimitiveStartIdx = node.PrimitiveStartIdx >= 0 ? node.PrimitiveStartIdx + faceOffset : -1,
                PrimitiveEndIdx = node.PrimitiveStartIdx >= 0 ? node.PrimitiveEndIdx + faceOffset : -1,
                MaterialIdx = node.PrimitiveStartIdx >= 0 ? materialIdx : 0,
                ChildIdx = node.PrimitiveStartIdx >= 0 ? -1 : nodes.Count + bnodes.Count + 1
            });
            if (node.LeftChild != null)
                nodes.Enqueue(node.LeftChild);
            if (node.RightChild != null)
                nodes.Enqueue(node.RightChild);
        }
        // add raw TLAS node
        meshNode.Add(new MeshNode()
        {
            BoundMax = BVHRoot.Bounds.max,
            BoundMin = BVHRoot.Bounds.min,
            NodeRootIdx = bnodesOffset,
            TransformIdx = objectTransformIdx
        });
    }

      
    /// <summary>
    /// rawNodes代表每一个Mesh的属性，包括包围盒、Transform索引等，而且是世界坐标系下的属性
    /// BVHRoot是整个场景的BVH根节点，是用rawNodes的信息构建的
    /// 这里通过rawNodes和BVH生成TLASNode
    /// </summary>
    /// <param name="meshNodes"></param>
    /// <param name="tnodes"></param>
    public void FlattenTLAS(ref List<MeshNode> meshNodes, ref List<TLASNode> tnodes)
    {
        // reorder raw nodes
        List<MeshNode> newRawNodes = new List<MeshNode>();
        foreach(int rawNodeId in OrderedFaceId)
        {
            newRawNodes.Add(meshNodes[rawNodeId]);
        }
        meshNodes = newRawNodes;
        // add TLAS nodes
        Queue<BVHNode> nodes = new Queue<BVHNode>();
        nodes.Enqueue(BVHRoot);
        while (nodes.Count > 0)
        {
            var node = nodes.Dequeue();
            tnodes.Add(new TLASNode
            {
                BoundMax = node.Bounds.max,
                BoundMin = node.Bounds.min,
                MeshNodeStartIdx = node.PrimitiveStartIdx >= 0 ? node.PrimitiveStartIdx : -1,
                // Mesh = node.FaceStart >= 0 ? node.FaceEnd : -1,
                ChildIdx = node.PrimitiveStartIdx >= 0 ? -1 : nodes.Count + tnodes.Count + 1
            });
            if (node.LeftChild != null)
                nodes.Enqueue(node.LeftChild);
            if (node.RightChild != null)
                nodes.Enqueue(node.RightChild);
        }
    }
    
    // 将顶点和顶点对应的索引转换为PrimitiveInfo，存储AABB和中心点信息
    // 此处生成的PrimitiveInfo的PrimitiveIdx与顶点的索引的对应关系是，PrimitiveIdx = 顶点索引 / 3 取整
    protected List<PrimitiveInfo> CreatePrimitiveInfo(List<Vector3> vertices, List<int> indices)
    {
        List<PrimitiveInfo> info = new List<PrimitiveInfo>();
        for (int i = 0; i < indices.Count / 3; i++)
        {
            info.Add(new PrimitiveInfo
            {
                Bounds = new AABB(
                    vertices[indices[i * 3]],
                    vertices[indices[i * 3 + 1]],
                    vertices[indices[i * 3 + 2]]
                ),
                PrimitiveIdx = i
            });
            info[i].Center = info[i].Bounds.Center();
        }
        return info;
    }

    
    /// <summary>
    /// 通过TLASRawNode和Transforms生成PrimitiveInfo
    /// 这里的rawNodes，通常情况下，场景中有几个物体，就有几个rawNodes，但如何一个mesh有多个submesh，那么这个mesh就会有多个rawNodes
    /// </summary>
    /// <param name="meshNodes"></param>
    /// <param name="transforms"></param>
    /// <returns></returns>
    protected List<PrimitiveInfo> CreatePrimitiveInfo(List<MeshNode> meshNodes, List<Matrix4x4> transforms)
    {
        //这个函数完全可以改名，它只是把rawNode的包围盒从local space转换到world space，并且计算了变换后的包围盒的中心点
        List<PrimitiveInfo> info = new List<PrimitiveInfo>();
        for (int i = 0; i < meshNodes.Count; i++)
        {
            var node = meshNodes[i];
            info.Add(new PrimitiveInfo
            {
                Bounds = new AABB(
                    // 这里的乘以2是因为每个Transform有两个矩阵，一个是localToWorld，一个是worldToLocal，这里的transform是localToWorld，如果加一才那就是worldToLocal
                    transforms[node.TransformIdx * 2].MultiplyPoint3x4(node.BoundMin),
                    transforms[node.TransformIdx * 2].MultiplyPoint3x4(node.BoundMax)
                ),
                PrimitiveIdx = i
            });
            info[i].Center = info[i].Bounds.Center();
        }
        return info;
    }

    public BVHNode BVHRoot = null;
    public List<int> OrderedFaceId = new List<int>();

    public static BVH Construct(List<Vector3> vertices, List<int> indices, BVHType type)
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
    private readonly int nBuckets = 12;

    /// <summary>
    /// Info for SAH
    /// </summary>
    public class SAHBucket
    {
        public int Count = 0;
        public AABB Bounds = new AABB();
    }

    public BVHSAH(List<Vector3> vertices, List<int> indices)
    {
        // generate face info
        var faceInfo = CreatePrimitiveInfo(vertices, indices);
        // build tree
        BVHRoot = Build(faceInfo, 0, faceInfo.Count);
    }

    public BVHSAH(List<MeshNode> rawNodes, List<Matrix4x4> transforms)
    {
        // generate face info
        var faceInfo = CreatePrimitiveInfo(rawNodes, transforms);
        // build tree
        BVHRoot = Build(faceInfo, 0, faceInfo.Count);
    }

    protected override BVHNode Build(
        List<PrimitiveInfo> faceInfo,
        int faceInfoStart, int faceInfoEnd,
        int depth = 0
    )
    {
        // get vertices bounding
        AABB bounding = new AABB();
        for (int i = faceInfoStart; i < faceInfoEnd; i++)
            bounding.Extend(faceInfo[i].Bounds);
        int faceInfoCount = faceInfoEnd - faceInfoStart;
        // if only one face, create a leaf
        if (faceInfoCount == 1)
        {
            int idx = OrderedFaceId.Count;
            int faceIdx = faceInfo[faceInfoStart].PrimitiveIdx;
            OrderedFaceId.Add(faceIdx);
            return BVHNode.CreateLeaf(idx, faceInfoCount, bounding);
        }

        // get centroids bounding
        AABB centerBounding = new AABB();
        for (int i = faceInfoStart; i < faceInfoEnd; i++)
            centerBounding.Extend(faceInfo[i].Center);
        int dim = centerBounding.MaxDimension();
        int faceInfoMid = (faceInfoStart + faceInfoEnd) / 2;
        // if cannot further split on this axis, generate a leaf
        if (centerBounding.max[dim] == centerBounding.min[dim])
        {
            int idx = OrderedFaceId.Count;
            for (int i = faceInfoStart; i < faceInfoEnd; i++)
            {
                int faceIdx = faceInfo[i].PrimitiveIdx;
                OrderedFaceId.Add(faceIdx);
            }
            return BVHNode.CreateLeaf(idx, faceInfoCount, bounding);
        }

        if (faceInfoCount <= 2)
        {
            // if only 2 faces remain, skip SAH
            faceInfo.Sort(
                faceInfoStart, faceInfoCount,
                Comparer<PrimitiveInfo>.Create((x, y) => x.Center[dim].CompareTo(y.Center[dim]))
            );
        }
        else
        {
            List<SAHBucket> buckets = new();
            for (int i = 0; i < nBuckets; i++)
            {
                buckets.Add(new SAHBucket());
            }

            for (int i = faceInfoStart; i < faceInfoEnd; i++)
            {
                int b = (int)Mathf.Floor(nBuckets * centerBounding.Offset(faceInfo[i].Center)[dim]); //确认该面片属于哪个桶
                b = Mathf.Clamp(b, 0, nBuckets - 1);
                buckets[b].Count++;
                buckets[b].Bounds.Extend(faceInfo[i].Bounds);
            }

            //处理桶的cost
            List<int> countLeft = new();
            int[] countRight = new int[nBuckets - 1];
            List<float> areaLeft = new();
            float[] areaRight = new float[nBuckets - 1];


            int leftSum = 0;
            int rightSum = 0;
            AABB leftBox = new();
            AABB rightBox = new();
            for (int i = 0; i < nBuckets - 1; i++)    // 12个桶，只有11个划分点
            {
                leftSum += buckets[i].Count;
                countLeft.Add(leftSum);
                leftBox.Extend(buckets[i].Bounds);
                areaLeft.Add(leftBox.SurfaceArea());
                
                rightSum += buckets[nBuckets - 1 - i].Count;
                countRight[nBuckets - 2 - i] = rightSum;
                rightBox.Extend(buckets[nBuckets - 1 - i].Bounds);
                areaRight[nBuckets - 2 - i] = rightBox.SurfaceArea();
            }

            //计算cost
            float minCost = float.MaxValue;
            int minCostSplitBucket = -1;
            for (int i = 0; i < nBuckets - 1; i++)
            {
                if (countLeft[i] == 0 || countRight[i] == 0) continue;
                float cost = countLeft[i] * areaLeft[i] + countRight[i] * areaRight[i];
                if (cost < minCost)
                {
                    minCost = cost;
                    minCostSplitBucket = i;
                }
            }
            //// create leaf or split primitives at selected bucket
            float leafCost = faceInfoCount;
            minCost = 0.5f + minCost / bounding.SurfaceArea();
            if (faceInfoCount > 16 || minCost < leafCost)
            {
                var partition = faceInfo.GetRange(faceInfoStart, faceInfoCount).ToList().ToLookup(info =>
                {
                    int b = (int)Mathf.Floor(nBuckets * centerBounding.Offset(info.Center)[dim]);
                    b = Mathf.Clamp(b, 0, nBuckets - 1);
                    return b <= minCostSplitBucket;
                });
                var pLeft = partition[true].ToList();
                var pRight = partition[false].ToList();
                faceInfoMid = pLeft.Count + faceInfoStart;
                for (int i = faceInfoStart; i < faceInfoEnd; i++)
                    faceInfo[i] = (i < faceInfoMid) ? pLeft[i - faceInfoStart] : pRight[i - faceInfoMid];
            }
            else
            {
                int idx = OrderedFaceId.Count;
                for (int i = faceInfoStart; i < faceInfoEnd; i++)
                {
                    int faceIdx = faceInfo[i].PrimitiveIdx;
                    OrderedFaceId.Add(faceIdx);
                }
                return BVHNode.CreateLeaf(idx, faceInfoCount, bounding);
            }
        }
        // avoid middle index error
        if (faceInfoMid == faceInfoStart) faceInfoMid = (faceInfoStart + faceInfoEnd) / 2;
        // recursively build left and right nodes
        var leftChild = Build(faceInfo, faceInfoStart, faceInfoMid, depth + 1);
        var rightChild = Build(faceInfo, faceInfoMid, faceInfoEnd, depth + 1);
        return BVHNode.CreateParent(
            dim,
            leftChild,
            rightChild
        );
    }
}
