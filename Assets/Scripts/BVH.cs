using System.Collections.Generic;
using UnityEngine;
using System.Linq;


//BLAS 对应于场景中的单个 3D 模型。BLASes 存储实际的顶点数据
//MeshNode 代表每一个Mesh的属性，包括包围盒、Transform索引。 NodeRootIdx是这个Mesh的BLAS节点的起始索引
//TLASNode 有点怪，感觉应该删掉，或者把MeshNode改名为TLASNode
//TODO: 重构这个文件，把MeshNode、TLASNode分开，不要混在一起


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
        int faceInfoStart, int faceInfoEnd
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
        int originPrimitiveCount = indices.Count / 3; // 已有的面片数量
        int originBnodesCount = bnodes.Count; // 已有的BLAS节点数量
        
        foreach (var primitiveIdx in OrderedPrimitiveIndices)
        {
            indices.Add(subindices[primitiveIdx * 3 + 0] + verticesIdxOffset);
            indices.Add(subindices[primitiveIdx * 3 + 1] + verticesIdxOffset);
            indices.Add(subindices[primitiveIdx * 3 + 2] + verticesIdxOffset);
        }
        
        Queue<BVHNode> nodes = new Queue<BVHNode>();
        nodes.Enqueue(BVHRoot); //BVHRoot是调用这个函数的BVH的根节点
        
        while (nodes.Count > 0)
        {
            var node = nodes.Dequeue();
            bnodes.Add(new BLASNode
            {
                BoundMax = node.Bounds.max,
                BoundMin = node.Bounds.min,
                // node.PrimitiveStartIdx >= 0 说明是叶子节点
                PrimitiveStartIdx = node.PrimitiveStartIdx >= 0 ? node.PrimitiveStartIdx + originPrimitiveCount : -1,
                PrimitiveEndIdx = node.PrimitiveStartIdx >= 0 ? node.PrimitiveEndIdx + originPrimitiveCount : -1,
                MaterialIdx = node.PrimitiveStartIdx >= 0 ? materialIdx : 0,
                ChildIdx = node.PrimitiveStartIdx >= 0 ? -1 : nodes.Count + bnodes.Count + 1
            });
            // 注意这里是先插入左节点，再插入右节点，所以在BLAS中，右节点的索引是左节点的索引+1
            if (node.LeftChild != null) nodes.Enqueue(node.LeftChild);
            if (node.RightChild != null) nodes.Enqueue(node.RightChild);
        }
        
        meshNode.Add(new MeshNode
        {
            BoundMax = BVHRoot.Bounds.max,
            BoundMin = BVHRoot.Bounds.min,
            TransformIdx = objectTransformIdx,
            NodeRootIdx = originBnodesCount,
        });
    }

      
    /// <summary>
    /// meshNodes代表每一个Mesh的属性，包括包围盒、Transform索引等，而且是世界坐标系下的属性
    /// BVHRoot是整个场景的BVH根节点，是用meshNodes的信息构建的
    /// 这里通过meshNodes和BVH生成TLASNode
    /// </summary>
    /// <param name="meshNodes"></param>
    /// <param name="tnodes"></param>
    public void FlattenTLAS(ref List<MeshNode> meshNodes, ref List<TLASNode> tnodes)
    {
        List<MeshNode> orderedMeshNodes = new List<MeshNode>();
        foreach (var meshNodeIdx in OrderedPrimitiveIndices) //实际上，在这里OrderedPrimitiveIndices存储的是meshNode(Mesh)的索引，而不是primitive的索引
        {
            orderedMeshNodes.Add(meshNodes[meshNodeIdx]);
        }
        
        meshNodes = orderedMeshNodes;
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
                // MeshNodeEndIdx = node.PrimitiveStartIdx >= 0 ? node.PrimitiveEndIdx : -1,
                ChildIdx = node.PrimitiveStartIdx >= 0 ? -1 : nodes.Count + tnodes.Count + 1
            });
            if (node.LeftChild != null) nodes.Enqueue(node.LeftChild);
            if (node.RightChild != null) nodes.Enqueue(node.RightChild);
        }
    }
    
    // 将顶点和顶点对应的索引转换为PrimitiveInfo，存储AABB和中心点信息
    // 此处生成的PrimitiveInfo的PrimitiveIdx与顶点的索引的对应关系是，PrimitiveIdx = 顶点索引 / 3 取整
    protected List<PrimitiveInfo> CreatePrimitiveInfo(List<Vector3> vertices, List<int> indices)
    {
        List<PrimitiveInfo> infos = new List<PrimitiveInfo>();
        for (int i = 0; i < indices.Count / 3; i++)
        {
            infos.Add(new PrimitiveInfo
            {
                Bounds = new AABB(
                    vertices[indices[i * 3]],
                    vertices[indices[i * 3 + 1]],
                    vertices[indices[i * 3 + 2]]
                ),
                PrimitiveIdx = i
            });
            infos[i].Center = infos[i].Bounds.Center();
        }
        return infos;
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
        List<PrimitiveInfo> infos = new List<PrimitiveInfo>();
        for (int i = 0; i < meshNodes.Count; i++)
        {
            var node = meshNodes[i];
            infos.Add(new PrimitiveInfo
            {
                Bounds = new AABB(
                    // 这里的乘以2是因为每个Transform有两个矩阵，一个是localToWorld，一个是worldToLocal，这里的transform是localToWorld，如果加一才那就是worldToLocal
                    transforms[node.TransformIdx * 2].MultiplyPoint3x4(node.BoundMin),
                    transforms[node.TransformIdx * 2].MultiplyPoint3x4(node.BoundMax)
                ),
                PrimitiveIdx = i
            });
            infos[i].Center = infos[i].Bounds.Center();
        }
        return infos;
    }

    public BVHNode BVHRoot = null;
    public List<int> OrderedPrimitiveIndices = new List<int>();

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
        List<PrimitiveInfo> primitiveInfos,
        int start, int end
    )
    {
        AABB bounding = new AABB();
        //  计算所有面片的包围盒
        for (int i = start; i < end; i++)
        {
            bounding.Extend(primitiveInfos[i].Bounds);
        }

        int primitiveInfoCount = end - start;
        // 如果只有一个面片，直接创建叶子节点
        if (primitiveInfoCount == 1)
        {
            int idx = OrderedPrimitiveIndices.Count;
            int primitiveIdx = primitiveInfos[start].PrimitiveIdx;
            // 从这里可以看出，OrderedPrimitiveIndices中存储的是面片的索引，排序后的索引对应原面片索引
            OrderedPrimitiveIndices.Add(primitiveIdx);
            return BVHNode.CreateLeaf(idx, 1, bounding);
        }

        AABB centerBounding = new AABB(); //所有面片的中心点的包围盒
        for (int i = start; i < end; i++)
        {
            centerBounding.Extend(primitiveInfos[i].Center);
        }

        int dim = centerBounding.MaxDimension();
        int primitiveInfoMid = (start + end) / 2;
        if (Mathf.Approximately(centerBounding.max[dim], centerBounding.min[dim])) //无法在最大维度上划分，则直接创建叶子节点
        {
            int idx = OrderedPrimitiveIndices.Count;
            for (int i = start; i < end; i++)
            {
                int primitiveIdx = primitiveInfos[i].PrimitiveIdx;
                OrderedPrimitiveIndices.Add(primitiveIdx);
            }
            
            return BVHNode.CreateLeaf(idx, primitiveInfoCount, bounding);
        }

        if (primitiveInfoCount <= 2) // 面片数量太少，跳过SAH，直接按照中心点在最大维度上的位置排序
        {
            primitiveInfos.Sort(start, primitiveInfoCount, Comparer<PrimitiveInfo>.Create((x, y) =>
                x.Center[dim].CompareTo(y.Center[dim]) //按照中心点在最大维度上的位置排序
            ));
        }
        else
        {
            List<SAHBucket> buckets = new();
            for (int i = 0; i < nBuckets; i++)
            {
                buckets.Add(new SAHBucket());
            }

            for (int i = start; i < end; i++)
            {
                int b = (int)Mathf.Floor(nBuckets * centerBounding.Offset(primitiveInfos[i].Center)[dim]); //确认该面片属于哪个桶
                b = Mathf.Clamp(b, 0, nBuckets - 1);
                buckets[b].Count++;
                buckets[b].Bounds.Extend(primitiveInfos[i].Bounds);
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
            
            // 如果没有任何划分的cost比当前的叶子节点还要大，则直接创建叶子节点，要不然只是徒增cost
            float leafCost = primitiveInfoCount;
            minCost = 0.5f + minCost / bounding.SurfaceArea();
            
            if (primitiveInfoCount > 16 || minCost < leafCost) //继续划分
            {
                var partition = primitiveInfos.GetRange(start, primitiveInfoCount).ToList().ToLookup(info =>
                {
                    int b = (int)Mathf.Floor(nBuckets * centerBounding.Offset(info.Center)[dim]);
                    b = Mathf.Clamp(b, 0, nBuckets - 1);
                    return b <= minCostSplitBucket;
                });
                var leftInfos = partition[true].ToList();
                var rightInfos = partition[false].ToList();
                primitiveInfoMid = leftInfos.Count + start;
                for (int i = start; i < end; i++)
                {
                    primitiveInfos[i] = i < primitiveInfoMid ? leftInfos[i - start] : rightInfos[i - primitiveInfoMid]; //索引重排
                }
            }
            else  //直接创建叶子节点
            {
                int idx = OrderedPrimitiveIndices.Count;
                for (int i = start; i < end; i++)
                {
                    int primitiveIdx = primitiveInfos[i].PrimitiveIdx;
                    OrderedPrimitiveIndices.Add(primitiveIdx);
                }
                
                // bound是所有面片的包围盒
                // idx是当前叶子节点的索引，primitiveInfoCount是面片数量
                // primitiveInfoCount是怎么和primitiveIdx对应的呢？
                // idx索引对应的叶子节点的第一个面片的索引是idx，最后一个面片的索引是idx+primitiveInfoCount
                // 然后通过这个idx+primitiveInfoCount在OrderedPrimitiveIndices中找到对应的实际面片索引
                return BVHNode.CreateLeaf(idx, primitiveInfoCount, bounding);
            }
        }
        
        if (primitiveInfoMid == start) primitiveInfoMid = (start + end) / 2;
        
        // 递归细分
        var leftChild = Build(primitiveInfos, start, primitiveInfoMid);
        var rightChild = Build(primitiveInfos, primitiveInfoMid, end);
        return BVHNode.CreateParent(dim, leftChild, rightChild);
    }
}
