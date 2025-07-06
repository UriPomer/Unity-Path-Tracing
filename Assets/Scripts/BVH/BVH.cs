using System.Collections.Generic;
using UnityEngine;

public struct BLASNode
{
    public Vector3 BoundMax;
    public Vector3 BoundMin;
    public int PrimitiveStartIdx;
    public int PrimitiveEndIdx;
    public int MaterialIdx;
    public int ChildIdx;

    public static int TypeSize = sizeof(float) * 3 * 2 + sizeof(int) * 4; 
}

public struct MeshNode
{
    public Vector3 BoundMax;
    public Vector3 BoundMin;
    public int TransformIdx;
    public int NodeRootIdx;
    public int ChildIdx;

    public static int TypeSize = sizeof(float)*3*2+sizeof(int)*3;
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
        public int OriginTriOrMeshStartIndex;
        public int OriginTriOrMeshEndIndex;

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
                OriginTriOrMeshStartIndex = start,
                OriginTriOrMeshEndIndex = start + count
            };
            return node;
        }

        public static BVHNode CreateParent(BVHNode nodeLeft, BVHNode nodeRight)
        {
            BVHNode node = new BVHNode
            {
                Bounds = AABB.Combine(nodeLeft.Bounds, nodeRight.Bounds),
                LeftChild = nodeLeft,
                RightChild = nodeRight,
                OriginTriOrMeshStartIndex = -1,
                OriginTriOrMeshEndIndex = -1
            };
            return node;
        }
    }

    public struct PrimitiveBoundInfo
    {
        public AABB Bounds;
        public Vector3 Center;
        public int OriginTriOrMeshIndex;
    }

    protected abstract BVHNode Build(
        List<PrimitiveBoundInfo> Infos,
        int PrimitiveBoundInfoStart, int PrimitiveBoundInfoEnd, bool BLASFlag
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
            var node = q.Dequeue();
            bnodes.Add(new BLASNode
            {
                BoundMax         = node.Bounds.max,
                BoundMin         = node.Bounds.min,
                PrimitiveStartIdx= node.IsLeaf() ? node.OriginTriOrMeshStartIndex + globalPrimitiveBase : -1,
                PrimitiveEndIdx  = node.IsLeaf() ? node.OriginTriOrMeshEndIndex   + globalPrimitiveBase : -1,
                MaterialIdx      = node.IsLeaf() ? materialIdx : 0,
                ChildIdx         = node.IsLeaf() ? -1 : q.Count + bnodes.Count + 1
            });
            if (node.LeftChild  != null) q.Enqueue(node.LeftChild);
            if (node.RightChild != null) q.Enqueue(node.RightChild);
        }

        meshNodes.Add(new MeshNode
        {
            BoundMax     = BVHRoot.Bounds.max,
            BoundMin     = BVHRoot.Bounds.min,
            TransformIdx = objectTransformIdx,
            NodeRootIdx  = blasRootIdx
        });
        
        OriginTriOrMeshStartIndices.Clear();    // TODO: 优化OrderedPrimitiveIndices
    }
    
    public void FlattenTLAS(
        ref List<MeshNode> dst,
        List<MeshNode> leafSrc,
        IReadOnlyList<Matrix4x4> transforms)
    {
        dst.Clear();
        Queue<BVHNode> q = new();
        q.Enqueue(BVHRoot);

        while (q.Count > 0)
        {
            var cur = q.Dequeue();
            MeshNode n;

            if (cur.IsLeaf())
            {
                int orderedIdx = OriginTriOrMeshStartIndices[cur.OriginTriOrMeshStartIndex];
                MeshNode src   = leafSrc[orderedIdx];
                Matrix4x4 l2w = transforms[src.TransformIdx * 2];
                
                n = new MeshNode {
                    TransformIdx = src.TransformIdx,
                    NodeRootIdx  = src.NodeRootIdx,
                    ChildIdx     = -1
                };
                
                TransformUtils.TransformBounds(
                    l2w,
                    src.BoundMin,
                    src.BoundMax,
                    out n.BoundMin,
                    out n.BoundMax
                );
            }
            else
            {
                n = new MeshNode {
                    BoundMin     = cur.Bounds.min,
                    BoundMax     = cur.Bounds.max,
                    TransformIdx = -1,
                    NodeRootIdx  = -1,
                    ChildIdx     = dst.Count + q.Count + 1
                };

                if (cur.LeftChild != null)
                {
                    q.Enqueue(cur.LeftChild);
                    q.Enqueue(cur.RightChild);
                }
            }

            dst.Add(n);
        }
    }
    
    private readonly List<PrimitiveBoundInfo> sharedInfos = new List<PrimitiveBoundInfo>();
    
    // 将顶点和顶点对应的索引转换为一个个小bounding box(PrimitiveBoundInfo)，存储AABB和中心点信息
    // 此处生成的PrimitiveInfo的PrimitiveIdx与顶点的索引的对应关系是，OriginTriOrMeshIndex = 顶点索引 / 3 取整
    protected List<PrimitiveBoundInfo> CreatePrimitiveBoundInfo(Vector3[] vertices, int[] indices)
    {
        sharedInfos.Clear();
        int triCount = indices.Length / 3;
        // 预分配容量，减少扩容
        if (sharedInfos.Capacity < triCount) sharedInfos.Capacity = triCount;

        for (int i = 0; i < triCount; i++)
        {
            int i0 = indices[i * 3], i1 = indices[i * 3 + 1], i2 = indices[i * 3 + 2];
            var bounds = new AABB(vertices[i0], vertices[i1], vertices[i2]);
            sharedInfos.Add(new PrimitiveBoundInfo
            {
                Bounds = bounds,
                Center = bounds.Center(),
                OriginTriOrMeshIndex = i
            });
        }
        return sharedInfos;
    }

    protected List<PrimitiveBoundInfo> CreatePrimitiveBoundInfo
        (List<MeshNode> meshNodes, List<Matrix4x4> transforms)
    {
        sharedInfos.Clear();
        if (sharedInfos.Capacity < meshNodes.Count)
            sharedInfos.Capacity = meshNodes.Count;

        for (int i = 0; i < meshNodes.Count; i++)
        {
            var n = meshNodes[i];
            PrimitiveBoundInfo boundInfo;

            var l2w = transforms[n.TransformIdx * 2];
            
            TransformUtils.TransformBounds(
                l2w,
                n.BoundMin,
                n.BoundMax,
                out var WorldMin,
                out var WorldMax
            );
            
            boundInfo.Bounds = new AABB
            (
                WorldMin,
                WorldMax
            );
            boundInfo.Center       = boundInfo.Bounds.Center();
            boundInfo.OriginTriOrMeshIndex = i;
            sharedInfos.Add(boundInfo);
        }
        return sharedInfos;
    }

    public BVHNode BVHRoot = null;

    public List<int> OriginTriOrMeshStartIndices { get; protected set; } = new List<int>(4096);

    public static BVH Construct(Vector3[] vertices, int[] indices, BVHType type)
    {
                return new BVHSAH(vertices, indices);
    }

    public static BVH Construct(List<MeshNode> rawNodes, List<Matrix4x4> transforms, BVHType type)
    {
                return new BVHSAH(rawNodes, transforms);
    }
}
