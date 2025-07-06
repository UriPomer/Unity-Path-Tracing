using System;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

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
    
    // used to reduce memory allocation in recursive build
    private static readonly SAHBucket[] bucketsCache = new SAHBucket[nBuckets];

    public BVHSAH(Vector3[] vertices,int[] indices)
    {
        OriginTriOrMeshStartIndices.Clear();
        var primitiveBoundInfo = CreatePrimitiveBoundInfo(vertices, indices);
        BVHRoot = Build(primitiveBoundInfo, 0, primitiveBoundInfo.Count, true);
    }

    public BVHSAH(List<MeshNode> rawNodes, List<Matrix4x4> transforms)
    {
        OriginTriOrMeshStartIndices.Clear();
        var primitiveBoundInfo = CreatePrimitiveBoundInfo(rawNodes, transforms);
        BVHRoot = Build(primitiveBoundInfo, 0, primitiveBoundInfo.Count, false);
    }

    protected sealed override BVHNode Build(List<PrimitiveBoundInfo> Infos, int PrimitiveBoundInfoStart, int PrimitiveBoundInfoEnd, bool BLASFlag)
    {
        //// ------------- //// Total Bounds
        bool IsBuildingBLAS = BLASFlag;
        AABB BoundingBox = new AABB();
        for (int i = PrimitiveBoundInfoStart; i < PrimitiveBoundInfoEnd; i++) BoundingBox.Extend(Infos[i].Bounds);

        int count = PrimitiveBoundInfoEnd - PrimitiveBoundInfoStart;
        if (count == 1)
        {
            int dst = OriginTriOrMeshStartIndices.Count;
            OriginTriOrMeshStartIndices.Add(Infos[PrimitiveBoundInfoStart].OriginTriOrMeshIndex);
            return BVHNode.CreateLeaf(dst, 1, BoundingBox);
        }

        var CenterBoundingBox = new AABB();
        for (int i = PrimitiveBoundInfoStart; i < PrimitiveBoundInfoEnd; i++)
            CenterBoundingBox.Extend(Infos[i].Center);
            
        int SplitAxis = CenterBoundingBox.MaxDimension();

        if (IsBuildingBLAS && CenterBoundingBox.extent[SplitAxis] < 1e-4f)
        {
            int dst = OriginTriOrMeshStartIndices.Count;
            for (int i = PrimitiveBoundInfoStart; i < PrimitiveBoundInfoEnd; i++)
                OriginTriOrMeshStartIndices.Add(Infos[i].OriginTriOrMeshIndex);
            return BVHNode.CreateLeaf(dst, count, BoundingBox);
        }
        //// ------------- ////

        //// ------------- //// Calculate Buckets
        for (int i = 0; i < nBuckets; i++) bucketsCache[i].Reset();

        float extent = CenterBoundingBox.extent[SplitAxis];
        float invSplitAxisLength = extent > 1e-5f ? 1f / extent : 0f;

        for (int i = PrimitiveBoundInfoStart; i < PrimitiveBoundInfoEnd; i++)
        {
            int b = (int)((Infos[i].Center[SplitAxis] - CenterBoundingBox.min[SplitAxis]) * invSplitAxisLength * nBuckets);
            b = Mathf.Clamp(b, 0, nBuckets - 1);

            ref SAHBucket bucket = ref bucketsCache[b];
            bucket.Count++;
            bucket.Bounds.Extend(Infos[i].Bounds);
        }

        Span<float> areaL = stackalloc float[nBuckets - 1];
        Span<float> areaR = stackalloc float[nBuckets - 1];
        Span<int>   countL  = stackalloc int  [nBuckets - 1];
        Span<int>   countR  = stackalloc int  [nBuckets - 1];

        AABB tempBoundingBox = new AABB();
        int accumulateCount = 0;
        for (int i = 0; i < nBuckets - 1; i++)
        {
            accumulateCount += bucketsCache[i].Count;
            countL[i] = accumulateCount;
            tempBoundingBox.Extend(bucketsCache[i].Bounds);
            areaL[i] = tempBoundingBox.SurfaceArea();
        }
        tempBoundingBox.Reset(); accumulateCount = 0;
        for (int i = nBuckets - 1; i > 0; i--)
        {
            accumulateCount += bucketsCache[i].Count;
            countR[i - 1] = accumulateCount;
            tempBoundingBox.Extend(bucketsCache[i].Bounds);
            areaR[i - 1] = tempBoundingBox.SurfaceArea();
        }

        float bestCost = float.MaxValue; int bestSplit = -1;
        float invSA = 1f / BoundingBox.SurfaceArea();

        const float TraversalCost = 1f;
        for (int i = 0; i < nBuckets - 1; i++)
        {
            if (countL[i] == 0 || countR[i] == 0) continue;
            float cost = TraversalCost + (countL[i] * areaL[i] + countR[i] * areaR[i]) * invSA;
            if (cost < bestCost) { bestCost = cost; bestSplit = i; }
        }

        if (IsBuildingBLAS && bestCost >= count)
        {
            int dst = OriginTriOrMeshStartIndices.Count;
            for (int i = PrimitiveBoundInfoStart; i < PrimitiveBoundInfoEnd; i++)
                OriginTriOrMeshStartIndices.Add(Infos[i].OriginTriOrMeshIndex);
            return BVHNode.CreateLeaf(dst, count, BoundingBox);
        }

        Infos.Sort(PrimitiveBoundInfoStart, count, Comparer<PrimitiveBoundInfo>.Create((a, b) =>
        {
            int ba = Mathf.Clamp(
                (int)((a.Center[SplitAxis] - CenterBoundingBox.min[SplitAxis]) * invSplitAxisLength * nBuckets), 0, nBuckets - 1);
            int bb = Mathf.Clamp(
                (int)((b.Center[SplitAxis] - CenterBoundingBox.min[SplitAxis]) * invSplitAxisLength * nBuckets), 0, nBuckets - 1);
            return ba.CompareTo(bb);
        }));

        int mid = PrimitiveBoundInfoStart;
        for (int i = 0; i <= bestSplit; i++) mid += bucketsCache[i].Count;
        if (mid == PrimitiveBoundInfoStart || mid == PrimitiveBoundInfoEnd) mid = (PrimitiveBoundInfoStart + PrimitiveBoundInfoEnd) >> 1;

        var left  = Build(Infos, PrimitiveBoundInfoStart, mid, IsBuildingBLAS);
        var right = Build(Infos, mid,   PrimitiveBoundInfoEnd, IsBuildingBLAS);
        return BVHNode.CreateParent(left, right);
    }
}
