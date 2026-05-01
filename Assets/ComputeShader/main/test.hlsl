#ifndef TEST_SHADER
#define TEST_SHADER

#include "global.hlsl"
#include "intersection.hlsl"

// float4 DebugBVH(uint2 pixID)
// {
//     Ray ray = GenRayByID(pixID.xy);
//     float  hitT = 1e20;
//     float3 color = 0;                           // 黑 = 没击中
//
//     /*------------- 2. 遍历 TLAS (数组根永远是 0) -------------*/
//     int  stackTLAS[BVHTREE_RECURSE_SIZE];
//     int  spT = 0;
//     stackTLAS[0] = 0;
//
//     while (spT >= 0)
//     {
//         int idxT = stackTLAS[spT--];
//         if (idxT >= _TLASNodesCount) break;     // 越界保护
//
//         TLASNode tn = _TLASNodes[idxT];
//         float4x4 localToWorld = _Transforms[tn.transformIdx * 2];
//
//         // 把 min/max 点都变到世界空间
//         float3 worldMin = mul(localToWorld, float4(tn.boundMin, 1.0)).xyz;
//         float3 worldMax = mul(localToWorld, float4(tn.boundMax, 1.0)).xyz;
//
//         float dstWorld = RayBoundingBoxDst(ray, worldMin, worldMax);
//
//         if (dstWorld >= 0.0f) {
//             return dstWorld * 0.2f;
//         } else {
//             return 0.0f;
//         }
//
//         /*--- 颜色：内部=蓝  叶子=绿 ---*/
//         float3 tlasCol = (tn.childIdx < 0) ? float3(0,1,0) : float3(0,0.4,1);
//         
//         if (dstWorld < hitT) { hitT = dstWorld; color = tlasCol; }
//
//         if (tn.childIdx >= 0)                   // 内部：压左右孩子
//         {
//             stackTLAS[++spT] = tn.childIdx;       // 左
//             stackTLAS[++spT] = tn.childIdx + 1;   // 右
//             continue;
//         }
//
//         /*------------- 3. 叶子 → 深入对应 BLAS -------------*/
//         float4x4 L2W = _Transforms[tn.transformIdx];
//
//         int  stackB[BVHTREE_RECURSE_SIZE];
//         int  spB = 0;
//         stackB[0] = tn.rootIdx;
//
//         while (spB >= 0)
//         {
//             int idxB = stackB[spB--];
//             if (idxB >= _BNodesCount) break;
//
//             BLASNode bn = _BNodes[idxB];
//
//             float3 bMin = mul(L2W, float4(bn.boundMin, 1)).xyz;
//             float3 bMax = mul(L2W, float4(bn.boundMax, 1)).xyz;
//
//             if (!IntersectBox2(ray, bMax, bMin)) continue;
//
//             float dstB = RayBoundingBoxDst(ray, bMin, bMax);
//             if (dstB < hitT)
//             {
//                 hitT  = dstB;
//                 color = float3(1,0,0) * 0.85;   // 红色半透
//             }
//
//             if (bn.primitiveStartIdx < 0)       // 内部继续压栈
//             {
//                 stackB[++spB] = bn.childIdx;
//                 stackB[++spB] = bn.childIdx + 1;
//             }
//         }
//     }
//
//     return float4(color, 1);
// }


float4 DebugBVH(uint2 pixID)
{
    Ray ray = GenRayByID(pixID);
    // 最小相交距离，初始化为一个很大的数
    float  bestT = 1e20;
    float3 bestCol = float3(0,0,0);  // 默认黑色

    // TLAS 遍历栈
    int  stackTLAS[BVHTREE_RECURSE_SIZE];
    int  spT = 0;
    stackTLAS[0] = 0;  // 从根节点开始

    while (spT >= 0)
    {
        int idxT = stackTLAS[spT--];
        if (idxT < 0 || idxT >= (int)_TLASNodesCount)
            continue;

        TLASNode tn = _TLASNodes[idxT];
        // 相交测试
        float t = RayBoundingBoxDst(ray, tn.boundMin, tn.boundMax);
        if (t >= 0.0 && t < bestT)
        {
            bestT   = t;
            // 如果想区分叶子/内部的颜色：
            bestCol = (tn.transformIdx >= 0)
                      ? float3(0,1,0)    // 叶子 = 绿
                      : float3(0,0.4,1); // 内部 = 蓝
        }

        // 如果这是内部节点，就把两个子节点压栈
        if (tn.transformIdx < 0)
        {
            stackTLAS[++spT] = tn.Index;       // 左子节点
            stackTLAS[++spT] = tn.Index + 1;   // 右子节点
        }
    }

    // 遍历结束后，返回最小距离对应的渐变颜色
    if (bestT < 1e20)
        return float4(bestCol * (bestT * 0.2), 1.0);
    else
        return float4(0,0,0,1);  // 完全未命中，黑色
}

#endif
