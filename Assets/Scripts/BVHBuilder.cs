using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using UnityEngine;

public struct MaterialData
{
    public Vector4 Color;
    public Vector3 Emission;
    public float Metallic;
    public float Smoothness;
    public float IOR;
    public float RenderMode;
    public int AlbedoIdx;
    public int EmitIdx;
    public int MetallicIdx;
    public int NormalIdx;
    public int RoughIdx;

    public static int TypeSize = sizeof(float)*11+sizeof(int)*5;
}

public class BVHBuilder
{
    // object data
    private static List<GameObject> objects = new List<GameObject>();
    // material data
    private static List<MaterialData> materials = new List<MaterialData>();
    // Mesh data
    private static List<Vector3> vertices = new List<Vector3>();
    private static List<Vector2> uvs = new List<Vector2>();
    private static List<Vector3> normals = new List<Vector3>();
    private static List<Vector4> tangents = new List<Vector4>();
    // Acceleration structure
    private static List<BLASNode> bnodes = new List<BLASNode>();
    private static List<MeshNode> meshNodes = new List<MeshNode>();

    // transform data, size of objects * 2, contains local to world and inverse matrix
    private static List<Matrix4x4> transforms = new List<Matrix4x4>();

    // algorithm data
    private static List<int> indices = new List<int>(); // indices of vertices

    public static ComputeBuffer VertexBuffer;
    public static ComputeBuffer UVBuffer;
    public static ComputeBuffer IndexBuffer;
    public static ComputeBuffer NormalBuffer;
    public static ComputeBuffer TangentBuffer;
    public static ComputeBuffer MaterialBuffer;
    public static ComputeBuffer BLASBuffer;
    public static ComputeBuffer MeshNodeBuffer;
    public static ComputeBuffer TransformBuffer;
    
    public static Texture2DArray AlbedoTextures = null;
    public static Texture2DArray EmissionTextures = null;
    public static Texture2DArray MetallicTextures = null;
    public static Texture2DArray NormalTextures = null;
    public static Texture2DArray RoughnessTextures = null;

    private static bool objectUpdated = false;
    private static bool objectTransformUpdated = false;

    public static void RegisterObject(GameObject o)
    {
        objects.Add(o);
        objectUpdated = true;
        objectTransformUpdated = true;
    }

    public static void UnregisterObject(GameObject o)
    {
        objects.Remove(o);
        objectUpdated = true;
        objectTransformUpdated = true;
    }

    public static bool Validate()
    {
        // —— 1. 检测物体结构变化（Register/Unregister） —— //
        if (objectUpdated)
        {
            BuildBVH();
            LoadTransforms();
            objectUpdated = false;
            objectTransformUpdated = false;
            return true;
        }

        bool anyMoved = false;
        foreach (var obj in objects)
        {
            if (obj.transform.hasChanged || (obj.transform.parent != null && obj.transform.parent.hasChanged))
            {
                anyMoved = true;
                obj.transform.hasChanged = false;
                if (obj.transform.parent != null) obj.transform.parent.hasChanged = false;
                break;
            }
        }

        if (anyMoved || objectTransformUpdated)
        {
            LoadTransforms();
            objectTransformUpdated = false;
            return true;
        }

        return false;
    }
    
    private static readonly int ID_MainTex          = Shader.PropertyToID("_MainTex");
    private static readonly int ID_EmissionMap      = Shader.PropertyToID("_EmissionMap");
    private static readonly int ID_MetallicGlossMap = Shader.PropertyToID("_MetallicGlossMap");
    private static readonly int ID_BumpMap          = Shader.PropertyToID("_BumpMap");
    private static readonly int ID_SpecGlossMap     = Shader.PropertyToID("_SpecGlossMap");
    private static readonly int ID_EmissionColor    = Shader.PropertyToID("_EmissionColor");
    private static readonly int ID_Metallic         = Shader.PropertyToID("_Metallic");
    private static readonly int ID_Glossiness       = Shader.PropertyToID("_Glossiness");
    private static readonly int ID_IOR              = Shader.PropertyToID("_IOR");
    private static readonly int ID_Mode             = Shader.PropertyToID("_Mode");
    
    private struct SubMeshKey
    {
        public Mesh Mesh;
        public int  SubMeshIndex;
        public override int GetHashCode() => Mesh.GetHashCode() * 397 ^ SubMeshIndex;
        public override bool Equals(object obj) =>
            obj is SubMeshKey o && o.Mesh == Mesh && o.SubMeshIndex == SubMeshIndex;
    }

    private static void BuildMaterialAndMeshData(List<GameObject> objects)
    {
        materials.Clear();
        indices   .Clear();
        bnodes    .Clear();
        meshNodes .Clear();
        
        Dictionary<Mesh, (int start, int count)> meshCache = new();
        Dictionary<(Mesh mesh,int subIdx), (BVH bvh, int indexStart)> subMeshCache = new();
        
        var albedoMap   = new Dictionary<Texture2D,int>();
        var emitMap     = new Dictionary<Texture2D,int>();
        var metalMap    = new Dictionary<Texture2D,int>();
        var normalMap   = new Dictionary<Texture2D,int>();
        var roughMap    = new Dictionary<Texture2D,int>();
        var objIndexMap = new Dictionary<GameObject,int>(objects.Count);
        for (int i = 0; i < objects.Count; i++)
            objIndexMap[objects[i]] = i;

        materials.Add(new MaterialData()
        {
            Color = new Vector4(1.0f, 1.0f, 1.0f, 1.0f),
            Emission = Vector3.zero,
            Metallic = 0.0f,
            Smoothness = 0.0f,
            IOR = 1.0f,
            RenderMode = 0,
            AlbedoIdx = -1,
            EmitIdx = -1,
            MetallicIdx = -1,
            NormalIdx = -1,
            RoughIdx = -1
        });
        var albedoTex = new List<Texture2D>();
        var emitTex   = new List<Texture2D>();
        var metalTex  = new List<Texture2D>();
        var normTex   = new List<Texture2D>();
        var roughTex  = new List<Texture2D>();

        for (int objIdx = 0; objIdx < objects.Count; objIdx++)
        {
            GameObject obj       = objects[objIdx];
            Renderer renderer = obj.GetComponent<Renderer>();
            var sharedMats = renderer.sharedMaterials;
            int matStart   = materials.Count;
            foreach (var mat in sharedMats)
            {
                ExtractMaterialTexture(mat, albedoMap, albedoTex,  ID_MainTex,          out int albedoIdx);
                ExtractMaterialTexture(mat, emitMap,   emitTex,    ID_EmissionMap,      out int emitIdx);
                ExtractMaterialTexture(mat, metalMap,  metalTex,   ID_MetallicGlossMap, out int metalIdx);
                ExtractMaterialTexture(mat, normalMap,   normTex,    ID_BumpMap,          out int normIdx);
                ExtractMaterialTexture(mat, roughMap,  roughTex,   ID_SpecGlossMap,     out int roughIdx);

                Color emCol = mat.IsKeywordEnabled("_EMISSION") ? mat.GetColor(ID_EmissionColor) : Color.black;
                materials.Add(new MaterialData{
                    Color      = mat.color,
                    Emission   = new Vector3(emCol.r, emCol.g, emCol.b),
                    Metallic   = mat.HasProperty(ID_Metallic)   ? mat.GetFloat(ID_Metallic)   : 0f,
                    Smoothness = mat.HasProperty(ID_Glossiness) ? mat.GetFloat(ID_Glossiness) : 0f,
                    IOR        = mat.HasProperty(ID_IOR)        ? mat.GetFloat(ID_IOR)        : 1f,
                    RenderMode = mat.HasProperty(ID_Mode)       ? mat.GetFloat(ID_Mode)       : 0f,
                    AlbedoIdx  = albedoIdx, EmitIdx   = emitIdx,
                    MetallicIdx= metalIdx,  NormalIdx = normIdx,
                    RoughIdx   = roughIdx
                });
            }

            Mesh mesh = obj.GetComponent<MeshFilter>().sharedMesh;
            
            if (!meshCache.TryGetValue(mesh, out var vRange))
            {
                int vStart = vertices.Count;
                vertices.AddRange(mesh.vertices);
                normals .AddRange(mesh.normals);
                var u  = mesh.uv;       verticesEnsure(uvs,     u,  mesh.vertexCount, Vector2.zero);
                var tg = mesh.tangents; verticesEnsure(tangents, tg, mesh.vertexCount, Vector4.zero);
                vRange = (vStart, mesh.vertexCount);
                meshCache[mesh] = vRange;
            }
            int vertexStart = vRange.start;
            
            for (int s = 0; s < mesh.subMeshCount; s++)
            {
                var key = (mesh, s);

                if (!subMeshCache.TryGetValue(key, out var entry))
                {
                    int[] subIdx = mesh.GetIndices(s);
                    BVH   bvh    = BVH.Construct(mesh.vertices, subIdx, BVHType.SAH);

                    int indexStart = indices.Count;
                    foreach (int p in bvh.OrderedPrimitiveIndices)
                    {
                        indices.Add(subIdx[p*3+0] + vertexStart);
                        indices.Add(subIdx[p*3+1] + vertexStart);
                        indices.Add(subIdx[p*3+2] + vertexStart);
                    }

                    entry = (bvh, indexStart);
                    subMeshCache[key] = entry;
                }

                int primitiveBase = entry.indexStart / 3;
                
                entry.bvh.FlattenBLAS(
                    ref bnodes,
                    ref meshNodes,
                    s < sharedMats.Length ? matStart + s : 0,
                    objIdx,                       // TransformIdx == object idx
                    primitiveBase
                );
            }
        }
        // if not UV is used, insert empty one
        if (uvs.Count == 0) uvs.Add(Vector2.zero);
        
        // create texture 2d array
        if (AlbedoTextures != null) UnityEngine.Object.Destroy(AlbedoTextures);
        if (EmissionTextures != null) UnityEngine.Object.Destroy(EmissionTextures);
        if (MetallicTextures != null) UnityEngine.Object.Destroy(MetallicTextures);
        if (NormalTextures != null) UnityEngine.Object.Destroy(NormalTextures);
        if (RoughnessTextures != null) UnityEngine.Object.Destroy(RoughnessTextures);
        AlbedoTextures = CreateTextureArray(ref albedoTex);
        EmissionTextures = CreateTextureArray(ref emitTex);
        MetallicTextures = CreateTextureArray(ref metalTex);
        NormalTextures = CreateTextureArray(ref normTex);
        RoughnessTextures = CreateTextureArray(ref roughTex);
    }
    
    static void verticesEnsure<T>(List<T> dst, T[] src, int count, T pad)
    {
        if (src != null && src.Length == count) dst.AddRange(src);
        else dst.AddRange(System.Linq.Enumerable.Repeat(pad, count));
    }
    
    static void ExtractMaterialTexture(
        Material mat, Dictionary<Texture2D,int> map, List<Texture2D> list,
        int propId, out int idx)
    {
        idx = -1;
        if (mat.HasProperty(propId) && mat.GetTexture(propId) is Texture2D t)
        {
            if (!map.TryGetValue(t, out idx))
            {
                idx = list.Count;
                list.Add(t);
                map[t] = idx;
            }
        }
    }

    private static bool BuildBVH()
    {
        if (!objectUpdated)
            return false;
        
        vertices.Clear();
        uvs.Clear();
        indices.Clear();
        normals.Clear();
        tangents.Clear();
        materials.Clear();
        bnodes.Clear();
        meshNodes.Clear();

        BuildMaterialAndMeshData(objects);

        // build TLAS bvh
        RebuildAS();

        SetBuffers();

        objectUpdated = false;
        return true;
    }

    private static bool LoadTransforms()
    {
        if (!objectTransformUpdated) return false;

        transforms.Clear();

        // 突然发现，由于每次都是使用“foreach(var obj in objects)”遍历所有物体，所以这些数组的索引都是一一对应的
        foreach (var obj in objects)
        {
            transforms.Add(obj.transform.localToWorldMatrix);
            transforms.Add(obj.transform.worldToLocalMatrix);
        }

        SetBuffer(ref TransformBuffer, transforms, sizeof(float) * 4 * 4);

        objectTransformUpdated = false;
        return true;
    }

    private static void SetBuffers()
    {
        SetBuffer(ref IndexBuffer, indices, sizeof(int));
        SetBuffer(ref VertexBuffer, vertices, sizeof(float) * 3);
        SetBuffer(ref UVBuffer, uvs, sizeof(float) * 2);
        SetBuffer(ref NormalBuffer, normals, sizeof(float) * 3);
        SetBuffer(ref TangentBuffer, tangents, sizeof(float) * 4);
        SetBuffer(ref MaterialBuffer, materials, MaterialData.TypeSize);
        SetBuffer(ref BLASBuffer, bnodes, BLASNode.TypeSize);
    }
    
    private static List<MeshNode> tlasNodes = new List<MeshNode>();

    public static IReadOnlyList<MeshNode> GetTLASNodes() => tlasNodes;

    public static BVH tlasTree;
    
    public static void RebuildAS()
    {
        if (meshNodes.Count <= 0) return;
        if (transforms.Count <= 0) LoadTransforms();
        tlasTree = BVH.Construct(meshNodes, transforms, BVHType.SAH);
        tlasNodes.Clear();
        tlasTree.FlattenTLAS(ref tlasNodes, meshNodes, transforms);
        SetBuffer(ref MeshNodeBuffer, tlasNodes, MeshNode.TypeSize);
    }

    private static void SetBuffer<T>(ref ComputeBuffer buffer, List<T> data, int stride) where T : struct
    {
        if (data.Count == 0) return;
        if (buffer == null || buffer.count != data.Count || buffer.stride != stride) {
            buffer?.Release();
            buffer = new ComputeBuffer(data.Count, stride);
        }
        buffer.SetData(data);
    }

    public static void ReloadMaterials()
    {
        int matIdx = 1;
        // get info from each object
        foreach (var obj in objects)
        {
            // load materials
            var meshMats = obj.GetComponent<Renderer>().sharedMaterials;
            foreach (var mat in meshMats)
            {
                Color emission = mat.IsKeywordEnabled("_EMISSION") ? mat.GetColor("_EmissionColor") : Color.black;
                materials[matIdx] = new MaterialData()
                {
                    Color = new Vector4(mat.color.r, mat.color.g, mat.color.b, mat.color.a),
                    Emission = new Vector3(emission.r, emission.g, emission.b),
                    Metallic = mat.GetFloat("_Metallic"),
                    Smoothness = mat.GetFloat("_Glossiness"),
                    IOR = mat.HasProperty("_IOR") ? mat.GetFloat("_IOR") : 1.0f,
                    RenderMode = mat.GetFloat("_Mode"),
                    AlbedoIdx = materials[matIdx].AlbedoIdx,
                    EmitIdx = materials[matIdx].EmitIdx,
                    MetallicIdx = materials[matIdx].MetallicIdx,
                    NormalIdx = materials[matIdx].NormalIdx,
                    RoughIdx = materials[matIdx].RoughIdx,
                };
                matIdx++;
            }
        }
        SetBuffer(ref MaterialBuffer, materials, MaterialData.TypeSize);
    }

    public static void Destroy()
    {
        if (IndexBuffer != null) IndexBuffer.Release();
        if (VertexBuffer != null) VertexBuffer.Release();
        if (NormalBuffer != null) NormalBuffer.Release();
        if (TangentBuffer != null) TangentBuffer.Release();
        if (UVBuffer != null) UVBuffer.Release();
        if (MaterialBuffer != null) MaterialBuffer.Release();
        if (MeshNodeBuffer != null) MeshNodeBuffer.Release();
        if (BLASBuffer != null) BLASBuffer.Release();
        if (TransformBuffer != null) TransformBuffer.Release();
        if (AlbedoTextures != null) UnityEngine.Object.Destroy(AlbedoTextures);
        if (EmissionTextures != null) UnityEngine.Object.Destroy(EmissionTextures);
        if (MetallicTextures != null) UnityEngine.Object.Destroy(MetallicTextures);
        if (NormalTextures != null) UnityEngine.Object.Destroy(NormalTextures);
        if (RoughnessTextures != null) UnityEngine.Object.Destroy(RoughnessTextures);
    }

    private static Texture2DArray CreateTextureArray(ref List<Texture2D> textures)
    {
        int sliceCount = Mathf.Max(1, textures.Count);

        int maxW = 1, maxH = 1;
        foreach (var tex in textures)
        {
            if (tex == null) continue;
            maxW = Mathf.Max(maxW, tex.width);
            maxH = Mathf.Max(maxH, tex.height);
        }

        int maxDim = GetMaxDimension(sliceCount, Mathf.Max(maxW, maxH));
        int targetW = Mathf.Clamp(maxW, 1, maxDim);
        int targetH = Mathf.Clamp(maxH, 1, maxDim);

        var array = new Texture2DArray(
            targetW, targetH, sliceCount,
            TextureFormat.ARGB32, /*mip*/ true, /*linear*/ false
        );
        Color32[] clearColors = Enumerable.Repeat<Color32>(new Color32(255, 255, 255, 255), targetW * targetH).ToArray();
        for (int i = 0; i < sliceCount; ++i)
            array.SetPixels32(clearColors, i, 0);

        RenderTexture blitRT = new RenderTexture(targetW, targetH, 0, RenderTextureFormat.ARGB32)
        {
            filterMode = FilterMode.Bilinear,
            useMipMap   = true,
            autoGenerateMips = false,
            enableRandomWrite = false
        };
        blitRT.Create();

        for (int i = 0; i < sliceCount; i++)
        {
            Texture src = (i < textures.Count && textures[i] != null) ? textures[i] : Texture2D.whiteTexture;
            Graphics.Blit(src, blitRT);
            Graphics.CopyTexture(blitRT, 0, 0, array, i, 0);
        }

        array.Apply(updateMipmaps: true, makeNoLongerReadable: true);
        blitRT.Release();

        return array;
    }

    private static int GetMaxDimension(int count, int dim)
    {
        // 看上去是用于纹理压缩
        if (dim >= 2048)
        {
            if (count <= 16) return 2048;
            else return 1024;
        }
        else if (dim >= 1024)
        {
            if (count <= 48) return 1024;
            else return 512;
        }
        else return dim;
    }

    public static List<BLASNode> GetBLASNodes()
    {
        return bnodes;
    }

    public static List<MeshNode> GetMeshNodes()
    {
        return meshNodes;
    }

    public static List<Matrix4x4> GetTransforms()
    {
        return transforms;
    }
}
