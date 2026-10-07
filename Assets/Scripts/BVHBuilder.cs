// #define DEBUG_TEXTURE

using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using UnityEngine;

public struct MaterialData
{
    public Vector4 Color;
    public Vector3 Emission;
    public float  EmissionIntensity;
    public float Metallic;
    public float Smoothness;
    public float IOR;
    public float RenderMode;
    public int AlbedoIdx;
    public int EmitIdx;
    public int MetallicIdx;
    public int NormalIdx;
    public int RoughIdx;

    public static int TypeSize = sizeof(float)*12+sizeof(int)*5;
}

public class BVHBuilder
{
    // object data
    private static List<GameObject> objects = new List<GameObject>();
    public static IReadOnlyList<GameObject> GetObjects() => objects;
    // material data
    private static List<MaterialData> materials = new List<MaterialData>();
    public static IReadOnlyList<MaterialData> GetMaterials() => materials;
    // Mesh data
    private static List<Vector3> vertices = new List<Vector3>();
    public static IReadOnlyList<Vector3> GetVertices() => vertices;
    private static List<Vector2> uvs = new List<Vector2>();
    public static IReadOnlyList<Vector2> GetUVs() => uvs;
    private static List<Vector3> normals = new List<Vector3>();
    public static IReadOnlyList<Vector3> GetNormals() => normals;
    private static List<Vector4> tangents = new List<Vector4>();
    public static IReadOnlyList<Vector4> GetTangents() => tangents;
    // Acceleration structure
    private static List<BLASNode> bnodes = new List<BLASNode>();
    public static IReadOnlyList<BLASNode> GetBLASNodes() => bnodes;
    private static List<MeshNode> meshNodes = new List<MeshNode>();
    public static IReadOnlyList<MeshNode> GetMeshNodes() => meshNodes;
    private static List<MeshNode> tlasNodes = new List<MeshNode>();
    public static IReadOnlyList<MeshNode> GetTLASNodes() => tlasNodes;

    public static BVH tlasTree;

    // transform data, size of objects * 2, contains local to world and inverse matrix
    private static List<Matrix4x4> transforms = new List<Matrix4x4>();

    // algorithm data
    private static List<int> indices = new List<int>(); // indices of vertices
    public static List<int> GetIndices() => indices;

    public static ComputeBuffer VertexBuffer;
    public static ComputeBuffer TriangleBuffer;
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

    private static bool objectUpdated = true;
    public static int GeometryRevision { get; private set; }
    private static bool objectTransformUpdated = false;

    public static void RegisterObject(GameObject o)
    {
        if (objects.Contains(o)) return;
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
            GeometryRevision++;
            BuildBVH();
            LoadTransforms();
            RebuildTLAS();
            objectUpdated = false;
            objectTransformUpdated = false;
            return true;
        }

        bool anyMoved = false;
        for (int i = 0; i < objects.Count; i++)
            anyMoved |= transforms.Count <= i * 2 || transforms[i * 2] != objects[i].transform.localToWorldMatrix;

        if (anyMoved || objectTransformUpdated)
        {
            LoadTransforms();
            RebuildTLAS();
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
    private static float ReadIor(Material material)
    {
        float ior = material.HasProperty(ID_IOR) ? material.GetFloat(ID_IOR) : 1.5f;
        return float.IsFinite(ior) && ior > 0.0f ? ior : 1.5f;
    }

    private struct SubMeshKey
    {
        public Mesh Mesh;
        public int  SubMeshIndex;
        public override int GetHashCode() => Mesh.GetHashCode() * 397 ^ SubMeshIndex;
        public override bool Equals(object obj) =>
            obj is SubMeshKey o && o.Mesh == Mesh && o.SubMeshIndex == SubMeshIndex;
    }
    private static void BuildMaterialAndMeshData(List<GameObject> SceneObjects)
    {
        // Packed offsets are valid only for this build's vertex/index arrays.
        var subMeshCache = new Dictionary<SubMeshKey, (BVH bvh, int indexStart)>();
        materials.Clear();
        indices   .Clear();
        bnodes    .Clear();
        meshNodes .Clear();
        
        var albedoMap   = new Dictionary<Texture2D,int>();
        var emitMap     = new Dictionary<Texture2D,int>();
        var metalMap    = new Dictionary<Texture2D,int>();
        var normalMap   = new Dictionary<Texture2D,int>();
        var roughMap    = new Dictionary<Texture2D,int>();

        materials.Add(new MaterialData()
        {
            Color = new Vector4(1.0f, 1.0f, 1.0f, 1.0f),
            Emission = Vector3.zero,
            Metallic = 0.0f,
            Smoothness = 0.0f,
            IOR = 1.5f,
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

        for (int objIdx = 0; objIdx < SceneObjects.Count; objIdx++)
        {
            GameObject obj       = SceneObjects[objIdx];
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
                
                Vector3 EmissionColor = Vector3.zero;
                float Intensity = 0;
                if (mat.IsKeywordEnabled("_EMISSION"))
                {
                    Color hdr = mat.GetColor(ID_EmissionColor);
                    obj.TryGetComponent<Emission>(out var emission);
                    EmissionColor = new Vector3(hdr.r, hdr.g, hdr.b);
                    Intensity = emission?.Intensity ?? 0;
                }
                
                materials.Add(new MaterialData{
                    Color      = mat.color,
                    Emission   = EmissionColor,
                    EmissionIntensity = Intensity,
                    Metallic   = mat.HasProperty(ID_Metallic)   ? mat.GetFloat(ID_Metallic)   : 0f,
                    Smoothness = mat.HasProperty(ID_Glossiness) ? mat.GetFloat(ID_Glossiness) : 0f,
                    IOR        = ReadIor(mat),
                    RenderMode = mat.HasProperty(ID_Mode)       ? mat.GetFloat(ID_Mode)       : 0f,
                    AlbedoIdx  = albedoIdx, EmitIdx   = emitIdx,
                    MetallicIdx= metalIdx,  NormalIdx = normIdx,
                    RoughIdx   = roughIdx
                });
            }

            Mesh mesh = obj.GetComponent<MeshFilter>().sharedMesh;
            int vertexStart = vertices.Count;
            vertices.AddRange(mesh.vertices);
            normals .AddRange(mesh.normals);
            verticesEnsure(uvs, mesh.uv, mesh.vertexCount, Vector2.zero);
            verticesEnsure(tangents, mesh.tangents, mesh.vertexCount, Vector4.zero);
            
            for (int submeshidx = 0; submeshidx < mesh.subMeshCount; submeshidx++)
            {
                var key = new SubMeshKey
                {
                    Mesh = mesh,
                    SubMeshIndex = submeshidx
                };

                if (!subMeshCache.TryGetValue(key, out var entry))
                {
                    int[] subIdx = mesh.GetIndices(submeshidx);
                    BVH   bvh    = BVH.Construct(mesh.vertices, subIdx, BVHType.SAH);

                    int indexStart = indices.Count;
                    foreach (int p in bvh.OriginTriOrMeshStartIndices)
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
                    submeshidx < sharedMats.Length ? matStart + submeshidx : 0,
                    objIdx,                       // TransformIdx == object idx
                    primitiveBase
                );
            }
        }
        if (uvs.Count == 0) uvs.Add(Vector2.zero);
        
        // create texture 2d array
        if (AlbedoTextures) UnityEngine.Object.Destroy(AlbedoTextures);
        if (EmissionTextures) UnityEngine.Object.Destroy(EmissionTextures);
        if (MetallicTextures) UnityEngine.Object.Destroy(MetallicTextures);
        if (NormalTextures) UnityEngine.Object.Destroy(NormalTextures);
        if (RoughnessTextures) UnityEngine.Object.Destroy(RoughnessTextures);
        AlbedoTextures = BVHTextureArrays.Create(albedoTex, BVHTextureArrays.Kind.Color);
#if UNITY_EDITOR && DEBUG_TEXTURE
        UnityEditor.EditorApplication.delayCall += () =>
        {
            var win = UnityEditor.EditorWindow.GetWindow<TextureArrayPreviewWindow>("Texture2DArray 预览");
            win.array = AlbedoTextures;
            win.Repaint();
        };
        
        UnityEditor.EditorApplication.delayCall += () =>
        {
            var win = UnityEditor.EditorWindow.GetWindow<TexturePreviewWindow>("Texture2D 预览");
            win.texture2D = albedoTex[0];
            win.Repaint();
        };
#endif
        EmissionTextures = BVHTextureArrays.Create(emitTex, BVHTextureArrays.Kind.Color);
        MetallicTextures = BVHTextureArrays.Create(metalTex, BVHTextureArrays.Kind.Data);
        NormalTextures = BVHTextureArrays.Create(normTex, BVHTextureArrays.Kind.Normal);
        RoughnessTextures = BVHTextureArrays.Create(roughTex, BVHTextureArrays.Kind.Data);
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
        
        var stopwatch = Stopwatch.StartNew();
        
        vertices.Clear();
        uvs.Clear();
        indices.Clear();
        normals.Clear();
        tangents.Clear();
        materials.Clear();
        bnodes.Clear();
        meshNodes.Clear();

        BuildMaterialAndMeshData(objects);

        SetBLASBuffers();
        
        objectUpdated = false;
        
        stopwatch.Stop();
        UnityEngine.Debug.Log($"[BuildBVH] 耗时：{stopwatch.Elapsed.TotalMilliseconds:F2} ms");
        return true;
    }

    private static bool LoadTransforms()
    {
        transforms.Clear();

        foreach (var obj in objects)
        {
            transforms.Add(obj.transform.localToWorldMatrix);
            transforms.Add(obj.transform.worldToLocalMatrix);
        }

        SetBuffer(ref TransformBuffer, transforms, sizeof(float) * 4 * 4);

        objectTransformUpdated = false;
        return true;
    }

    private static void SetBLASBuffers()
    {
        SetBuffer(ref IndexBuffer, indices, sizeof(int));
        SetBuffer(ref VertexBuffer, vertices, sizeof(float) * 3);
        // Flattened BLAS primitive order: v0, e1, e2. Rigid transforms keep this local cache valid.
        var triangles = new List<Vector3>(indices.Count);
        for (int i = 0; i < indices.Count; i += 3) {
            Vector3 v0 = vertices[indices[i]];
            triangles.Add(v0);
            triangles.Add(vertices[indices[i+1]] - v0);
            triangles.Add(vertices[indices[i+2]] - v0);
        }
        SetBuffer(ref TriangleBuffer, triangles, sizeof(float) * 3);
        SetBuffer(ref UVBuffer, uvs, sizeof(float) * 2);
        SetBuffer(ref NormalBuffer, normals, sizeof(float) * 3);
        SetBuffer(ref TangentBuffer, tangents, sizeof(float) * 4);
        SetBuffer(ref MaterialBuffer, materials, MaterialData.TypeSize);
        SetBuffer(ref BLASBuffer, bnodes, BLASNode.TypeSize);
    }
    public static void RebuildTLAS()
    {
        tlasNodes.Clear();
        tlasTree = meshNodes.Count > 0 ? BVH.Construct(meshNodes, transforms, BVHType.SAH) : null;
        tlasTree?.FlattenTLAS(ref tlasNodes, meshNodes, transforms);
        SetBuffer(ref MeshNodeBuffer, tlasNodes, MeshNode.TypeSize);
    }

    private static void SetBuffer<T>(ref ComputeBuffer buffer, List<T> data, int stride) where T : struct
    {
        int count = Mathf.Max(data.Count, 1);
        if (buffer == null || buffer.count != count || buffer.stride != stride) {
            buffer?.Release();
            buffer = new ComputeBuffer(count, stride);
        }
        if (data.Count > 0) buffer.SetData(data);
    }

    private static readonly List<Material> reloadMaterials = new List<Material>();

    public static bool ReloadMaterials()
    {
        if (objectUpdated || materials.Count <= 1 || objects.Count == 0)
            return false;

        int matIdx = 1;
        bool changed = false;
        // get info from each object
        foreach (var obj in objects)
        {
            if (obj == null) continue;

            obj.GetComponent<Renderer>().GetSharedMaterials(reloadMaterials);
            foreach (var mat in reloadMaterials)
            {
                Color emission = mat.IsKeywordEnabled("_EMISSION") ? mat.GetColor("_EmissionColor") : Color.black;
                obj.TryGetComponent<Emission>(out var emissionComponent);
                float emissionIntensity = mat.IsKeywordEnabled("_EMISSION")
                    ? emissionComponent?.Intensity ?? 0.0f
                    : 0.0f;

                MaterialData newData = new MaterialData()
                {
                    Color = new Vector4(mat.color.r, mat.color.g, mat.color.b, mat.color.a),
                    Emission = new Vector3(emission.r, emission.g, emission.b),
                    EmissionIntensity = emissionIntensity,
                    Metallic = mat.HasProperty("_Metallic") ? mat.GetFloat("_Metallic") : 0.0f,
                    Smoothness = mat.HasProperty("_Glossiness") ? mat.GetFloat("_Glossiness") : 0.0f,
                    IOR = ReadIor(mat),
                    RenderMode = mat.HasProperty("_Mode") ? mat.GetFloat("_Mode") : 0.0f,
                    AlbedoIdx = materials[matIdx].AlbedoIdx,
                    EmitIdx = materials[matIdx].EmitIdx,
                    MetallicIdx = materials[matIdx].MetallicIdx,
                    NormalIdx = materials[matIdx].NormalIdx,
                    RoughIdx = materials[matIdx].RoughIdx,
                };

                if (!MaterialDataEquals(materials[matIdx], newData))
                {
                    materials[matIdx] = newData;
                    changed = true;
                }
                matIdx++;
            }
        }

        if (changed)
            SetBuffer(ref MaterialBuffer, materials, MaterialData.TypeSize);

        return changed;
    }

    private static bool MaterialDataEquals(MaterialData a, MaterialData b)
    {
        return a.Color == b.Color &&
               a.Emission == b.Emission &&
               a.EmissionIntensity == b.EmissionIntensity &&
               a.Metallic == b.Metallic &&
               a.Smoothness == b.Smoothness &&
               a.IOR == b.IOR &&
               a.RenderMode == b.RenderMode &&
               a.AlbedoIdx == b.AlbedoIdx &&
               a.EmitIdx == b.EmitIdx &&
               a.MetallicIdx == b.MetallicIdx &&
               a.NormalIdx == b.NormalIdx &&
               a.RoughIdx == b.RoughIdx;
    }

    public static void Destroy()
    {
        IndexBuffer?.Release(); IndexBuffer = null;
        VertexBuffer?.Release(); VertexBuffer = null;
        TriangleBuffer?.Release(); TriangleBuffer = null;
        NormalBuffer?.Release(); NormalBuffer = null;
        TangentBuffer?.Release(); TangentBuffer = null;
        UVBuffer?.Release(); UVBuffer = null;
        MaterialBuffer?.Release(); MaterialBuffer = null;
        MeshNodeBuffer?.Release(); MeshNodeBuffer = null;
        BLASBuffer?.Release(); BLASBuffer = null;
        TransformBuffer?.Release(); TransformBuffer = null;
        if (AlbedoTextures != null) UnityEngine.Object.Destroy(AlbedoTextures);
        AlbedoTextures = null;
        if (EmissionTextures != null) UnityEngine.Object.Destroy(EmissionTextures);
        EmissionTextures = null;
        if (MetallicTextures != null) UnityEngine.Object.Destroy(MetallicTextures);
        MetallicTextures = null;
        if (NormalTextures != null) UnityEngine.Object.Destroy(NormalTextures);
        NormalTextures = null;
        if (RoughnessTextures != null) UnityEngine.Object.Destroy(RoughnessTextures);
        RoughnessTextures = null;
        // Registered scene objects survive a renderer disable/enable cycle.
        objectUpdated = true;
        objectTransformUpdated = true;
    }

    public static List<Matrix4x4> GetTransforms()
    {
        return transforms;
    }
}
