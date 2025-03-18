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
    private static List<TLASNode> tnodes = new List<TLASNode>();
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
    public static ComputeBuffer TLASBuffer;
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
        // check if object data is updated
        foreach (GameObject obj in objects)
        {
            if (obj.transform.hasChanged)
            {
                objectTransformUpdated = true;
                obj.transform.hasChanged = false;
                break;
            }
            if (obj.transform.parent.transform.hasChanged)
            {
                objectTransformUpdated = true;
                obj.transform.parent.transform.hasChanged = false;
                break;
            }
        }

        return BuildBVH() || LoadTransforms();
    }

    private static void BuildMaterialAndMeshData(List<GameObject> objects)
    {
        materials.Clear();
        List<Texture2D> albedoTex = new List<Texture2D>();
        List<Texture2D> emitTex = new List<Texture2D>();
        List<Texture2D> metalTex = new List<Texture2D>();
        List<Texture2D> normTex = new List<Texture2D>();
        List<Texture2D> roughTex = new List<Texture2D>();

        materials.Add(new MaterialData()
        {
            Color = new Vector4(1.0f, 1.0f, 1.0f, 1.0f), // white color by default
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

        foreach (var obj in objects)
        {
            Renderer renderer = obj.GetComponent<Renderer>();
            var mats = renderer.sharedMaterials;
            int matStart = materials.Count;
            int matCount = mats.Length;
            foreach (var mat in mats)
            {
                int albedoIdx = -1, emitIdx = -1, metallicIdx = -1, normalIdx = -1, roughIdx = -1;
                if (mat.HasProperty("_MainTex"))
                {
                    albedoIdx = albedoTex.IndexOf((Texture2D)mat.mainTexture);
                    if (albedoIdx < 0 && mat.mainTexture != null)
                    {
                        albedoTex.Add((Texture2D)mat.mainTexture);
                        albedoIdx = albedoTex.Count - 1;
                    }
                }

                if (mat.HasProperty("_EmissionMap"))
                {
                    var emitMap = mat.GetTexture("_EmissionMap");
                    emitIdx = emitTex.IndexOf(emitMap as Texture2D);
                    if (emitIdx < 0 && emitMap != null)
                    {
                        emitIdx = emitTex.Count;
                        emitTex.Add(emitMap as Texture2D);
                    }
                }

                if (mat.HasProperty("_MetallicGlossMap"))
                {
                    var metalMap = mat.GetTexture("_MetallicGlossMap");
                    metallicIdx = metalTex.IndexOf(metalMap as Texture2D);
                    if (metallicIdx < 0 && metalMap != null)
                    {
                        metallicIdx = metalTex.Count;
                        metalTex.Add(metalMap as Texture2D);
                    }
                }

                if (mat.HasProperty("_BumpMap"))
                {
                    var normMap = mat.GetTexture("_BumpMap");
                    normalIdx = normTex.IndexOf(normMap as Texture2D);
                    if (normalIdx < 0 && normMap != null)
                    {
                        normalIdx = normTex.Count;
                        normTex.Add(normMap as Texture2D);
                    }
                }

                if (mat.HasProperty("_SpecGlossMap"))
                {
                    var roughMap = mat.GetTexture("_SpecGlossMap"); // assume Autodesk interactive shader
                    roughIdx = roughTex.IndexOf(roughMap as Texture2D);
                    if (roughIdx < 0 && roughMap != null)
                    {
                        roughIdx = roughTex.Count;
                        roughTex.Add(roughMap as Texture2D);
                    }
                }

                Color emission = Color.black;
                if (mat.IsKeywordEnabled("_EMISSION"))
                {
                    emission = mat.GetColor("_EmissionColor");
                }

                materials.Add(new MaterialData()
                {
                    Color = new Vector4(mat.color.r, mat.color.g, mat.color.b, mat.color.a),
                    Emission = new Vector3(emission.r, emission.g, emission.b),
                    Metallic = mat.GetFloat("_Metallic"),
                    Smoothness = mat.GetFloat("_Glossiness"),
                    IOR = mat.HasProperty("_IOR") ? mat.GetFloat("_IOR") : 1.0f,
                    RenderMode = mat.HasProperty("_Mode") ? mat.GetFloat("_Mode") : 0.0f,
                    AlbedoIdx = albedoIdx,
                    EmitIdx = emitIdx,
                    MetallicIdx = metallicIdx,
                    NormalIdx = normalIdx,
                    RoughIdx = roughIdx
                });
            }

            Mesh mesh = obj.GetComponent<MeshFilter>().sharedMesh;
            var meshVertices = mesh.vertices.ToList();
            var meshNormals = mesh.normals;
            var meshUVs = mesh.uv;
            var meshTangents = mesh.tangents;
            int vertexStart = vertices.Count;
            int objectIdx = objects.IndexOf(obj);
            for (int i = 0; i < mesh.subMeshCount; i++)
            {
                var subMeshIndices = mesh.GetIndices(i).ToList();
                //TODO:
                //这里的build是build了BVHNode，这个BVH和下面的bnodes的区别是什么？
                // BVH blasTree = new BVH(meshVertices, subMeshIndices);   //这个对象创建之后就没有用了，数据存储在下面的ref的参数里
                BVH blasTree = BVH.Construct(meshVertices, subMeshIndices, BVHType.SAH);
                blasTree.FlattenBLAS(ref indices, ref bnodes, ref meshNodes, subMeshIndices, vertexStart, i < matCount ? i + matStart : 0, objectIdx);
            }
            vertices.AddRange(meshVertices);
            normals.AddRange(meshNormals);
            uvs.AddRange(meshUVs);
            tangents.AddRange(meshTangents);
            
        }
        // if not UV is used, insert empty one
        if (uvs.Count <= 0) uvs.Add(Vector2.zero);
        
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
        RebuildTLAS();

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


    public static void RebuildTLAS()
    {
        if (meshNodes.Count <= 0) return;
        if (transforms.Count <= 0) LoadTransforms();
        tnodes.Clear();
        // BVH tlasTree = new BVH(meshNodes, transforms);
        BVH tlasTree = BVH.Construct(meshNodes, transforms, BVHType.SAH);
        tlasTree.FlattenTLAS(ref meshNodes, ref tnodes);
        SetBuffer(ref TLASBuffer, tnodes, TLASNode.TypeSize);
        SetBuffer(ref MeshNodeBuffer, meshNodes, MeshNode.TypeSize);
    }

    private static void SetBuffer<T>(ref ComputeBuffer buffer, List<T> data, int stride) where T : struct
    {
        if (buffer != null) buffer.Release();
        if (data.Count == 0) return;
        buffer = new ComputeBuffer(data.Count, stride);
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
        if (TLASBuffer != null) TLASBuffer.Release();
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
        int texWidth = 1, texHeight = 1;
        foreach (Texture tex in textures)
        {
            texWidth = Mathf.Max(texWidth, tex.width);
            texHeight = Mathf.Max(texHeight, tex.height);
        }
        int maxDim = GetMaxDimension(textures.Count, Mathf.Max(texWidth, texHeight));
        texWidth = Mathf.Min(texWidth, maxDim);
        texHeight = Mathf.Min(texHeight, maxDim);
        var newTexture = new Texture2DArray(
            texWidth, texHeight, Mathf.Max(1, textures.Count),
            TextureFormat.ARGB32, true, false
        );
        newTexture.SetPixels(Enumerable.Repeat(Color.white, texWidth * texHeight).ToArray(), 0, 0);
        RenderTexture rt = new RenderTexture(texWidth, texHeight, 1, RenderTextureFormat.ARGB32);
        Texture2D tmp = new Texture2D(texWidth, texHeight, TextureFormat.ARGB32, false);
        for (int i = 0; i < textures.Count; i++)
        {
            RenderTexture.active = rt;
            Graphics.Blit(textures[i], rt);
            tmp.ReadPixels(new Rect(0, 0, texWidth, texHeight), 0, 0);
            tmp.Apply();
            newTexture.SetPixels(tmp.GetPixels(0), i, 0);
        }
        newTexture.Apply();
        RenderTexture.active = null;
        UnityEngine.Object.Destroy(rt);
        UnityEngine.Object.Destroy(tmp);
        return newTexture;
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
