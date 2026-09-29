using System.Collections.Generic;
using System.Linq;
using UnityEngine;

internal static class BVHTextureArrays
{
    public static Texture2DArray Create(List<Texture2D> textures, bool linear)
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
            TextureFormat.ARGB32, true, linear
        );
        newTexture.SetPixels(Enumerable.Repeat(Color.white, texWidth * texHeight).ToArray(), 0, 0);
        RenderTextureReadWrite readWrite = linear ? RenderTextureReadWrite.Linear : RenderTextureReadWrite.sRGB;
        RenderTexture rt = new RenderTexture(texWidth, texHeight, 1, RenderTextureFormat.ARGB32, readWrite);
        Texture2D tmp = new Texture2D(texWidth, texHeight, TextureFormat.ARGB32, false, linear);
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
}
