using System.Collections.Generic;
using UnityEngine;

internal static class BVHTextureArrays
{
    public enum Kind { Color, Data, Normal }

    public static Texture2DArray Create(List<Texture2D> textures, Kind kind)
    {
        int texWidth = 1, texHeight = 1;
        foreach (Texture tex in textures)
        {
            texWidth = Mathf.Max(texWidth, tex.width);
            texHeight = Mathf.Max(texHeight, tex.height);
        }
        var newTexture = new Texture2DArray(
            texWidth, texHeight, Mathf.Max(1, textures.Count),
            kind == Kind.Color ? TextureFormat.RGBAHalf : TextureFormat.ARGB32, true, true
        );
        newTexture.filterMode = FilterMode.Trilinear;
        if (textures.Count == 0)
        {
            newTexture.SetPixels(new[] { Color.white },0,0);
            newTexture.Apply(false);
            return newTexture;
        }
        newTexture.Apply(false, true);
        // Decode imported sRGB in the blit and store/filter linear values.
        // Half precision retains dark-color detail without requiring sRGB mip
        // generation, whose encoded-byte averaging darkened minified textures.
        var rt = new RenderTexture(new RenderTextureDescriptor(texWidth,texHeight) {
            graphicsFormat = newTexture.graphicsFormat, depthBufferBits = 0,
            useMipMap = true, autoGenerateMips = false, msaaSamples = 1
        });
        rt.Create();
        // Normal arrays have one canonical XYZ encoding, including their mips.
        // Unity's platform-dependent RG/AG packing ends at this boundary.
        Material decoder = kind == Kind.Normal ?
            new Material(Resources.Load<Shader>("TextureArrayNormal")) : null;
        RenderTexture previousTarget = RenderTexture.active;
        try
        {
            for (int i = 0; i < textures.Count; i++)
            {
                if (decoder == null) Graphics.Blit(textures[i], rt);
                else Graphics.Blit(textures[i], rt, decoder);
                rt.GenerateMips();
                for (int mip=0;mip<newTexture.mipmapCount;mip++)
                    Graphics.CopyTexture(rt,0,mip,newTexture,i,mip);
            }
        }
        finally
        {
            RenderTexture.active = previousTarget;
            rt.Release();
            UnityEngine.Object.Destroy(rt);
            if (decoder != null) UnityEngine.Object.Destroy(decoder);
        }
        return newTexture;
    }

}
