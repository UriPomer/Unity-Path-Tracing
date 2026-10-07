Shader "Hidden/PathTracing/TextureArrayNormal"
{
    Properties { _MainTex ("Normal map", 2D) = "bump" {} }
    SubShader
    {
        Cull Off ZWrite Off ZTest Always
        Pass
        {
            CGPROGRAM
            #pragma vertex vert_img
            #pragma fragment frag
            #include "UnityCG.cginc"

            sampler2D _MainTex;
            float4 frag(v2f_img input) : SV_Target
            {
                return float4(UnpackNormal(tex2D(_MainTex, input.uv)) * 0.5 + 0.5, 1);
            }
            ENDCG
        }
    }
}
