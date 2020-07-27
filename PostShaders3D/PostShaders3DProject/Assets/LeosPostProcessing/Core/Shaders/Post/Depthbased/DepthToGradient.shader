Shader "LeosPostShaders/DepthToGradient"
{
    Properties
    {
		_GradientTex("Gradient Texture", 2D) = "white" {}
		_BackColor("BackGround Color", Color) = (1,1,1,1)
		_DistanceFactor("Distance Factor", float) = 10
    }
    SubShader
    {
        // No culling or depth
        Cull Off ZWrite Off ZTest Always

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
            };

            struct v2f
            {
                float2 uv : TEXCOORD0;
                float4 vertex : SV_POSITION;
            };

            v2f vert (appdata v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.uv = v.uv;
                return o;
            }

			sampler2D _CameraDepthTexture, _GradientTex;
			half4 _BackColor;
			float _DistanceFactor;

            fixed4 frag (v2f i) : SV_Target
            {
				float depth = tex2D(_CameraDepthTexture, i.uv).r;

				depth = Linear01Depth(depth);

				if (depth >= 1)
				{
					return _BackColor;
				}

				depth *= _DistanceFactor;

				fixed4 result = tex2D(_GradientTex, float2(depth, depth));
				result.rgb *= result.a;

				return result;
            }
            ENDCG
        }
    }
}
