Shader "LeosPostShaders/DepthNormalsToGreyscale"
{
    Properties
    {
		_MainTex("Texture", 2D) = "white" {}
		_Direction("Direction", Vector) = (1,0,0,0)
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

            sampler2D _MainTex;
			sampler2D _CameraDepthNormalsTexture, _CameraDepthTexture;
			float4 _Direction;

			fixed4 frag(v2f i) : SV_Target
			{
				half4 color = tex2D(_MainTex, i.uv);
				half3 normal;
				float depth;

				DecodeDepthNormal(tex2D(_CameraDepthNormalsTexture, i.uv), depth, normal);
				//normal = mul((float3x3)unity_CameraToWorld, normal);
				//normal = (normal + 1)/2;
				//normal = normalize(normal);
				
				normal = (normal + 1) / 2;

				float val = normal.x * _Direction.x + normal.y * _Direction.y + normal.z * _Direction.z;

				if (Linear01Depth(depth) >= 1) { return 0; }

				val /= length(_Direction);
				val *= color.a;

				fixed4 result = fixed4(val, val, val, color.a);

				return result;
            }
            ENDCG
        }
    }
}
