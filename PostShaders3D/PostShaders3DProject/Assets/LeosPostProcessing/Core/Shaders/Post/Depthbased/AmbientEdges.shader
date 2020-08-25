Shader "LeosPostShaders/AmbientEdges"
{
    Properties
    {
		_MainTex("Texture", 2D) = "white" {}
		_Aspect("Aspect", float) = 1.7777777
		_Distance("Radius", float) = 0.01
		_Intensity("Intensity", float) = 1
		_Factor("Factor", float) = 1
		_Treshold("Treshold", float) = 1
		_Samples("Sample", Range(1, 5)) = 2
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

			#define pi 3.141592653589793238462
			#define pi2 6.283185307179586476924

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
			sampler2D _CameraDepthTexture, _CameraDepthNormalsTexture;
			float _Distance, _Aspect, _Intensity, _Factor, _Treshold;
			int _Samples;
			half4x4 _ClipToCamSpace, _CamToClipSpace;

			float angle(float3 a, float3 b)
			{
				float c = dot(a, b); //cosine
				float3 cr = cross(a, b);
				float s = asin(length(cr)); //sine
				float ac = acos(c);
				if (c < 0) {
					return -s;
				}
				else {
					return s;
				}
			}

			float getAOvalueByNormal(float2 centerUV, float2 offsetUV, float curDepth, float3 normal)
			{
				offsetUV = float2(offsetUV.x, offsetUV.y * _Aspect);

				float3 otherNormal;
				float depth;
				DecodeDepthNormal(tex2D(_CameraDepthNormalsTexture, centerUV + offsetUV), depth, otherNormal);
				normal = normalize(normal);
				float value = 1 - (dot(normal, otherNormal) - _Treshold);

				if (depth >= 1) { return 1; }
				return value;

				float a = angle(normal, otherNormal) * _Factor;

				return value * a;
			}

			fixed4 frag(v2f i) : SV_Target
			{
				half4 color = tex2D(_MainTex, i.uv);

				float depth = tex2D(_CameraDepthTexture, i.uv).r;
				depth = Linear01Depth(depth);
				if (depth >= 1) { return 0; }

				float rad = _Distance / depth * 0.0001;

				float3 normal;
				DecodeDepthNormal(tex2D(_CameraDepthNormalsTexture, i.uv), depth, normal);
				normal = normalize(normal);

				float occlusion = 0;
				[unroll(5)]
				for (int s = 0; s < _Samples; s++)
				{
					[unroll(4)]
					for (int x = 0; x < 4; x++)
					{
						float2 offset = float2(-1, 0);
						if (x == 1) { offset = float2(1, 0); }
						if (x == 2) { offset = float2(0, -1); }
						if (x == 3) { offset = float2(0, 1); }

						float2 offset1 = offset * rad;
						float2 offset2 = float2(offset1.x * 0.707 - offset1.y * 0.707, offset1.x * 0.707 + offset1.y * 0.707);

						occlusion += getAOvalueByNormal(i.uv, offset1 * (0.25 + s), depth, normal);
						occlusion += getAOvalueByNormal(i.uv, offset2 * (0.5 + s), depth, normal);
						occlusion += getAOvalueByNormal(i.uv, offset1 * (0.75 + s), depth, normal);
						occlusion += getAOvalueByNormal(i.uv, offset2 * (1 + s), depth, normal);
					}
				}
				
				occlusion *= _Intensity / (4 * _Samples);
				occlusion = saturate(occlusion);

				return fixed4(occlusion, occlusion, occlusion, occlusion);
            }
            ENDCG
        }
    }
}
