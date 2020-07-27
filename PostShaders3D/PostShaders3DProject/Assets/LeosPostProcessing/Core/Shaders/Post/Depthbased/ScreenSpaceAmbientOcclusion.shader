Shader "LeosPostShaders/ScreenSpaceAmbientOcclusion"
{
    Properties
    {
		_MainTex("Texture", 2D) = "white" {}
		_RandomTex("Random Texture", 2D) = "white" {}
		_Aspect("Aspect", float) = 1.7777777
		_Distance("Radius", float) = 0.01
		_Intensity("Intensity", float) = 1
		_Factor("Factor", float) = 1
		_Factor2("Factor2", float) = 1
		_Treshold("Treshold", float) = 1
		_Samples("Sample", Range(1, 5)) = 2
		_UVtoView("UV to View Space", Vector) = (0,0,0,0)
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

            sampler2D _MainTex, _RandomTex;
			float4 _RandomTex_ST, _UVtoView;
			sampler2D _CameraDepthTexture, _CameraDepthNormalsTexture;
			float _Distance, _Aspect, _Intensity, _Factor, _Factor2, _Treshold;
			int _Samples;
			half4x4 _ClipToCamSpace, _CamToClipSpace;

			float3 positionFromUV(float2 uv) //position in clip space
			{
				float depth = tex2D(_CameraDepthTexture, uv).r;
				depth = Linear01Depth(depth);
				//uv = uv * 2 - 1;
				//float3 result = float3(uv * _UVtoView.xy + _UVtoView.zw, depth * _Factor2);
				//float3 result = float3(uv.x * _Aspect, uv.y, depth * _Factor2);
				float3 result = float3(uv.xy, depth * _Factor2);
				//float3 result = float3(uv, depth * _Factor2);
				//result = mul(_ClipToCamSpace, float4(result, 1)).xyz;
				return result;
			}

			float getAOvalue(float2 centerUV, float2 offsetUV, float3 pos, float3 normal)
			{
				//offsetUV = float2(offsetUV.x, offsetUV.y);
				offsetUV = float2(offsetUV.x, offsetUV.y * _Aspect);
				float3 distanceVector = positionFromUV(centerUV + offsetUV) - pos;
				return max(0, (dot(normal, normalize(distanceVector)) - _Treshold)) / (1.0 + length(distanceVector) * _Factor);
				//return abs(dot(normal, normalize(distanceVector)) - _Treshold) / (1.0 + length(distanceVector) * _Factor);
			}

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
				float sign = 1;
				if ((offsetUV.x > offsetUV.y && offsetUV.x > 0 == a < 0) || (offsetUV.x > offsetUV.y && offsetUV.y > 0 == a < 0)) {
					//sign = -1;
				}

				return value * a * sign;
			}

			fixed4 frag(v2f i) : SV_Target
			{
				half4 color = tex2D(_MainTex, i.uv);

				float depth = tex2D(_CameraDepthTexture, i.uv).r;
				depth = Linear01Depth(depth);
				if (depth >= 1) { return 0; }

				float3 pos = positionFromUV(i.uv);
				float rad = _Distance / depth * 0.0001;

				float3 normal;
				DecodeDepthNormal(tex2D(_CameraDepthNormalsTexture, i.uv), depth, normal);
				//normal = normal * 2 - 1;
				//normal = float3(normal.xy, -normal.z);
				//normal = mul(_CamToClipSpace, float4(normal, 1)).xyz;
				normal = normalize(normal);

				float occlusion = 0;
				[unroll(5)]
				for (int s = 0; s < _Samples; s++)
				{
					float2 rand = tex2D(_RandomTex, i.uv * _RandomTex_ST.xy + _RandomTex_ST.zw);
					[unroll(4)]
					for (int x = 0; x < 4; x++)
					{
						float2 offset = float2(-1, 0);
						if (x == 1) { offset = float2(1, 0); }
						if (x == 2) { offset = float2(0, -1); }
						if (x == 3) { offset = float2(0, 1); }

						//float2 offset1 = reflect(offset, rand) * rad;
						float2 offset1 = offset * rad;
						float2 offset2 = float2(offset1.x * 0.707 - offset1.y * 0.707, offset1.x * 0.707 + offset1.y * 0.707);
						//float2 offset2 = offset1;

						//occlusion += getAOvalue(i.uv, offset1 * 0.25, pos, normal);
						//occlusion += getAOvalue(i.uv, offset2 * 0.5, pos, normal);
						//occlusion += getAOvalue(i.uv, offset1 * 0.75, pos, normal);
						//occlusion += getAOvalue(i.uv, offset2, pos, normal);
						occlusion += getAOvalueByNormal(i.uv, offset1 * (0.25 + s), depth, normal);
						occlusion += getAOvalueByNormal(i.uv, offset2 * (0.5 + s), depth, normal);
						occlusion += getAOvalueByNormal(i.uv, offset1 * (0.75 + s), depth, normal);
						occlusion += getAOvalueByNormal(i.uv, offset2 * (1 + s), depth, normal);
					}
				}
				
				occlusion *= _Intensity / (4 * _Samples);

				return fixed4(occlusion, occlusion, occlusion, 1);
            }
            ENDCG
        }
    }
}
