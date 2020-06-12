Shader "LeosPostShaders/TresholdHalftone"
{
    Properties
    {
		_MainTex("Main Texture", 2D) = "white" {}
		_BackTex("Back Texture", 2D) = "white" {}
		_RasterTex("Raster Texture", 2D) = "white" {}
		_Color("Bright Color", Color) = (1,1,1,1)
		_RasterSize("Raster Size", Range(0.001,1)) = 0.02
		_ColorFade("Color Fade", Range(0,1)) = 0.5
		_Intensity("Intensity Factor", Range(0,1)) = 0.75
		_Aspect("Aspect Ratio", Float) = 1.78
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

			sampler2D _MainTex, _RasterTex, _BackTex;
			half4 _Color;
			float _Aspect, _RasterSize, _ColorFade, _Intensity;

            fixed4 frag (v2f i) : SV_Target
            {
				fixed4 color = tex2D(_MainTex, i.uv);

				fixed4 ma = max(color.r, color.g);
				ma = max(ma, color.b);
				fixed4 mi = min(color.r, color.g);
				mi = min(mi, color.b);
				fixed intensity = 0.5 * (ma + mi);

				//intensity = (intensity - 0.5 + _Mid) * _Spread;
				//intensity = saturate(intensity);
				
				fixed4 result = fixed4(0, 0, 0, 0);
				float2 uv = float2(fmod(i.uv.x, _RasterSize) / _RasterSize, fmod(i.uv.y, _RasterSize * _Aspect) / (_RasterSize * _Aspect));
				float rasterIntensity = tex2D(_RasterTex, uv).r * _Intensity;
				float factor = 0;

				if (intensity >= 1 - rasterIntensity) { factor = 1; }

				result += fixed4(1, 1, 1, 1) * factor;
				result = lerp(result, color, _ColorFade * factor) * _Color;

				result.rgb *= color.a;
				result.a = color.a;

				//return rasterIntensity;
				//return (uv.x + uv.y) / 2;

				return result;
            }
            ENDCG
        }
    }
}
