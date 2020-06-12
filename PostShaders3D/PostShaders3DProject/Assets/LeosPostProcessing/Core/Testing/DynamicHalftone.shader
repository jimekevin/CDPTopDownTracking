Shader "LeosPostShaders/DynamicHalftone"
{
    Properties
    {
		_MainTex("Main Texture", 2D) = "white" {}
		_RasterTex("Raster Texture", 2D) = "white" {}
		_BrightColor("Bright Color", Color) = (1,1,1,1)
		_RasterSize("Raster Size", Range(0.001,1)) = 0.02
		_ColorFade("Color Fade", Range(0,1)) = 0.5
		_Mid("Mid Intensity", Range(0,1)) = 0.5
		_HighTreshold("High Treshold", Range(0,1)) = 0.75
		_LowTreshold("Low Treshold", Range(0,1)) = 0.25
		_Spread("Intensity Spread", Range(0,2)) = 1
		_SoftFactor("Soft Factor", Range(0.01, 1)) = 0.1
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

			sampler2D _MainTex, _RasterTex;
			half4 _BrightColor;
			float _Aspect, _RasterSize, _ColorFade, _Mid, _Spread, _SoftFactor, _LowTreshold, _HighTreshold;

            fixed4 frag (v2f i) : SV_Target
            {
				fixed4 color = tex2D(_MainTex, i.uv);
				float intensity = color.r;
				if (color.g > intensity) { intensity = color.g; }
				if (color.b > intensity) { intensity = color.b; }

				//intensity = (intensity - 0.5 + _Mid) * _Spread;
				//intensity = saturate(intensity);
				
				fixed4 result = fixed4(0, 0, 0, 0);
				float2 uv = float2(fmod(i.uv.x, _RasterSize) / _RasterSize, fmod(i.uv.y, _RasterSize * _Aspect) / (_RasterSize * _Aspect));
				float rasterIntensity = tex2D(_RasterTex, uv).r;
				float factor = 0;

				//rasterIntensity = (rasterIntensity - 0.5 + _Mid) * _Spread;
				//rasterIntensity *= _Spread;
				//rasterIntensity = saturate(rasterIntensity);

				intensity = saturate((intensity - _LowTreshold) / (_HighTreshold - _LowTreshold));

				factor = saturate((intensity - (1 - rasterIntensity)) / _SoftFactor);

				//if (rasterIntensity < 0.05f) { factor = 0; }

				result += _BrightColor * factor;
				result = lerp(result, color, _ColorFade * factor);

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
