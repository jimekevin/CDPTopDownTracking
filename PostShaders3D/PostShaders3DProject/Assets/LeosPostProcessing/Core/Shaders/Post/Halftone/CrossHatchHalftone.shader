Shader "LeosPostShaders/CrossHatchHalftone"
{
    Properties
    {
		_MainTex("Main Texture", 2D) = "white" {}
		_RasterTexBright("Raster Texture Bright", 2D) = "white" {}
		_RasterTexDark("Raster Texture Dark", 2D) = "white" {}
		_BrightColor("Bright Color", Color) = (1,1,1,1)
		_DarkColor("Dark Color", Color) = (0,0,0,0)
		_RasterSize("Raster Size", Range(0.001, 0.2)) = 0.02
		_ColorFade("Color Fade", Range(0,1)) = 0.5
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

			sampler2D _MainTex, _RasterTexBright, _RasterTexDark, _CameraDepthNormalsTexture;
			half4 _BrightColor, _DarkColor;
			float _Aspect, _RasterSize, _ColorFade;

            fixed4 frag (v2f i) : SV_Target
            {
				fixed4 color = tex2D(_MainTex, i.uv);

				fixed4 ma = max(color.r, color.g);
				ma = max(ma, color.b);
				fixed4 mi = min(color.r, color.g);
				mi = min(mi, color.b);
				fixed intensity = 0.5 * (ma + mi);
				
                float2 uv = i.uv;
                uv = float2(fmod(uv.x, _RasterSize) / _RasterSize, fmod(uv.y, _RasterSize * _Aspect) / (_RasterSize * _Aspect));
				float rasterIntensityBright = tex2D(_RasterTexBright, uv).r;
				float rasterIntensityDark = tex2D(_RasterTexDark, uv).r;

				float factor = 1;
				if (intensity <= 1 - rasterIntensityBright || intensity * 2 <= 1 - rasterIntensityDark) { factor = 0; }

				fixed4 result = lerp(_DarkColor, _BrightColor, factor);
				result = lerp(result, color, _ColorFade * factor);

				result.rgb *= color.a * result.a;
				result.a *= color.a;

				return result;
            }
            ENDCG
        }
    }
}
