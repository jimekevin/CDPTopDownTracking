Shader "LeosPostShaders/DottedHalftone"
{
    Properties
    {
		_MainTex("Main Texture", 2D) = "white" {}
		_BrightColor("Bright Color", Color) = (1,1,1,1)
		_DarkColor("Dark Color", Color) = (0,0,0,0)
		_RasterSize("Raster Size", Range(0.001,1)) = 0.02
		_ColorFade("Color Fade", Range(0,1)) = 0.5
		_Mid("Mid Intensity", Range(0,1)) = 0.5
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

			sampler2D _MainTex;
			half4 _BrightColor, _DarkColor;
			float _Aspect, _RasterSize, _ColorFade, _Mid, _Spread, _SoftFactor;

            fixed4 frag (v2f i) : SV_Target
            {
				fixed4 color = tex2D(_MainTex, i.uv);
				float intensity = color.r;
				if (color.g > intensity) { intensity = color.g; }
				if (color.b > intensity) { intensity = color.b; }

				intensity = (intensity - 0.5 + _Mid) * _Spread;
				intensity = saturate(intensity);

				float radius = 0.7071 * intensity;
				
				float2 uv = float2(fmod(i.uv.x, _RasterSize) / _RasterSize, fmod(i.uv.y, _RasterSize * _Aspect) / (_RasterSize * _Aspect));
				uv = uv - float2(0.5, 0.5);
				float factor = 0;

				factor = saturate(1 - (length(uv) - radius) / _SoftFactor);

				if (radius < 0.1f) { factor = 0; }

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
