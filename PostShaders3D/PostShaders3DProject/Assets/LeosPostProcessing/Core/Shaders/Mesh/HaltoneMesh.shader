Shader "LeosPostShaders/HalftoneMesh" {
    Properties{
        _MainTex("Texture", 2D) = "white" {}
		_Color("Color", Color) = (1,1,1,1)
		_BrightColor("Bright Color", Color) = (1,1,1,1)
		_DarkColor("Dark Color", Color) = (0,0,0,1)
        _RasterTex("Raster Texture", 2D) = "Black" {}
		_Intensity("Intensity", Range(0,4)) = 1
		_SmoothBorder("Smooth (0 or 1)", Range(0,1)) = 1
    }
        SubShader{
        Tags { "RenderType" = "Opaque" }
        CGPROGRAM
          #pragma surface surf Halftone
        
            struct SurfaceOutputHalftone
            {
                fixed3 Albedo;  // diffuse color
                fixed3 Normal;  // tangent space normal, if written
                fixed3 Emission;
                half Specular;  // specular power in 0..1 range
                fixed Gloss;    // specular intensity
                fixed Alpha;    // alpha for transparencies
                float2 uv;
            };

        sampler2D _RasterTex;
        float4 _RasterTex_ST;
		float4 _BrightColor, _DarkColor;
        float _Intensity;
		int _SmoothBorder;

          half4 LightingHalftone(SurfaceOutputHalftone s, half3 lightDir, half atten) {
              half NdotL = dot(s.Normal, lightDir);
              
              float2 uv = s.uv;
              float2 rasterSize = 1 / _RasterTex_ST.xy;

              uv = float2(fmod(uv.x, rasterSize.x) / rasterSize.x, fmod(uv.y, rasterSize.y) / rasterSize.y);
              float rasterIntensity = tex2D(_RasterTex, uv).r * _Intensity;
              float factor = 0;

			  if (_SmoothBorder == 1) {
				  if (NdotL >= 1 - rasterIntensity * 1.5) { factor = 0.5; }
				  if (NdotL >= 1 - rasterIntensity * 1.25) { factor = 0.75; }
				  if (NdotL >= 1 - rasterIntensity * 1.0) { factor = 1; }
				  if (NdotL >= 1 - rasterIntensity * 0.75) { factor = 1.25; }
				  if (NdotL >= 1 - rasterIntensity * 0.5) { factor = 1.5; }
			  }
			  else {
				  if (NdotL >= 1 - rasterIntensity) { factor = 1; }
			  }
			  

			  half4 result;
			  result.rgb = lerp(_DarkColor.rgb, _BrightColor.rgb * _LightColor0.rgb, factor);
			  result.a = s.Alpha;

              return result;
          }

        struct Input {
            float2 uv_MainTex;
        };

        sampler2D _MainTex;
        float4 _Color;

        void surf(Input IN, inout SurfaceOutputHalftone o) {
            o.Albedo = tex2D(_MainTex, IN.uv_MainTex).rgb * _Color;
            o.uv = IN.uv_MainTex;
        }
        ENDCG
    }
        Fallback "Diffuse"
}