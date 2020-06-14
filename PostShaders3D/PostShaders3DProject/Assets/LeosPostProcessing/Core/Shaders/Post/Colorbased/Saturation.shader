Shader "LeosPostShaders/Saturation"
{
	Properties
	{
		_MainTex("Main Texture", 2D) = "white" {}
		_Color("Tint", Color) = (1,1,1,1)
		_Saturation("Saturation", Range(0, 1)) = 1.0
		[MaterialToggle] PixelSnap("Pixel snap", Float) = 0
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

				v2f vert(appdata v)
				{
					v2f o;
					o.vertex = UnityObjectToClipPos(v.vertex);
					o.uv = v.uv;
					return o;
				}

				sampler2D _MainTex;
				float _Saturation;

				fixed4 frag(v2f i) : SV_Target
				{
					fixed4 color = tex2D(_MainTex, i.uv);

					fixed4 ma = max(color.r, color.g);
					ma = max(ma, color.b);
					fixed4 mi = min(color.r, color.g);
					mi = min(mi, color.b);

					fixed g = 0.5 * (ma + mi);
					fixed4 greycolor = fixed4(g, g, g, color.a);

					color = lerp(greycolor, color, _Saturation);

					color.rgb *= color.a;
					return color;
				}
			ENDCG
			}
		}
}