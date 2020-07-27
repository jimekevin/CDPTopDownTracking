Shader "LeosPostShaders/Canny"
{
	Properties
	{
		_MainTex("Main Texture", 2D) = "white" {}
		_HorizontalTex("Horizontal Texture", 2D) = "white" {}
		_VerticalTex("Vertical Texture", 2D) = "white" {}
		_Aspect("Aspect", float) = 1.7777777
		_Distance("Pixel Distance in UV", Range(0.0,0.1)) = 0.01
	}

		SubShader
		{

		Cull Off ZWrite Off ZTest Always

			Pass
			{
			CGPROGRAM
				#pragma vertex vert
				#pragma fragment frag
				#include "UnityCG.cginc"

				struct appdata_t
				{
					float4 vertex   : POSITION;
					float4 color    : COLOR;
					float2 texcoord : TEXCOORD0;
				};

				struct v2f
				{
					float4 vertex   : SV_POSITION;
					fixed4 color : COLOR;
					float2 texcoord  : TEXCOORD0;
				};

				v2f vert(appdata_t IN)
				{
					v2f OUT;
					OUT.vertex = UnityObjectToClipPos(IN.vertex);
					OUT.texcoord = IN.texcoord;
					OUT.color = IN.color;

					return OUT;
				}

				sampler2D _MainTex, _HorizontalTex, _VerticalTex;
				float _Distance, _Aspect;

				fixed4 frag(v2f IN) : SV_Target
				{
					float2 uv = IN.texcoord;
					fixed4 midColor = tex2D(_MainTex, uv);
					fixed4 horColor = tex2D(_HorizontalTex, uv);
					fixed4 verColor = tex2D(_VerticalTex, uv);

					//Angle by gradient
					float4 offsets;
					if (abs(length(horColor) - length(verColor)) > 0.5)
					{
						if (length(horColor) > length(verColor))
						{
							offsets = float4(float2(-1, 0), float2(1, 0));
						}
						else
						{
							offsets = float4(float2(1, 0), float2(-1, 0));
						}
					}
					else
					{
						if (length(horColor) > length(verColor))
						{
							offsets = float4(float2(-1, -1), float2(1, 1));
						}
						else
						{
							offsets = float4(float2(-1, 1), float2(1, -1));
						}
					}

					//NON Maximum Suppression
					fixed4 color = midColor;
					bool changed = false;
					{
						float2 cuv = uv + float2(offsets.x, _Aspect * offsets.y) * _Distance;
						float4 val = tex2D(_MainTex, cuv);
						if (length(val) > length(color)) { changed = true; }
					}
					{
						float2 cuv = uv + float2(offsets.z, _Aspect * offsets.w) * _Distance;
						float4 val = tex2D(_MainTex, cuv);
						if (length(val) > length(color)) { changed = true; }
					}
					if (changed) {
						color = fixed4(0,0,0,1);
					}

					//alpha
					color.rgb *= midColor.a;
					color.a = midColor.a;

					return color;
				}
			ENDCG
			}
		}
}