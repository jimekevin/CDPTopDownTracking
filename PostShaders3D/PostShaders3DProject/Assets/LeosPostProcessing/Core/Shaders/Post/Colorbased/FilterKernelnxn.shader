Shader "LeosPostShaders/FilterKernelnxn"
{
	Properties
	{
		_MainTex("Main Texture", 2D) = "white" {}
		_KernelTex("Kernel Texture", 2D) = "white" {}
		_Color("Tint", Color) = (1,1,1,1)
		_Fade("Fade", Range(0.0,1)) = 1
		_Aspect("Aspect", float) = 1.7777777
		_Distance("Distance", Range(0.0,0.1)) = 0.01
		_Median("Median", Float) = 1
		_KernelSize("Kernel Size", Int) = 3
		_KernelWeight("Kernel Weight", Float) = 4
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

				fixed4 _Color;

				v2f vert(appdata_t IN)
				{
					v2f OUT;
					OUT.vertex = UnityObjectToClipPos(IN.vertex);
					OUT.texcoord = IN.texcoord;
					OUT.color = IN.color * _Color;

					return OUT;
				}

				sampler2D _MainTex, _KernelTex;
				float _Fade, _Distance, _Median, _Aspect, _KernelWeight;
				float4 _Row0;
				float4 _Row1;
				float4 _Row2;
				uint _KernelSize;

				fixed4 frag(v2f IN) : SV_Target
				{
					float2 uv = IN.texcoord;
					fixed4 midColor = tex2D(_MainTex, uv);

					float4 sum = 0;
					int offset = _KernelSize / -2;
					for (int x = -offset; x <= offset; x++) {
						for (int y = -offset; y <= offset; y++) {
							float2 cuv = uv + float2(_Aspect * x, y) * _Distance;
							float4 val = tex2D(_MainTex, cuv);
							val *= (tex2D(_KernelTex, float2(x + offset, y + offset)).r - 0.5) * 2 * _KernelWeight;
							sum += val;
						}
					}

					float weightSum = _Median;
					if (!weightSum) { weightSum = 1; }

					fixed4 color = sum / weightSum;
					color = lerp(midColor, color, _Fade);

					color.rgb *= midColor.a;
					color.a = midColor.a;

					return color;
				}
			ENDCG
			}
		}
}