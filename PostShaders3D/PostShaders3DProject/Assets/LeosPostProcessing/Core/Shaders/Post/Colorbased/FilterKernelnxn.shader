Shader "LeosPostShaders/FilterKernelnxn"
{
	Properties
	{
		_MainTex("Main Texture", 2D) = "white" {}
		_KernelTex("Kernel Texture", 2D) = "white" {}
		_Fade("Fade", Range(0.0,1)) = 1
		_Aspect("Aspect", float) = 1.7777777
		_Distance("Distance", Range(0.0,0.1)) = 0.01
		_KernelSize("Kernel Size", Range(1, 9)) = 3
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

				v2f vert(appdata_t IN)
				{
					v2f OUT;
					OUT.vertex = UnityObjectToClipPos(IN.vertex);
					OUT.texcoord = IN.texcoord;
					OUT.color = IN.color;

					return OUT;
				}

				sampler2D _MainTex, _KernelTex;
				float _Fade, _Distance, _Aspect, _KernelWeight;
				uint _KernelSize;

				fixed4 frag(v2f IN) : SV_Target
				{
					float2 uv = IN.texcoord;

					float4 sum = float4(0,0,0,0);
					int offset = _KernelSize / 2;
					for (int x = -offset; x <= offset; x++) {
						for (int y = -offset; y <= offset; y++) {
							float2 cuv = uv + float2(x, _Aspect * y) * _Distance;
							float4 val = tex2D(_MainTex, cuv);
							float2 kernelUV = (float2(x, y) + offset + 0.5) / (offset * 2 + 1);
							//val *= (tex2D(_KernelTex, kernelUV).r) * _KernelWeight;
							val *= (tex2D(_KernelTex, kernelUV).r - 0.5) * 2 * _KernelWeight;
							sum += val;
						}
					}

					fixed4 color = sum;
					fixed4 midColor = tex2D(_MainTex, uv);
					color = lerp(midColor, color, _Fade);

					color.rgb *= midColor.a;
					color.a = midColor.a;

					return color;
				}
			ENDCG
			}
		}
}