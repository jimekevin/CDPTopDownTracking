Shader "LeosPostShaders/FilterKernel3x3"
{
	Properties
	{
		_MainTex("Main Texture", 2D) = "white" {}
		_Color("Tint", Color) = (1,1,1,1)
		_Fade("Fade", Range(0.0,1)) = 1
		_Aspect("Aspect", float) = 1.7777777
		_Distance("Distance", Range(0.0,0.1)) = 0.01
		_Median("Median", Float) = 1
		_Row0("Row0", Vector) = (0,1,0,0)
		_Row1("Row1", Vector) = (1,1,1,0)
		_Row2("Row2", Vector) = (0,1,0,0)
		[MaterialToggle] PixelSnap("Pixel snap", Float) = 0
	}

		SubShader
		{

		Cull Off ZWrite Off ZTest Always

			Pass
			{
			CGPROGRAM
				#pragma vertex vert
				#pragma fragment frag
				#pragma multi_compile _ PIXELSNAP_ON
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
					#ifdef PIXELSNAP_ON
					OUT.vertex = UnityPixelSnap(OUT.vertex);
					#endif

					return OUT;
				}

				sampler2D _MainTex;
				sampler2D _AlphaTex;
				float _Fade, _Distance, _Median, _Aspect;
				float4 _Row0;
				float4 _Row1;
				float4 _Row2;

				fixed4 SampleSpriteTexture(float2 uv)
				{
					fixed4 midColor = tex2D(_MainTex, uv);

					fixed4 c00 = tex2D(_MainTex, uv + float2(-_Aspect,-1) * _Distance) * _Row0.x;
					fixed4 c01 = tex2D(_MainTex, uv + float2(0, -1) * _Distance) * _Row0.y;
					fixed4 c02 = tex2D(_MainTex, uv + float2(_Aspect, -1) * _Distance) * _Row0.z;
					fixed4 c10 = tex2D(_MainTex, uv + float2(-_Aspect, 0) * _Distance) * _Row1.x;
					fixed4 c11 = midColor * _Row1.y;
					fixed4 c12 = tex2D(_MainTex, uv + float2(_Aspect, 0) * _Distance) * _Row1.z;
					fixed4 c20 = tex2D(_MainTex, uv + float2(-_Aspect, 1) * _Distance) * _Row2.x;
					fixed4 c21 = tex2D(_MainTex, uv + float2(0, 1) * _Distance) * _Row2.y;
					fixed4 c22 = tex2D(_MainTex, uv + float2(_Aspect, 1) * _Distance) * _Row2.z;

					float4 sum = c00 + c01 + c02 + c10 + c11 + c12 + c20 + c21 + c22;
					//float weightSum = _Row0.x + _Row0.y + _Row0.z + _Row1.x + _Row1.y + _Row1.z + _Row2.x + _Row2.y + _Row2.z;
					float weightSum = _Median;
					if (!weightSum) { weightSum = 1; }

					float4 color = sum/weightSum;
					color = lerp(midColor, color, _Fade);

					color.rgb *= midColor.a;
					color.a = midColor.a;

					return color;
				}

				fixed4 frag(v2f IN) : SV_Target
				{
					fixed4 c = SampleSpriteTexture(IN.texcoord) * IN.color;
					return c;
				}
			ENDCG
			}
		}
}