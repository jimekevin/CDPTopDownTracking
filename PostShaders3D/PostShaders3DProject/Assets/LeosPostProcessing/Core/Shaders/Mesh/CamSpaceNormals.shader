Shader "LeosPostShaders/CamSpaceNormals"
{
	Properties
	{
		_Gradient("Gradient", 2D) = "white" {}
		_Color("Tint", Color) = (1,1,1,1)
	}

	SubShader
	{
		Pass
		{
			Tags { "Queue" = "Geometry" "RenderType" = "Opaque" }
			CGPROGRAM
			#pragma vertex vert
			#pragma fragment frag

			#include "UnityCG.cginc"

			struct v2f {
				half3 normal : TEXCOORD0;
				float4 pos : SV_POSITION;
			};

	v2f vert(float4 vertex : POSITION, float3 normal : NORMAL)
	{
		v2f o;
		o.pos = UnityObjectToClipPos(vertex);
		o.normal = UnityObjectToWorldNormal(normal);
		return o;
	}

	sampler2D _Gradient;
	fixed4 _Color;

	fixed4 frag(v2f i) : SV_Target
	{
		fixed4 c = 0;
		c.rgb = mul(UNITY_MATRIX_V, i.normal);
		//c.r = 0;
		//c.rgb += 0.5;
		//c.r = abs(c.r) + 0.5;

		//c.rgb = tex2D(_Gradient, float2(0, (c.r + c.g + c.b) * 0.75)) * _Color;
		//c.a = _Color.a;
		return c;
	}
	ENDCG
	}
	}
}