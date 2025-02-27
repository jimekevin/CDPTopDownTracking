﻿Shader "LeosPostShaders/HystereseTreshold"
{
    Properties
    {
        _MainTex ("Texture", 2D) = "white" {}
		_Treshold("Treshold", Range(0.0,1.0)) = 0.5
		_FrontColor("Front Color", Color) = (1,1,1,1)
		_BackColor("Background Color", Color) = (0,0,0,1)
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
			float _Treshold;
			half4 _BackColor, _FrontColor;

            fixed4 frag (v2f i) : SV_Target
            {
                fixed4 col = tex2D(_MainTex, i.uv);

				if (col.r >= _Treshold || col.g >= _Treshold || col.b >= _Treshold)
				{
					return _FrontColor;
				}

				return fixed4(_BackColor.rgb * _BackColor.a, _BackColor.a);
            }
            ENDCG
        }
    }
}
