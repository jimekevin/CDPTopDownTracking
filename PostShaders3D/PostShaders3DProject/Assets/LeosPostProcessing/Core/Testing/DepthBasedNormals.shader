Shader "LeosPostShaders/NormalMap"
{
    Properties
    {
        _MainTex ("Texture", 2D) = "white" {}
		_Aspect("Aspect", float) = 1.7777777
		_SampleOffset("Sample Offset", Range(0.0001, 0.01)) = 0.001
		_Factor("Factor", float) = 0.5
    }
    SubShader
    {
        // No culling or sample
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
			float _SampleOffset, _Factor, _Aspect;

            fixed4 frag (v2f i) : SV_Target
            {
				fixed4 col = tex2D(_MainTex, i.uv);

				float sampleRight = tex2D(_MainTex, i.uv + float2(1, 0) * _SampleOffset * _Aspect).r;
				float sampleLeft = tex2D(_MainTex, i.uv + float2(-1, 0) * _SampleOffset * _Aspect).r;

				float sampleTop = tex2D(_MainTex, i.uv + float2(0, 1) * _SampleOffset).r;
				float sampleBottom = tex2D(_MainTex, i.uv + float2(0, -1) * _SampleOffset).r;

				float dx = (sampleRight - sampleLeft) * _Factor;
				dx = (dx + 1) * 0.5;
				float dy = (sampleTop - sampleBottom) * _Factor;
				dy = (dy + 1) * 0.5;
				float3 dir = float3(dx, dy, 1);
				float3 normal = normalize(dir);

				return half4(dir.xyz, 1);
            }
            ENDCG
        }
    }
}
