Shader "LeosPostShaders/Layer"
{
    Properties
    {
		_MainTex("Texture", 2D) = "white" {}
		_Subtract("Subtract (0 or 1)", Range(0, 1)) = 1
		_Fade("Fade", Range(0.0,1)) = 1
		_LayerTex("Layer Texture", 2D) = "white" {}
    }
    SubShader
    {
        // No culling or depth
        Cull Off ZWrite Off ZTest Off

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

            sampler2D _MainTex, _LayerTex;
			int _Subtract;
			float _Fade;

            fixed4 frag (v2f i) : SV_Target
            {
				fixed4 baseCol = tex2D(_MainTex, i.uv);
				fixed4 layerCol = tex2D(_LayerTex, i.uv);
				
				fixed4 resultCol = fixed4(baseCol.rgb + layerCol.rgb, 1);

				if (_Subtract) { resultCol = fixed4(baseCol.rgb - layerCol.rgb, 1); }

				resultCol.rgb *= baseCol.a;
				resultCol.a = baseCol.a;

                return lerp(baseCol, resultCol, _Fade);
            }
            ENDCG
        }
    }
}
