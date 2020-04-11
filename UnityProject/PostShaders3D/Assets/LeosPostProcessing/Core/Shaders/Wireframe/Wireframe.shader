Shader "LeosPostShaders/Wireframe" {
	Properties
	{
		_MainTex("Albedo (RGB)", 2D) = "black" {}
		_WireColor("WireColor", Color) = (1,0,0,1)
		_Color("Color", Color) = (1,1,1,1)
		_TintColor("TintColor", Color) = (1,1,1,1)
		_Thickness("Thickness", float) = 1.0
	}

		SubShader
		{
			Tags { "Queue" = "Transparent" "RenderType" = "Transparent" }

			Blend SrcAlpha OneMinusSrcAlpha

			Zwrite On

			Pass
			{
				CGPROGRAM
				#include "UnityCG.cginc"
				#pragma vertex vert
				#pragma geometry geom
				#pragma fragment frag

				float _Thickness;

				struct appdata
				{
					float4 vertex : POSITION;
					float2 texcoord : TEXCOORD0;
					UNITY_VERTEX_INPUT_INSTANCE_ID
				};

				struct v2g
				{
					float4 projectionSpaceVertex : SV_POSITION;
					float4 worldSpacePosition : TEXCOORD1;
					float2 uv : TEXCOORD0;
					UNITY_VERTEX_OUTPUT_STEREO_EYE_INDEX
				};

				struct g2f
				{
					float4 projectionSpaceVertex : SV_POSITION;
					float4 worldSpacePosition : TEXCOORD0;
					float2 uv : TEXCOORD2;
					float4 dist : TEXCOORD1;
					UNITY_VERTEX_OUTPUT_STEREO
				};

				v2g vert(appdata v)
				{
					v2g o;
					UNITY_SETUP_INSTANCE_ID(v);
					UNITY_INITIALIZE_OUTPUT_STEREO_EYE_INDEX(o);

					o.projectionSpaceVertex = UnityObjectToClipPos(v.vertex);
					o.worldSpacePosition = mul(unity_ObjectToWorld, v.vertex);
					o.uv = v.texcoord;
					return o;
				}

				[maxvertexcount(3)]
				void geom(triangle v2g i[3], inout TriangleStream<g2f> triangleStream)
				{
					UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX(i[0]);

					float2 p0 = i[0].projectionSpaceVertex.xy / i[0].projectionSpaceVertex.w;
					float2 p1 = i[1].projectionSpaceVertex.xy / i[1].projectionSpaceVertex.w;
					float2 p2 = i[2].projectionSpaceVertex.xy / i[2].projectionSpaceVertex.w;

					float2 edge0 = p2 - p1;
					float2 edge1 = p2 - p0;
					float2 edge2 = p1 - p0;

					// To find the distance to the opposite edge, we take the
					// formula for finding the area of a triangle Area = Base/2 * Height,
					// and solve for the Height = (Area * 2)/Base.
					// We can get the area of a triangle by taking its cross product
					// divided by 2.  However we can avoid dividing our area/base by 2
					// since our cross product will already be double our area.
					float area = abs(edge1.x * edge2.y - edge1.y * edge2.x);
					float wireThickness = 800 - _Thickness;

					g2f o;
					UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(o);
					o.worldSpacePosition = i[0].worldSpacePosition;
					o.projectionSpaceVertex = i[0].projectionSpaceVertex;
					o.dist.xyz = float3((area / length(edge0)), 0.0, 0.0) * o.projectionSpaceVertex.w * wireThickness;
					o.dist.w = 1.0 / o.projectionSpaceVertex.w;
					o.uv = i[0].uv;
					triangleStream.Append(o);

					o.worldSpacePosition = i[1].worldSpacePosition;
					o.projectionSpaceVertex = i[1].projectionSpaceVertex;
					o.dist.xyz = float3(0.0, (area / length(edge1)), 0.0) * o.projectionSpaceVertex.w * wireThickness;
					o.dist.w = 1.0 / o.projectionSpaceVertex.w;
					o.uv = i[1].uv;
					triangleStream.Append(o);

					o.worldSpacePosition = i[2].worldSpacePosition;
					o.projectionSpaceVertex = i[2].projectionSpaceVertex;
					o.dist.xyz = float3(0.0, 0.0, (area / length(edge2))) * o.projectionSpaceVertex.w * wireThickness;
					o.dist.w = 1.0 / o.projectionSpaceVertex.w;
					o.uv = i[2].uv;
					triangleStream.Append(o);
				}

				half4 _WireColor, _Color;
				sampler2D _MainTex;
				uniform float4 _MainTex_ST;

				fixed4 frag(g2f i) : SV_Target
				{
					float camDistance = length(i.projectionSpaceVertex.xyz - _WorldSpaceCameraPos.xyz);
					camDistance = camDistance * camDistance;
					camDistance = 1;
					//distance of frag from triangles center
					float d = min(i.dist.x, min(i.dist.y, i.dist.z)) / _Thickness * camDistance;
					//fade based on dist from center
					float I = exp2(-4.0 * d * d);

					fixed4 texColor = tex2D(_MainTex, i.uv * _MainTex_ST.xy + _MainTex_ST.zw);

					float4 colorResult = lerp(texColor * _Color, _WireColor, I);

					return colorResult;
				}
			ENDCG
			}
		}
}