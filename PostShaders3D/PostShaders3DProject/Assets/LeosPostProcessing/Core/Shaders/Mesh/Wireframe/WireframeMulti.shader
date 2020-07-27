Shader "LeosPostShaders/WireframeMulti"
{
	Properties
	{
		[PowerSlider(3.0)]
		_FrontlineWidth("Frontline width", Range(0., 0.5)) = 0.05
		_BacklineWidth("Backline width", Range(0., 0.5)) = 0.05
		_BacklineDistance("Backline Distance", Range(0.01, 2)) = 0.05
		_BacklineFactor("Backline Factor", Range(0.0, 1.0)) = 0.5
		_TintColor("Tint color", color) = (1., 1., 1., 1.)
		_FrontColor("Front color", color) = (1., 1., 1., 1.)
		_BackColor("Back color", color) = (1., 1., 1., 1.)
		_DrawDiag("Draw Diagonals (0 or 1)", Range(0, 1)) = 0
		_Transparent("Transparent (0 or 1)", Range(0, 1)) = 0
		_Backfaces("Backfaces (0 or 1)", Range(0, 1)) = 0
		_BacklineDotted("Backline Dotted (0 or 1)", Range(0, 1)) = 0
		_NormalTreshold("Normal Y Treshold", Range(0, 1)) = 0
	}
		SubShader
		{
			Tags { "Queue" = "Geometry" "RenderType" = "Opaque" }

			Pass
			{
				Cull Front
				CGPROGRAM
				#pragma vertex vert
				#pragma fragment frag
				#pragma geometry geom

			#include "UnityCG.cginc"

			struct v2g {
				float4 worldPos : SV_POSITION;
				float3 normal : NORMAL;
			};

			struct g2f {
				float4 pos : SV_POSITION;
				float3 bary : TEXCOORD0;
				half3 worldNormal : NORMAL;
			};

			v2g vert(appdata_base v) {
				v2g o;
				o.worldPos = mul(unity_ObjectToWorld, v.vertex);
				o.normal = UnityObjectToWorldNormal(v.normal);
				return o;
			}

			int _DrawDiag;

			[maxvertexcount(3)]
			void geom(triangle v2g IN[3], inout TriangleStream<g2f> triStream) {
				float3 param = float3(0., 0., 0.);

				if (!_DrawDiag)
				{
					float EdgeA = length(IN[0].worldPos - IN[1].worldPos);
					float EdgeB = length(IN[1].worldPos - IN[2].worldPos);
					float EdgeC = length(IN[2].worldPos - IN[0].worldPos);

					if (EdgeA > EdgeB && EdgeA > EdgeC)
						param.y = 1.;
					else if (EdgeB > EdgeC && EdgeB > EdgeA)
						param.x = 1.;
					else
						param.z = 1.;
				}

				g2f o;
				o.worldNormal = normalize((IN[0].normal + IN[1].normal + IN[2].normal) / 3.0);
				o.pos = mul(UNITY_MATRIX_VP, IN[0].worldPos);
				o.bary = float3(1., 0., 0.) + param;
				triStream.Append(o);
				o.pos = mul(UNITY_MATRIX_VP, IN[1].worldPos);
				o.bary = float3(0., 0., 1.) + param;
				triStream.Append(o);
				o.pos = mul(UNITY_MATRIX_VP, IN[2].worldPos);
				o.bary = float3(0., 1., 0.) + param;
				triStream.Append(o);
			}

			float _BacklineWidth;
			float _BacklineDistance;
			float _BacklineFactor;
			float _NormalTreshold;
			fixed4 _BackColor;
			int _Backfaces;
			int _BacklineDotted;

			fixed4 frag(g2f i) : SV_Target{
				if (!_Backfaces)
				{
					discard;
				}

				bool any = 0;
				bool nrm = abs(i.worldNormal.y) >= _NormalTreshold;
				if (i.bary.x < _BacklineWidth)
				{
					if (i.bary.z % _BacklineDistance > _BacklineFactor* _BacklineDistance && nrm)
					{
						return _BackColor;
					}
					any = 1;
				}
				if (i.bary.y < _BacklineWidth)
				{
					if (i.bary.x % _BacklineDistance < _BacklineFactor * _BacklineDistance && nrm)
					{
						return _BackColor;
					}
					any = 1;
				}
				if (i.bary.z < _BacklineWidth)
				{
					if (i.bary.y % _BacklineDistance < _BacklineFactor * _BacklineDistance && nrm)
					{
						return _BackColor;
					}
					any = 1;
				}
				if (any && !_BacklineDotted && nrm)
				{
					return _BackColor;
				}

				discard;
				return _BackColor;
			}

			ENDCG
		}

		Pass
		{
			Cull Back
			CGPROGRAM
			#pragma vertex vert
			#pragma fragment frag
			#pragma geometry geom

				#include "UnityCG.cginc"

				struct v2g {
					float4 worldPos : SV_POSITION;
					float3 normal : NORMAL;
				};

				struct g2f {
					float4 pos : SV_POSITION;
					float3 bary : TEXCOORD0;
					half3 worldNormal : NORMAL;
				};

				v2g vert(appdata_base v) {
					v2g o;
					o.worldPos = mul(unity_ObjectToWorld, v.vertex);
					o.normal = UnityObjectToWorldNormal(v.normal);
					return o;
				}

				int _DrawDiag;

				[maxvertexcount(3)]
				void geom(triangle v2g IN[3], inout TriangleStream<g2f> triStream) {
					float3 param = float3(0., 0., 0.);

					if (!_DrawDiag)
					{
						float EdgeA = length(IN[0].worldPos - IN[1].worldPos);
						float EdgeB = length(IN[1].worldPos - IN[2].worldPos);
						float EdgeC = length(IN[2].worldPos - IN[0].worldPos);

						if (EdgeA > EdgeB&& EdgeA > EdgeC)
							param.y = 1.;
						else if (EdgeB > EdgeC&& EdgeB > EdgeA)
							param.x = 1.;
						else
							param.z = 1.;
					}

					g2f o;
					o.worldNormal = normalize((IN[0].normal + IN[1].normal + IN[2].normal) / 3.0);
					o.pos = mul(UNITY_MATRIX_VP, IN[0].worldPos);
					o.bary = float3(1., 0., 0.) + param;
					triStream.Append(o);
					o.pos = mul(UNITY_MATRIX_VP, IN[1].worldPos);
					o.bary = float3(0., 0., 1.) + param;
					triStream.Append(o);
					o.pos = mul(UNITY_MATRIX_VP, IN[2].worldPos);
					o.bary = float3(0., 1., 0.) + param;
					triStream.Append(o);
				}

				float _FrontlineWidth;
				fixed4 _FrontColor, _TintColor;
				int _Transparent;
				float _NormalTreshold;

				fixed4 frag(g2f i) : SV_Target {
					if (!any(bool3(i.bary.x <= _FrontlineWidth, i.bary.y <= _FrontlineWidth, i.bary.z <= _FrontlineWidth)))
					{
						if (_Transparent) {
							discard;
						}
						else {
							return _TintColor;
						}
					}
					if (abs(i.worldNormal.y) < _NormalTreshold) {
						if (_Transparent) {
							discard;
						}
						else {
							return _TintColor;
						}
					}

					return _FrontColor;
				}

				ENDCG
			}
		}
}