using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class CannyEdgeDetection : BasicEffect
{
    public Material horizontalEdges, verticalEdges;
    [Range(0, 0.02f)]
    public float pixelDistance = 0.01f; 
    private RenderTexture rtHor, rtVert;

    // Postprocess Image
    protected override void OnRenderImage(RenderTexture source, RenderTexture destination)
    {
        horizontalEdges.SetFloat("_Distance", pixelDistance);
        verticalEdges.SetFloat("_Distance", pixelDistance);
        material.SetFloat("_Distance", pixelDistance);
        rtHor = RenderTexture.GetTemporary(Screen.width, Screen.height, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.Default, 2, RenderTextureMemoryless.None);
        Graphics.Blit(source, rtHor, horizontalEdges);
        rtVert = RenderTexture.GetTemporary(Screen.width, Screen.height, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.Default, 2, RenderTextureMemoryless.None);
        Graphics.Blit(source, rtVert, verticalEdges);
        material.SetTexture("_HorizontalTex", rtHor);
        material.SetTexture("_VerticalTex", rtVert);

        base.OnRenderImage(source, destination);

        /*
#if UNITY_EDITOR
        if (!UnityEditor.EditorApplication.isPlaying)
        {

            Graphics.Blit(source, destination, material);

            return;
        }
#endif
        rt = RenderTexture.GetTemporary(1920, 1080, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.Default, 2, RenderTextureMemoryless.None);
        Graphics.Blit(source, rt);



        Graphics.Blit(source, rt, material);
        RenderTexture.active = rt;
        if (img != null && rt != null)
        {
            img.gameObject.SetActive(true);
            img.texture = rt;
        }
        Graphics.Blit(rt, destination);
        RenderTexture.active = destination;
        GL.Clear(true, false, Color.clear);
        RenderTexture.active = null;
        RenderTexture.ReleaseTemporary(rt);
        */

        RenderTexture.ReleaseTemporary(rtHor);
        RenderTexture.ReleaseTemporary(rtVert);
    }
}