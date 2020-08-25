using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AmbientEdges : DepthVisualization
{
    public Material ao, blur;
    private RenderTexture rtAO, rtBlur;
    public bool debug;

    protected override void OnRenderImage(RenderTexture source, RenderTexture destination)
    {
        //ambient occlusion
        float fovRad = cam.fieldOfView * Mathf.Deg2Rad;
        float invHalfTanFov = 1 / Mathf.Tan(fovRad * 0.5f);
        Vector2 focalLen = new Vector2(invHalfTanFov * ((float)Screen.height / (float)Screen.width), invHalfTanFov);
        Vector2 invFocalLen = new Vector2(1 / focalLen.x, 1 / focalLen.y);
        ao.SetVector("_UVtoView", new Vector4(2 * invFocalLen.x, 2 * invFocalLen.y, -1 * invFocalLen.x, -1 * invFocalLen.y));
        ao.SetMatrix("_CamToClipSpace", cam.projectionMatrix);
        ao.SetMatrix("_ClipToCamSpace", cam.projectionMatrix.inverse);
        rtAO = RenderTexture.GetTemporary(Screen.width, Screen.height, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.Default, 2, RenderTextureMemoryless.None);
        Graphics.Blit(source, rtAO, ao);

        //blur
        rtBlur = RenderTexture.GetTemporary(Screen.width, Screen.height, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.Default, 2, RenderTextureMemoryless.None);
        Graphics.Blit(rtAO, rtBlur, blur);
        if (!debug)
        {
            Graphics.Blit(rtBlur, source);
            material.SetFloat("_Fade", 1);
            base.OnRenderImage(source, destination);
        }
        else
        {
            material.SetTexture("_LayerTex", rtBlur);
            material.SetFloat("_Fade", 1);
            base.OnRenderImage(source, destination);
        }

        //release (VERY IMPORTANT)
        RenderTexture.ReleaseTemporary(rtAO);
        RenderTexture.ReleaseTemporary(rtBlur);
    }
}
