using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class BasicEffect : MonoBehaviour
{
    public Material material;
    protected Camera cam;
    public UnityEngine.UI.RawImage img;
    private RenderTexture rt;

    protected virtual void Awake()
    {
        cam = GetComponent<Camera>();
    }

    // Postprocess Image
    protected virtual void OnRenderImage(RenderTexture source, RenderTexture destination)
    {
#if UNITY_EDITOR
        if (!UnityEditor.EditorApplication.isPlaying)
        {
            Graphics.Blit(source, destination, material);
            return;
        }
#endif
        rt = RenderTexture.GetTemporary(1920, 1080, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.Default, 2, RenderTextureMemoryless.None);
        Graphics.Blit(source, material);
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
    }
}