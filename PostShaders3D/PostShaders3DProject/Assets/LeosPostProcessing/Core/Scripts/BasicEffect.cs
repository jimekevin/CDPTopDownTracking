using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class BasicEffect : MonoBehaviour
{
    public Material material;
    protected Camera cam;
    public UnityEngine.UI.RawImage img;
    protected RenderTexture rt;

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
        if (rt == null)
        {
            rt = RenderTexture.GetTemporary(Screen.width, Screen.height, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.Default, 2, RenderTextureMemoryless.None);
        }
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
        //RenderTexture.ReleaseTemporary(rt);
    }

    protected void OnDisable()
    {
        if (img != null)
        {
            img.gameObject.SetActive(false);
        }
    }
}