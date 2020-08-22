using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class EffectCombination : MonoBehaviour
{
    public Material textureAdder;
    public Material[] materials;
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
            Graphics.Blit(source, destination, materials[0]);
            return;
        }
#endif
        if (SceneManager.maskedPost)
        {
            rt = RenderTexture.GetTemporary(Screen.width, Screen.height, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.Default, 2, RenderTextureMemoryless.None);
            Graphics.Blit(source, rt);
            Graphics.Blit(source, rt, materials[0]);
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
        else
        {
            if (img != null) { img.gameObject.SetActive(false); }
            Graphics.Blit(source, destination, materials[0]);
        }
    }
}
