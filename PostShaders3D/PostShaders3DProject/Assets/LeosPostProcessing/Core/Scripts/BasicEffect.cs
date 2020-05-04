using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class BasicEffect : MonoBehaviour
{
    public Material material;
    protected Camera cam;

    protected virtual void Awake()
    {
        cam = GetComponent<Camera>();
    }

    // Postprocess Image
    protected virtual void OnRenderImage(RenderTexture source, RenderTexture destination)
    {
        Graphics.Blit(source, destination, material);
    }
}