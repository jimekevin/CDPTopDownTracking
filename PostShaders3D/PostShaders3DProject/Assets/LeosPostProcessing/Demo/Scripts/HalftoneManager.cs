using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class HalftoneManager : MonoBehaviour
{
    public Material mat;
    public RawImage imageUI;
    public Texture2D[] textures;
    private int currentTexture;

    // Start is called before the first frame update
    void OnEnable()
    {
        currentTexture = 0;
        OnDestroy();
        ChangeTexture(0);
    }

    public void ChangeTexture(int value)
    {
        currentTexture += value;
        currentTexture = (currentTexture + textures.Length) % textures.Length;
        mat.SetTexture("_RasterTex", textures[currentTexture]);
        imageUI.texture = textures[currentTexture];
    }

    public void SetRasterSize(float value)
    {
        mat.SetFloat("_RasterSize", value);
    }

    public void SetColorFade(float value)
    {
        mat.SetFloat("_ColorFade", value);
    }

    public void SetSoftness(float value)
    {
        mat.SetFloat("_SoftFactor", value);
    }

    public void SetIntensity(float value)
    {
        mat.SetFloat("_Intensity", value);
    }

    public void SetShear(float value)
    {
        mat.SetFloat("_ShearFactor", value);
    }

    private void OnDestroy()
    {
        SetColorFade(0);
        SetRasterSize(0.01f);
        SetSoftness(0.4f);
        SetIntensity(0.6f);
        SetShear(0.5f);
    }
}
