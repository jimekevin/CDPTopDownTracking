using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WireframeManager : MonoBehaviour
{
    public Material mat;

    private void Start()
    {
        DrawDiagonals(false);
        DrawTransparent(false);
        DrawBackfaces(true);
        DrawDotted(true);
        SetNormalTreshold(0);
    }

    public void DrawDiagonals(bool value)
    {
        mat.SetInt("_DrawDiag", value ? 1 : 0);
    }

    public void DrawTransparent(bool value)
    {
        mat.SetInt("_Transparent", value ? 1 : 0);
    }

    public void DrawBackfaces(bool value)
    {
        mat.SetInt("_Backfaces", value ? 1 : 0);
    }

    public void DrawDotted(bool value)
    {
        mat.SetInt("_BacklineDotted", value ? 1 : 0);
    }

    public void SetNormalTreshold(float value)
    {
        mat.SetFloat("_NormalTreshold", value);
    }
}
