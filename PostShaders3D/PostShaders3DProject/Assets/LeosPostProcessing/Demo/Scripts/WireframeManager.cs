using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WireframeManager : MonoBehaviour
{
    public Material mat;

    private void Start()
    {
        DrawDiagonals(false);
        DrawTransparent(true);
        DrawBackfaces(true);
        DrawDotted(true);
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
}
