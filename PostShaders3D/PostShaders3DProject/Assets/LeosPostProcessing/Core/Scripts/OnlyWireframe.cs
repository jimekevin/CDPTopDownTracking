using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class OnlyWireframe : BasicEffect
{
    public LayerMask layer;

    private void OnPreRender()
    {
        cam.cullingMask = layer;
    }
}
