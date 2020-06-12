using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class DepthVisualization : BasicEffect
{
    protected override void Awake()
    {
        base.Awake();
        cam.depthTextureMode = DepthTextureMode.DepthNormals | DepthTextureMode.Depth;
        //cam.depthTextureMode = cam.depthTextureMode | DepthTextureMode.DepthNormals | DepthTextureMode.Depth;
    }
}
