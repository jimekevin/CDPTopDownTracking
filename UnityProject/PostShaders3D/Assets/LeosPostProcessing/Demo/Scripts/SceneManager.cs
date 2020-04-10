using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class SceneManager : MonoBehaviour
{
    public Material empty;
    private Dictionary<MeshRenderer, Material[]> allMaterials;

    // Start is called before the first frame update
    void Awake()
    {
        allMaterials = new Dictionary<MeshRenderer, Material[]>();
        MeshRenderer[] meshies = FindObjectsOfType<MeshRenderer>();
        for (int i = 0; i < meshies.Length; i++)
        {
            allMaterials.Add(meshies[i], meshies[i].materials);
        }
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void SetEmptyMaterials(bool value)
    {
        foreach (KeyValuePair<MeshRenderer,Material[]> pair in allMaterials)
        {
            if (value)
            {
                Material[] m = new Material[pair.Value.Length];
                for (int i = 0; i < m.Length; i++)
                {
                    m[i] = empty;
                }
                pair.Key.materials = m;
            }
            else
            {
                pair.Key.materials = pair.Value;
            }
        }
    }
}
