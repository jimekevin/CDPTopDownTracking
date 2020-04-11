using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class SceneManager : MonoBehaviour
{
    public Material[] globalMaterials;
    public GameObject[] environments;
    public string[] matNames, envNames;
    public Text matTitle, envTitle;
    private int curEnv, curMat;
    private Dictionary<MeshRenderer, Material[]> allMaterials;

    // Start is called before the first frame update
    void Awake()
    {
        allMaterials = new Dictionary<MeshRenderer, Material[]>();
        foreach (GameObject g in environments) { g.SetActive(true); }
        MeshRenderer[] meshies = FindObjectsOfType<MeshRenderer>();
        for (int i = 0; i < meshies.Length; i++)
        {
            allMaterials.Add(meshies[i], meshies[i].materials);
        }
        curEnv = 0;
        curMat = 0;
        ChangeEnvironment(0);
        ChangeGlobalMaterials(0);
    }

    public void ChangeGlobalMaterials(int plusValue)
    {
        curMat += plusValue;
        curMat = (curMat + globalMaterials.Length) % globalMaterials.Length;
        matTitle.text = matNames[curMat];
        foreach (KeyValuePair<MeshRenderer,Material[]> pair in allMaterials)
        {
            if (globalMaterials[curMat] == null)
            {
                pair.Key.materials = pair.Value;
            }
            else
            {
                Material[] m = new Material[pair.Value.Length];
                Debug.Log(m.Length);
                for (int i = 0; i < m.Length; i++)
                {
                    m[i] = globalMaterials[curMat];
                }
                pair.Key.materials = m;
            }
        }
    }

    public void ChangeEnvironment(int plusValue)
    {
        curEnv += plusValue;
        curEnv = (curEnv + environments.Length) % environments.Length;
        envTitle.text = envNames[curEnv];
        for (int i = 0; i < environments.Length; i++)
        {
            environments[i].SetActive(i == curEnv);
        }
    }
}
