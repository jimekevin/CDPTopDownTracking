using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class SceneManager : MonoBehaviour
{
    [Header("Environments")]
    public GameObject[] environments;
    public string[] envNames;
    public GameObject[] grounds;
    public Text envTitle;

    [Header("Setups")]
    public int setupNumber = 6;
    public Material[] matBySetup;
    public GameObject[] camBySetup;
    public GameObject[] optionBySetup;
    public bool[] groundBySetup;
    public Button[] SetupButtons;

    [Header("Additional Options")]
    public Light sceneLight;

    private int curEnv, curSetup;
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
        CameraManager.cams = new Camera[camBySetup.Length];
        for (int i = 0; i < camBySetup.Length; i++)
        {
            CameraManager.cams[i] = camBySetup[i].GetComponent<Camera>();
        }
        curEnv = 0;
        curSetup = 0;
        ChangeEnvironment(0);
        SetSetup(0);
        ToggleShadows(true);
        ToggleOrthographic(false);
    }

    public void SetSetup(int value)
    {
        curSetup = value;
        for (int i = 0; i < setupNumber; i++)
        {
            SetupButtons[i].image.color = i == curSetup ? new Color(0.6f, 1, 0.8f, 1) : Color.white;
            camBySetup[i].SetActive(i == curSetup);
            if (optionBySetup[i] != null) { optionBySetup[i].SetActive(i == curSetup); }
        }
        foreach (GameObject g in grounds)
        {
            g.SetActive(groundBySetup[curSetup]);
        }
        SetGlobalMaterials();
    }

    public void SetGlobalMaterials()
    {
        foreach (KeyValuePair<MeshRenderer,Material[]> pair in allMaterials)
        {
            if (matBySetup[curSetup] == null)
            {
                pair.Key.sharedMaterials = pair.Value;
            }
            else
            {
                Material[] m = new Material[pair.Value.Length];
                for (int i = 0; i < m.Length; i++)
                {
                    m[i] = matBySetup[curSetup];
                }
                pair.Key.sharedMaterials = m;
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

    public void ToggleShadows(bool value)
    {
        sceneLight.shadows = value ? LightShadows.Soft : LightShadows.None;
    }

    public void ToggleOrthographic(bool value)
    {
        foreach (Camera c in CameraManager.cams)
        {
            c.orthographic = value;
        }
    }
}
