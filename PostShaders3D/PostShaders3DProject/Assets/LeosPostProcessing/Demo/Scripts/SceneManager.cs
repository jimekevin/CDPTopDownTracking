using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System.Linq;

public class SceneManager : MonoBehaviour
{
    [Header("Environments")]
    public GameObject[] environments;
    public string[] envNames;
    public GameObject[] grounds;
    public Text envTitle;
    public Material doSky, noSky, noTexture, noTextureGround;

    [Header("Setups")]
    public int setupNumber = 6;
    public Material[] matBySetup;
    public GameObject[] camBySetup;
    public GameObject[] optionBySetup;
    public Button[] SetupButtons;

    [Header("Additional Options")]
    public Light sceneLight;

    private int curEnv, curSetup;
    private bool drawTextures;
    public static bool maskedPost;
    private Dictionary<MeshRenderer, Material[]> allMaterials;
    public static SceneManager instance;

    public List<GameObject> switchOn;

    // Start is called before the first frame update
    void Awake()
    {
        instance = this;
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
        drawTextures = true;
        ChangeEnvironment(0);
        SetSetup(0);
        ToggleShadows(true);
        ToggleOrthographic(false);
        foreach (GameObject g in switchOn)
        {
            g.SetActive(true);
        }
        ToggleMaskedPost(false);
    }

    public void ToggleSky(bool value)
    {
        RenderSettings.skybox = value ? doSky : noSky;
        foreach (GameObject c in camBySetup)
        {
            c.GetComponent<Camera>().clearFlags = value ? CameraClearFlags.Skybox : CameraClearFlags.SolidColor;
        }
    }

    public void ToggleGround(bool value)
    {
        foreach (GameObject g in grounds)
        {
            g.SetActive(value);
        }
    }

    public void ToggleTextures(bool value)
    {
        drawTextures = value;
        SetGlobalMaterials();
    }

    public void ToggleMaskedPost(bool value)
    {
        maskedPost = value;
        switchOn[0].SetActive(maskedPost);
        foreach (GameObject c in camBySetup)
        {
            if (!value)
            {
                string[] masknames = { "Default", "NoEffect" };
                c.GetComponent<Camera>().cullingMask = LayerMask.GetMask(masknames);
                c.GetComponent<Camera>().clearFlags = CameraClearFlags.Skybox;
            }
            else
            {
                string[] masknames = { "Default" };
                c.GetComponent<Camera>().cullingMask = LayerMask.GetMask(masknames);
                c.GetComponent<Camera>().clearFlags = CameraClearFlags.SolidColor;
            }
        }
        if (!value)
        {
            foreach (KeyValuePair<MeshRenderer, Material[]> pair in allMaterials)
            {
                pair.Key.gameObject.layer = LayerMask.NameToLayer("NoEffect");
            }
        }
        SetGlobalMaterials();
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
        SetGlobalMaterials();
    }

    public void SetGlobalMaterials()
    {
        foreach (KeyValuePair<MeshRenderer,Material[]> pair in allMaterials)
        {
            bool baseSetup = matBySetup[curSetup] == null;
            if (maskedPost) { baseSetup = baseSetup || pair.Key.gameObject.layer == LayerMask.NameToLayer("NoEffect"); }
            if (baseSetup)
            {
                if (drawTextures)
                {
                    pair.Key.sharedMaterials = pair.Value;
                }
                else
                {
                    Material[] m = new Material[pair.Value.Length];
                    for (int i = 0; i < m.Length; i++)
                    {
                        if (grounds.Contains(pair.Key.gameObject))
                        {
                            m[i] = noTextureGround;
                        }
                        else
                        {
                            m[i] = noTexture;
                        }
                    }
                    pair.Key.sharedMaterials = m;
                }
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
