using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System.Linq;

public class SceneManager : MonoBehaviour
{
    public bool isVR;

    [Header("Environments")]
    public GameObject[] environments;
    public string[] envNames;
    public GameObject[] grounds;
    public Text envTitle;
    public Material doSky, noSky, noTexture, noTextureGround, wireframe;

    [Header("Setups")]
    public int setupNumber = 6;
    public GameObject[] camBySetup;
    public GameObject[] optionBySetup;
    public Button[] SetupButtons;

    [Header("Additional Options")]
    public Light sceneLight;

    private int curEnv, lastClickedSetup, curSetup;
    private List<int> curSetups;
    private bool drawTextures;
    public static bool maskedPost;
    [HideInInspector]
    public Dictionary<MeshRenderer, RendererData> allMeshes;
    public static SceneManager instance;

    public List<GameObject> switchOn;

    public struct RendererData
    {
        public Material[] materials;
        public bool isWireframe;
    }

    // Start is called before the first frame update
    void Awake()
    {
        instance = this;
        allMeshes = new Dictionary<MeshRenderer, RendererData>();
        foreach (GameObject g in environments) { g.SetActive(true); }
        MeshRenderer[] meshies = FindObjectsOfType<MeshRenderer>();
        for (int i = 0; i < meshies.Length; i++)
        {
            RendererData rd;
            rd.materials = meshies[i].sharedMaterials;
            rd.isWireframe = meshies[i].gameObject.layer == LayerMask.NameToLayer("Wireframe");
            allMeshes.Add(meshies[i], rd);
        }
        CameraManager.cams = new Camera[camBySetup.Length];
        for (int i = 0; i < camBySetup.Length; i++)
        {
            CameraManager.cams[i] = camBySetup[i].GetComponent<Camera>();
        }
        curEnv = 0;
        lastClickedSetup = 0;
        curSetups = new List<int>();
        ToggleTextures(false);
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
                if (!c.name.Contains("Wireframe"))
                {
                    string[] masknames = { "Default", "NoEffect" };
                    c.GetComponent<Camera>().cullingMask = LayerMask.GetMask(masknames);
                    c.GetComponent<Camera>().clearFlags = CameraClearFlags.Skybox;
                }
            }
            else
            {
                if (!c.name.Contains("Wireframe"))
                {
                    string[] masknames = { "Default" };
                    c.GetComponent<Camera>().cullingMask = LayerMask.GetMask(masknames);
                    c.GetComponent<Camera>().clearFlags = CameraClearFlags.SolidColor;
                }
            }
        }
        foreach (KeyValuePair<MeshRenderer, RendererData> pair in allMeshes)
        {
            if (!value)
            {
                if (pair.Value.isWireframe)
                {
                    pair.Key.gameObject.layer = LayerMask.NameToLayer("Wireframe");
                }
                else
                {
                    pair.Key.gameObject.layer = LayerMask.NameToLayer("NoEffect");
                }
            }
            else
            {
                if (pair.Value.isWireframe)
                {
                    pair.Key.gameObject.layer = LayerMask.NameToLayer("WireframeNoEffect");
                }
            }
        }
        SetGlobalMaterials();
    }

    public void SetSetup(int value)
    {
        if (isVR) { SetSetupVR(value); }
        else { SetSetupMulti(value); }
    }

    public void SetSetupMulti(int value)
    {
        lastClickedSetup = value;
        if (curSetups.Contains(value)) { curSetups.Remove(value); }
        else { curSetups.Add(value); }
        if (curSetups.Count == 0) { SetSetup(0); }

        for (int i = 0; i < setupNumber; i++)
        {
            SetupButtons[i].image.color = curSetups.Contains(i) ? new Color(0.6f, 1, 0.8f, 1) : Color.white;
            camBySetup[i].SetActive(curSetups.Contains(i));
            if (optionBySetup[i] != null) { optionBySetup[i].SetActive(i == lastClickedSetup && curSetups.Contains(i)); }
        }

        camBySetup[value].GetComponent<BasicEffect>().img.transform.SetAsLastSibling();
        SetGlobalMaterials();
    }

    public void SetSetupVR(int value)
    {
        curSetup = value;

        for (int i = 0; i < setupNumber; i++)
        {
            SetupButtons[i].image.color = i == curSetup ? new Color(0.6f, 1, 0.8f, 1) : Color.white;
            camBySetup[i].SetActive(i == curSetup);
            if (optionBySetup[i] != null) { optionBySetup[i].SetActive(i == curSetup); }
        }

        camBySetup[value].GetComponent<BasicEffect>().img.transform.SetAsLastSibling();
        SetGlobalMaterials();
    }

    public void SetGlobalMaterials()
    {
        foreach (KeyValuePair<MeshRenderer,RendererData> pair in allMeshes)
        {
            if (pair.Value.isWireframe)
            {
                Material[] m = new Material[pair.Value.materials.Length];
                for (int i = 0; i < m.Length; i++)
                {
                    m[i] = wireframe;
                }
                pair.Key.sharedMaterials = m;
            }
            else
            {
                if (drawTextures)
                {
                    pair.Key.sharedMaterials = pair.Value.materials;
                }
                else
                {
                    Material[] m = new Material[pair.Value.materials.Length];
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
