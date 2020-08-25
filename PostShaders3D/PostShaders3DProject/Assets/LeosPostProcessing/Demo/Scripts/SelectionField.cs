using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;

public class SelectionField : MonoBehaviour, IPointerDownHandler, IPointerEnterHandler, IPointerExitHandler
{
    public Camera cam;

    public void OnPointerDown(PointerEventData data)
    {
        if (Input.GetMouseButtonDown(0) && SceneManager.maskedPost)
        {
            StartCoroutine(selectionRoutine());
        }
    }

    public void OnPointerEnter(PointerEventData data)
    {
        CameraManager.canMove = true;
    }

    public void OnPointerExit(PointerEventData data)
    {
        CameraManager.canMove = false;
    }

    IEnumerator selectionRoutine()
    {
        List<GameObject> changed = new List<GameObject>();
        while (Input.GetMouseButton(0))
        {
            RaycastHit[] hits = Physics.RaycastAll(cam.ScreenPointToRay(Input.mousePosition));
            bool changedSth = false;
            foreach (RaycastHit h in hits)
            {
                if (!changed.Contains(h.collider.gameObject))
                {
                    changedSth = true;
                    changed.Add(h.collider.gameObject);
                    MeshRenderer mr = h.collider.GetComponent<MeshRenderer>();
                    bool isWireframe = SceneManager.instance.allMeshes[mr].isWireframe;
                    if (isWireframe)
                    {
                        if (h.collider.gameObject.layer == LayerMask.NameToLayer("Wireframe"))
                        {
                            Debug.Log("hello1");
                            h.collider.gameObject.layer = LayerMask.NameToLayer("WireframeNoEffect");
                        }
                        else
                        {
                            Debug.Log("hello2");
                            h.collider.gameObject.layer = LayerMask.NameToLayer("Wireframe");
                        }
                    }
                    else
                    {
                        if (h.collider.gameObject.layer == LayerMask.NameToLayer("Default"))
                        {
                            h.collider.gameObject.layer = LayerMask.NameToLayer("NoEffect");
                        }
                        else
                        {
                            h.collider.gameObject.layer = LayerMask.NameToLayer("Default");
                        }
                    }
                }
            }
            if (changedSth) { SceneManager.instance.SetGlobalMaterials(); }
            yield return new WaitForSeconds(0.05f);
        }
    }
}
