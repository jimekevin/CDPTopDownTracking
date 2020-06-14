using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;

public class SelectionField : MonoBehaviour, IPointerDownHandler
{
    public Camera cam;

    public void OnPointerDown(PointerEventData data)
    {
        if (Input.GetMouseButtonDown(0) && SceneManager.maskedPost)
        {
            StartCoroutine(selectionRoutine());
        }
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
            if (changedSth) { SceneManager.instance.SetGlobalMaterials(); }
            yield return new WaitForSeconds(0.05f);
        }
    }
}
