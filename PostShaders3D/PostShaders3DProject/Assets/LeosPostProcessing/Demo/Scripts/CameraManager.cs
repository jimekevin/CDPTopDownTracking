using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraManager : MonoBehaviour
{
    public float rotationSpeed = 200, zoomSpeed;
    public Transform zoomer;
    public static Camera[] cams;
    private Vector3 rot;

    void Awake()
    {
        rot = transform.rotation.eulerAngles;
    }

    void Update()
    {
        float inp;
        inp = Input.GetAxis("Horizontal");
        if (inp != 0)
        {
            rot.y -= inp * Time.deltaTime * rotationSpeed;
        }
        inp = Input.GetAxis("Vertical");
        if (inp != 0)
        {
            rot.x += inp * Time.deltaTime * rotationSpeed;
        }
        rot.x = Mathf.Clamp(rot.x, 0, 85);
        transform.rotation = Quaternion.Euler(rot);

        inp = Input.GetAxis("Mouse ScrollWheel");
        if (inp != 0)
        {
            Vector3 lp = zoomer.localPosition;
            Debug.Log(inp * Time.deltaTime * zoomSpeed);
            lp.z += inp * Time.deltaTime * zoomSpeed;
            lp.z = Mathf.Clamp(lp.z, -50, -10);
            foreach (Camera c in cams)
            {
                c.orthographicSize = lp.z * -0.5f;
            }
            zoomer.localPosition = lp;
        }
    }
}
