using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraManager : MonoBehaviour
{
    public float rotationSpeed = 200;
    public static Camera cam;
    private Vector3 rot;

    void Awake()
    {
        cam = GetComponentInChildren<Camera>();
        rot = transform.rotation.eulerAngles;
    }

    void Update()
    {
        float inp;
        inp = Input.GetAxis("Horizontal");
        if (Input.GetAxis("Horizontal") != 0)
        {
            rot.y -= inp * Time.deltaTime * rotationSpeed;
        }
        inp = Input.GetAxis("Vertical");
        if (Input.GetAxis("Vertical") != 0)
        {
            rot.x += inp * Time.deltaTime * rotationSpeed;
        }
        rot.x = Mathf.Clamp(rot.x, 0, 85);
        transform.rotation = Quaternion.Euler(rot);
    }
}
