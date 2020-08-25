using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SceneSize : MonoBehaviour
{
    public Vector2 minMax;
    public float zoomSpeed = 20;
    private float size = 1;

    // Start is called before the first frame update
    void Start()
    {
        size = transform.localScale.x;
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKey(KeyCode.Plus) || Input.GetKey(KeyCode.KeypadPlus))
        {
            size += zoomSpeed * Time.deltaTime;
        }
        if (Input.GetKey(KeyCode.Minus) || Input.GetKey(KeyCode.KeypadMinus))
        {
            size -= zoomSpeed * Time.deltaTime;
        }
        size = Mathf.Clamp(size, minMax.x, minMax.y);
        transform.localScale = Vector3.one * size;
    }
}
