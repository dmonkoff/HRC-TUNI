using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Scripts;

public class KeyboardController : Singleton<KeyboardController>
{

    public float speed = 5.0f;


    // Use this for initialization
    void Start()
    {
    }

    void Update()
    {
#if UNITY_EDITOR
        TranslateCamera();
#endif
    }

    void TranslateCamera()
    {
        if (Input.GetKey(KeyCode.RightArrow))
        {
            transform.position += new Vector3((-1.0f) * speed * Time.deltaTime, 0, 0);
        }
        if (Input.GetKey(KeyCode.LeftArrow))
        {
            transform.position += new Vector3(speed * Time.deltaTime, 0, 0);
        }
        if (Input.GetKey(KeyCode.UpArrow))
        {
            transform.position += new Vector3(0, 0, speed * Time.deltaTime);
        }
        if (Input.GetKey(KeyCode.DownArrow))
        {
            transform.position += new Vector3(0, 0, (-1.0f) * speed * Time.deltaTime);
        }

        // keyboard inputs for testing differnet robot operations
        if (Input.GetKeyDown(KeyCode.U))
        {
            Debug.Log(InteractibleManager.Instance.FocusedGameObject.name);
            InteractibleManager.Instance.FocusedGameObject.SendMessageUpwards("OnTapped");
        }


    }

}
