using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Scripts;

public class MouseController : Singleton<MouseController> {

    public float sensitivity = 10f;
    public float maxYAngle = 80f;
    private Vector2 currentRotation;
    void Update()
    {
#if UNITY_EDITOR
        currentRotation.x += Input.GetAxis("Mouse X") * sensitivity;
        currentRotation.y -= Input.GetAxis("Mouse Y") * sensitivity;
        currentRotation.x = Mathf.Repeat(currentRotation.x, 360);
        currentRotation.y = Mathf.Clamp(currentRotation.y, -maxYAngle, maxYAngle);
        Camera.main.transform.rotation = Quaternion.Euler(currentRotation.y, currentRotation.x, 0);
        if (Input.GetMouseButtonDown(0))
            Cursor.lockState = CursorLockMode.Locked;
#endif
    }
}
