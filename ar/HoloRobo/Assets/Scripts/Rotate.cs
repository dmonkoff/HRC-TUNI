using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Scripts;

public class Rotate : MonoBehaviour {

    [Tooltip("Number of bytes ")]
    public Vector3 rotation_speed;

    // Update is called once per frame
    void Update () {
        transform.Rotate(Time.deltaTime * rotation_speed.x, Time.deltaTime * rotation_speed.y, Time.deltaTime * rotation_speed.z);
    }
}
