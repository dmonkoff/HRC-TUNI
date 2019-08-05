using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Scripts;
using Vuforia;

public class TransformToImageTarget : MonoBehaviour {

    private GameObject imageTarget;
    public Vector3 positionImageTarget;
    public Vector3 prev_positon = new Vector3(0.0f,0.0f,0.0f);
    public Quaternion prev_orientation;

    private Quaternion imageTarget_rotation;
    private Vector3 imageTarget_position;


    // Use this for initialization
    void Start () {
        imageTarget = GameObject.Find("ImageTarget");
        positionImageTarget = Vector3.zero;
        imageTarget_rotation = Quaternion.identity;
        imageTarget_position = Vector3.zero;
        // transform.rotation = Quaternion.identity;
        // prev_positon = transform.position;
        // prev_orientation = transform.rotation;
    }


    private void UpdatePosition()
    {
        // positionImageTargetFrame = this.transform.position - imageTarget.transform.position; // - offsets;
        // positionImageTargetFrame = imageTarget.transform.InverseTransformDirection(this.transform.position); // works also in some sense
        // positionImageTargetFrameTemp = imageTarget.transform.TransformPoint(this.transform.position);
        // Debug.Log(imageTarget.transform.position);

        var worldToLocalMatrix = Matrix4x4.TRS(imageTarget_position, imageTarget_rotation, Vector3.one);
        positionImageTarget = worldToLocalMatrix.MultiplyPoint3x4(prev_positon);
        transform.position = positionImageTarget;
        transform.rotation = imageTarget.transform.rotation;

        // transform.position = imageTarget.transform.position + prev_positon;
        // transform.rotation = imageTarget.transform.rotation;
        // transform.position = new_pos;
        // transform.position = imageTarget.transform.position;
        // Matrix4x4 m = Matrix4x4.Rotate(imageTarget.transform.rotation).inverse;
        // transform.position = m.MultiplyPoint3x4(transform.position);
    }

    private void UpdateImageTargetPose()
    {
        imageTarget_rotation = imageTarget.transform.rotation;
        imageTarget_position = imageTarget.transform.position;
    }


    // Update is called once per frame
    void Update () {
        // Debug.Log("Current position: " + transform.position);
        // transform.position = prev_positon;
        // transform.rotation = prev_orientation;
        /*Camera mainCamera = Camera.main;
        if (mainCamera.GetComponent<VuforiaBehaviour>() != null || imageTarget == null)
        {
            return;
        }*/
        UpdateImageTargetPose();
        UpdatePosition();
    }
}
