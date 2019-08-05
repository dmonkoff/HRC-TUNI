using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Vuforia;

public class StopRobot : MonoBehaviour {

#if UNITY_EDITOR
    // Update is called once per frame
    void Update () {
        if (Input.GetKeyDown(KeyCode.Q))
        {
            Debug.Log("Stopping robot");
            OnStopRobot();
        }
    }
#endif

    //Tap Gesture on HL
#if !UNITY_EDITOR
    void OnTapped()
    {
        if (InteractibleManager.Instance.FocusedGameObject == GameObject.Find("Stop_Button")) OnStopRobot();
    }
#endif

    private void OnStopRobot() {

        // This will now disable vuforia camera
        Camera mainCamera = Camera.main;
        if (mainCamera)
        {
            //Debug.Log("Disabling");
            if (mainCamera.GetComponent<VuforiaBehaviour>() != null)
            {
                Debug.Log("VuforiaBehaviour");
                // VuforiaBehaviour.Instance.enabled = false;
               mainCamera.GetComponent<VuforiaBehaviour>().enabled = false;
            }
            if (mainCamera.GetComponent<VideoBackgroundBehaviour>() != null)
            {
                Debug.Log("VideoBackgroundBehaviour");
                mainCamera.GetComponent<VideoBackgroundBehaviour>().enabled = false;
            }
            if (mainCamera.GetComponent<DefaultInitializationErrorHandler>() != null)
            {
                Debug.Log("DefaultInitializationErrorHandler");
                mainCamera.GetComponent<DefaultInitializationErrorHandler>().enabled = false;
            }
           
            //mainCamera.clearFlags = CameraClearFlags.Skybox;
        }

        /*
        Debug.Log("Starting robot request");
        TCP_connector.Instance.request_msg = "stop\n";
        */

    }
}