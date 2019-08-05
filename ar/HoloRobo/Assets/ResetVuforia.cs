using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Vuforia;
using UnityEngine.UI;

public class ResetVuforia : MonoBehaviour
{


    private bool vuforia_enabled = true;
    public Text textButton;
    // Update is called once per frame
    void OnTapped()
    {
        Debug.Log("Reseting Vuforia");
        Camera mainCamera = Camera.main;
        if (mainCamera && vuforia_enabled)
        {
            if (mainCamera.GetComponent<VuforiaBehaviour>() != null)
            {
                mainCamera.GetComponent<VuforiaBehaviour>().enabled = false;
            }
            if (mainCamera.GetComponent<VideoBackgroundBehaviour>() != null)
            {
                mainCamera.GetComponent<VideoBackgroundBehaviour>().enabled = false;
            }
            if (mainCamera.GetComponent<DefaultInitializationErrorHandler>() != null)
            {
                mainCamera.GetComponent<DefaultInitializationErrorHandler>().enabled = false;
            }
            vuforia_enabled = false;
        }
        else if (mainCamera && !vuforia_enabled)
        {
            if (mainCamera.GetComponent<VuforiaBehaviour>() != null)
            {
                mainCamera.GetComponent<VuforiaBehaviour>().enabled = true;
            }
            if (mainCamera.GetComponent<VideoBackgroundBehaviour>() != null)
            {
                mainCamera.GetComponent<VideoBackgroundBehaviour>().enabled = true;
            }
            if (mainCamera.GetComponent<DefaultInitializationErrorHandler>() != null)
            {
                mainCamera.GetComponent<DefaultInitializationErrorHandler>().enabled = true;
            }
            vuforia_enabled = true;
        }
    }

    void Update()
    {
        if (vuforia_enabled)
        {
            textButton.text = "Disable Vuforia";
        }
        else
        {
            textButton.text = "Enable Vuforia";
        }

    }

}
