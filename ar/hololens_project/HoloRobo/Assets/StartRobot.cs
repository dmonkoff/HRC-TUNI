using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class StartRobot : MonoBehaviour {

#if UNITY_EDITOR
    void Update() {
        if (Input.GetKeyDown(KeyCode.S))
        {
            Debug.Log("Starting robot");
            OnStartRobot();
        }
    }
#endif



    //Tap Gesture on HL
#if !UNITY_EDITOR
    void OnTapped()
    {
        if (InteractibleManager.Instance.FocusedGameObject == GameObject.Find("Start_Button")) OnStartRobot();
    }
#endif
    private void OnStartRobot() {
        Debug.Log("Starting robot request");
        TCP_connector.Instance.request_msg = "start\n";
    }



}
