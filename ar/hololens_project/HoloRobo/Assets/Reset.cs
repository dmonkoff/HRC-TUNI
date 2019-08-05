using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Scripts;

public class Reset : MonoBehaviour {

    // Use this for initialization
    public RobotStatus robotStatusObject;
	void Start () {
        if (robotStatusObject == null)
        {
            Debug.Log("RobotStatus object not set!");
        }
	}
	
	// Update is called once per frame
	void Update () {
		
	}

    void OnTapped()
    {
        robotStatusObject.ResetStatus();
    }



}
