using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class ConvexHull : MonoBehaviour {


    [SerializeField]
    private Transform cylinderPrefab;
    private Quaternion rotationDelta = Quaternion.identity;
    bool lock_transformation = false;
    public Vector3 markerOffset = new Vector3(0.0f, 0.0f, 0.0f);
    // public float object_scale = 1.0f; 
    private GameObject[] cylinders = new GameObject[500];



    // Use this for initialization
    void Start () {
        // InstantiateCylinder(cylinderPrefab);
	}


    private void InstantiateCylinder(Transform cylinderPrefab)
    {
        // instantiate the objects back of the object
        for (int i = 0; i < cylinders.Length; ++i)
        {
            cylinders[i] = Instantiate<GameObject>(cylinderPrefab.gameObject, new Vector3(0, 0, 0), Quaternion.identity);
        }
    }






    // Update is called once per frame
    void Update () {
        
        // parse the message
        string latest_msg = TCP_connector.Instance.latestRecievedMsg;
        // var result = latest_msg.Substring(latest_msg.Length - 3);
        //Debug.Log("Last characters: " + result);


        if (String.IsNullOrEmpty(latest_msg)) return;
        string[] words = latest_msg.Split(':');
        // Debug.Log("Last characters: "+words[words.Length - 1]+ " " + words[words.Length - 2]);
        // check if robot base translation request
        /*if (words[0] == "t")
        {
            if (words.Length != 4) return;
            TranslateBase(words);
            return;
        }*/
        if (words.Length % 2 != 0) return;

        // UpdateCylinderPosition(words);
        
    }


    private void UpdateCylinderPosition(string[] words) {
        for (int i = 0; i < cylinders.Length; ++i)
        {
            if (words.Length <= (i * 2) + 1)
                break;

            Vector3 pos;
            pos.x = Convert.ToSingle(words[i*2]);
            pos.y = Convert.ToSingle(words[(i*2)+1]);
            pos.z = 0.0f;
            cylinders[i].transform.position = pos;
            Vector3 localScale;
            localScale.x = localScale.y = localScale.z = 0.02f;
            cylinders[i].transform.localScale = localScale;
        }

    }



}
