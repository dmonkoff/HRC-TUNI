

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Vuforia;

public class CylinderBetweenTwoPoints : MonoBehaviour
{
    [SerializeField]
    private Transform cylinderPrefab;
    private Quaternion rotationDelta = Quaternion.identity;
    bool lock_transformation = false;
    public Vector3 markerOffset = new Vector3(0.0f, 0.0f, 0.0f);
    // public float object_scale = 1.0f; 
    private GameObject[] cylinders = new GameObject[6];

    private void Start()
    {
        /*leftSphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        rightSphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        leftSphere.transform.position = new Vector3(-3, 0, 0);
        rightSphere.transform.position = new Vector3(3, 0, 0);*/


        // TODO: comment out
        // InstantiateCylinder(cylinderPrefab);
    }

    private void Update()
    {
        // TODO: check if network is on

        // TODO: comment out
        // UpdateCylinderPosition(cylinders);
    }

    private void InstantiateCylinder(Transform cylinderPrefab)
    {
        // instantiate the objects back of the object
        for (int i = 0; i < cylinders.Length; ++i) {
            cylinders[i] = Instantiate<GameObject>(cylinderPrefab.gameObject, new Vector3(i, 1, i*(-1.0f)), Quaternion.identity);
        }
        
        // UpdateCylinderPosition(cylinders, beginPoint, endPoint);
    }


    private void TranslateBase(string[] words) {
        var x = Convert.ToSingle(words[1]);
        var y = Convert.ToSingle(words[2]);
        var z = Convert.ToSingle(words[3]);
        Vector3 offset = new Vector3((-1) * x, y, z);
        rotationDelta = transform.rotation * Quaternion.Euler(-90, 0, 0);
        for (int i = 0; i < cylinders.Length; ++i)
        {
            cylinders[i].transform.position += rotationDelta * offset;
        }

        /*
        // Debug.Log("Translating base by using offsets: " + x.ToString() + "," + y.ToString() + "," + z.ToString() );
        for (int i = 0; i < cylinders.Length; ++i)
        {
            cylinders[i].transform.position += new Vector3(x, y, z);
        }
        markerOffset += new Vector3(x, y, z);
        */
        markerOffset += offset;
        Debug.Log("Current offets (x,y,z): (" + markerOffset.x.ToString() + "," + markerOffset.y.ToString() + "," + markerOffset.z.ToString() + ")");
        TCP_connector.Instance.latestRecievedMsg = null;
    }


    private void UpdateCylinderPosition(GameObject[] cylinders)
    {
        // parse the message
        string latest_msg = TCP_connector.Instance.latestRecievedMsg;
        // var result = latest_msg.Substring(latest_msg.Length - 3);
        //Debug.Log("Last characters: " + result);
        

        if (String.IsNullOrEmpty(latest_msg)) return;
        string[] words = latest_msg.Split(':');
        // Debug.Log("Last characters: "+words[words.Length - 1]+ " " + words[words.Length - 2]);
        // check if robot base translation request
        if (words[0] == "t") {
            if (words.Length != 4) return;
            TranslateBase(words);
            return;
        }

        // Debug.Log(words.Length);
        if (words.Length != 3 * 8) return;
        
        for (int i = 0; i < cylinders.Length; ++i) {

            Vector3 beginPoint = new Vector3(Convert.ToSingle(words[(i * 3)]), Convert.ToSingle(words[(i * 3) + 1]), Convert.ToSingle(words[(i * 3) + 2]));
            Vector3 endPoint = new Vector3(Convert.ToSingle(words[((i+1) * 3)]), Convert.ToSingle(words[((i + 1) * 3) + 1]), Convert.ToSingle(words[((i + 1) * 3) + 2]));

            // 1). Convert right hand to left hand coordinate system
            // 2). Add the transformation between robot base and marker
            beginPoint.x *= (-1.0f); 
            endPoint.x *= (-1.0f);
            rotationDelta = transform.rotation * Quaternion.Euler(-90, 0, 0);

            beginPoint = (rotationDelta * (beginPoint + markerOffset)) + transform.position;
            endPoint = (rotationDelta * (endPoint + markerOffset)) + transform.position;
            
            Vector3 offset = endPoint - beginPoint;
            Vector3 position = beginPoint + (offset / 2.0f); // position is now on the middle on the vector (object) of endPoint - beginPoint

            cylinders[i].transform.position = position;
            cylinders[i].transform.LookAt(endPoint);
            cylinders[i].transform.Rotate(Vector3.right, 90);
            Vector3 localScale = cylinders[i].transform.localScale;
            // length of the cylinder
            // we divide by 2 because the length increases in both directions(?)
            localScale.y = (endPoint - beginPoint).magnitude*0.5f;
            localScale.z = 0.1f; // * object_scale; // width of the joints
            localScale.x = 0.1f; // * object_scale;
            cylinders[i].transform.localScale = localScale;
        }
    }

    void OnTapped() {
        Debug.Log("CylinderBetweenTwoPoints inside tapped");
        lock_transformation = !lock_transformation;
    }
}