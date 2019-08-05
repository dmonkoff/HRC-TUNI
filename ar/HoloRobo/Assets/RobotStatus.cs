using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Scripts;
using UnityEngine.UI;
using System;
public class RobotStatus : Singleton<RobotStatus> {

    struct Status
    {
        public Color32 statusColor;
        public string statusText;
        public int fontSize;

        //Constructor (not necessary, but helpful)
        public Status(Color32 statuscolor, string statustext, int fontsize)
        {
            this.statusColor = statuscolor;
            this.statusText = statustext;
            this.fontSize = fontsize;
        }
    }
    private Status active_status = new Status(new Color32(236, 10, 10, 100), "Robot active", 26);
    private Status go_and_dm_simultaneously = new Status(new Color32(0, 23, 255, 100), "Press GO and DM simultaneously", 26);
    private Status non_active_status = new Status(new Color32(7, 159, 33, 100), "Touch GO and DM to start the robot", 36);
    private Status force_mode = new Status(new Color32(209, 255, 0, 136), "Grab the rocker shaft, going into force mode in", 36);
    private Status accept_robot_action = new Status(new Color32(7, 159, 33, 100), "Touch GO and DM for accepting the following robot action:", 36);

    public SSListener connector;
    public ProceduralMeshWithImageTarget convexHull;
    public Text status_text;
    private string current_status;

    // timer for force mode
    private float time_left = 5.0f;

    // images of the different mode
    public GameObject frame_img;
    public GameObject rocket_shaft_img;


    // Use this for initialization
    private void UpdateStatus(Status new_status)
    {
        GetComponent<Image>().color = new_status.statusColor;
        status_text.text = new_status.statusText;
        status_text.fontSize = new_status.fontSize;
    }

    void Start () {
        UpdateStatus(non_active_status);
        current_status = "stat"; // stationary
        rocket_shaft_img.SetActive(false);
        frame_img.SetActive(false);
        convexHull.setMeshVisibility(true);
    }

    private void UpdateForceStatus()
    {
        Debug.Log("Here");
        time_left -= Time.deltaTime;
        if (time_left < 0) time_left = 0.0f;
        force_mode.statusText = "Grab the rocker shaft, going into force mode in " + time_left.ToString("0.00") + " seconds";
        UpdateStatus(force_mode);
        rocket_shaft_img.SetActive(false);
        frame_img.SetActive(false);
    }

    public void ResetStatus()
    {
        current_status = "stat";
        UpdateStatus(non_active_status);
        rocket_shaft_img.SetActive(false);
        frame_img.SetActive(false);
        convexHull.setMeshVisibility(true);
        time_left = 5.0f;
    }
	// Update is called once per frame
	void Update () {


        //// TODO comment out

        // time_left -= Time.deltaTime;
        // if (time_left < 0) time_left = 0.0f;
        // force_mode.statusText = "Grab the rocker shaft, going into force mode in " + time_left.ToString() + " seconds";
        // UpdateStatus(force_mode);
        // return;

        // force mode is the final state of the robot
        if (current_status == "forc") UpdateForceStatus();

        string latest_msg = connector.latestRecievedMsg;
        if (String.IsNullOrEmpty(latest_msg))
        {
            // Debug.Log("No message");
            return;
        }
        string status = latest_msg.Substring(0, 4);
    
        if (status == current_status) return;

        if (status == "move")
        {
            UpdateStatus(active_status);
            rocket_shaft_img.SetActive(false);
            frame_img.SetActive(false);
        }
        else if (status == "stat")
        {
            UpdateStatus(non_active_status);
            rocket_shaft_img.SetActive(false);
            frame_img.SetActive(false);
        }
        else if (status == "dmnp")
        {
            UpdateStatus(go_and_dm_simultaneously);
            rocket_shaft_img.SetActive(false);
            frame_img.SetActive(false);
        }
        else if (status == "con1")
        {
            UpdateStatus(accept_robot_action);
            rocket_shaft_img.SetActive(false);
            frame_img.SetActive(true);
        }
        else if (status == "con2")
        {
            UpdateStatus(accept_robot_action);
            rocket_shaft_img.SetActive(true);
            frame_img.SetActive(false);
        }
        else if (status == "forc")
        {
            UpdateForceStatus();
            convexHull.setMeshVisibility(false);
        }
        else
        {
            Debug.Log("Unknown state");
        }
        current_status = status;
        // GetComponent<Image>().color = red;
    }
}
