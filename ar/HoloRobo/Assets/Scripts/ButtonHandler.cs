using System.Collections;
using System.Collections.Generic;
using UnityEngine.UI;
using UnityEngine;
using Scripts;

public class ButtonHandler : Singleton<ButtonHandler>
{

    private Button button;
    private Image image;
    private Color default_color;

    void Awake()
    {
        button = GetComponent<Button>();
        image = GetComponent<Image>();
        default_color = image.color;
    }

    void OnGazeEnter()
    {
        image.color = default_color - new Color(20,20,20,0);
    }

    void OnGazeExit()
    {
        image.color = default_color;
    }


}
