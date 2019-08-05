//Based on the class originated from github.com/Microsoft/HoloToolkit-Unity/

using Scripts;
using UnityEngine.XR.WSA.Input;
using UnityEngine;

/// <summary>
/// HandsManager keeps track of when a hand is detected.
/// </summary>
public class HandsManager : Singleton<HandsManager>
{
    /// <summary>
    /// Tracks the hand detected state.
    /// </summary>
    public bool HandDetected
    {
        get;
        private set;
    }

    // Keeps track of the GameObject that the hand is interacting with.
    public GameObject FocusedGameObject { get; private set; }

    void Awake()
    {
        InteractionManager.InteractionSourceDetected += InteractionManager_InteractionSourceDetected;
        InteractionManager.InteractionSourceLost += InteractionManager_InteractionSourceLost;

        //Register for SourceManager.SourcePressed event.
        InteractionManager.InteractionSourcePressed += InteractionManager_InteractionSourcePressed;

        //Register for SourceManager.SourceReleased event.
        InteractionManager.InteractionSourceReleased += InteractionManager_InteractionSourceReleased;

        //Initialize FocusedGameObject as null.
        FocusedGameObject = null;
    }

    private void InteractionManager_InteractionSourceDetected(InteractionSourceDetectedEventArgs obj)
    {
        HandDetected = true;
    }

    private void InteractionManager_InteractionSourceLost(InteractionSourceLostEventArgs obj)
    {
        HandDetected = false;

        //Reset FocusedGameObject.
        ResetFocusedGameObject();
    }

    private void InteractionManager_InteractionSourcePressed(InteractionSourcePressedEventArgs hand)
    {
        if (InteractibleManager.Instance.FocusedGameObject != null)
        {
            //Cache InteractibleManager's FocusedGameObject in FocusedGameObject.
            FocusedGameObject = InteractibleManager.Instance.FocusedGameObject;
        }
    }

    private void InteractionManager_InteractionSourceReleased(InteractionSourceReleasedEventArgs hand)
    {
        //Reset FocusedGameObject.
        ResetFocusedGameObject();
    }

    private void ResetFocusedGameObject()
    {
        // Set FocusedGameObject to be null.
        FocusedGameObject = null;

        // On GestureManager call ResetGestureRecognizers
        // to complete any currently active gestures.
        GestureManager.Instance.ResetGestureRecognizers();
    }

    void OnDestroy()
    {
        InteractionManager.InteractionSourceDetected -= InteractionManager_InteractionSourceDetected;
        InteractionManager.InteractionSourceLost -= InteractionManager_InteractionSourceLost;

        // 2.a: Unregister the SourceManager.SourceReleased event.
        InteractionManager.InteractionSourceReleased -= InteractionManager_InteractionSourceReleased;

        // 2.a: Unregister for SourceManager.SourcePressed event.
        InteractionManager.InteractionSourcePressed -= InteractionManager_InteractionSourcePressed;
    }
}