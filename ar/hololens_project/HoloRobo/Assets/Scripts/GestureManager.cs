//Based on the class originated from github.com/Microsoft/HoloToolkit-Unity/

using Scripts;
using UnityEngine;


public class GestureManager : Singleton<GestureManager>
{
    public UnityEngine.XR.WSA.Input.GestureRecognizer HoverRecognizer { get; private set; }

    public UnityEngine.XR.WSA.Input.GestureRecognizer ManipulationRecognizer { get; private set; }

    public bool IsManipulating { get; private set; }

    public Vector3 ManipulationPosition { get; private set; }

    public UnityEngine.XR.WSA.Input.GestureRecognizer ActiveRecognizer { get; private set; }

    // public Source sc;

    void Awake()
    {
        HoverRecognizer = new UnityEngine.XR.WSA.Input.GestureRecognizer();

        HoverRecognizer.TappedEvent += (source, tapCount, ray) =>
        {
            if (InteractibleManager.Instance.FocusedGameObject != null)
            {
                InteractibleManager.Instance.FocusedGameObject.SendMessageUpwards("OnTapped");
            }
        };
        HoverRecognizer.StartCapturingGestures();

        // Instantiate the ManipulationRecognizer.
        ManipulationRecognizer = new UnityEngine.XR.WSA.Input.GestureRecognizer();

        // Add the ManipulationTranslate GestureSetting to the ManipulationRecognizer's RecognizableGestures.
        ManipulationRecognizer.SetRecognizableGestures(UnityEngine.XR.WSA.Input.GestureSettings.ManipulationTranslate);

        // Register for the Manipulation events on the ManipulationRecognizer.
        ManipulationRecognizer.ManipulationStartedEvent += ManipulationRecognizer_ManipulationStartedEvent;
        ManipulationRecognizer.ManipulationUpdatedEvent += ManipulationRecognizer_ManipulationUpdatedEvent;
        ManipulationRecognizer.ManipulationCompletedEvent += ManipulationRecognizer_ManipulationCompletedEvent;
        ManipulationRecognizer.ManipulationCanceledEvent += ManipulationRecognizer_ManipulationCanceledEvent;

        ResetGestureRecognizers();
    }

    void OnDestroy()
    {

        // Unregister the Manipulation events on the ManipulationRecognizer.
        ManipulationRecognizer.ManipulationStartedEvent -= ManipulationRecognizer_ManipulationStartedEvent;
        ManipulationRecognizer.ManipulationUpdatedEvent -= ManipulationRecognizer_ManipulationUpdatedEvent;
        ManipulationRecognizer.ManipulationCompletedEvent -= ManipulationRecognizer_ManipulationCompletedEvent;
        ManipulationRecognizer.ManipulationCanceledEvent -= ManipulationRecognizer_ManipulationCanceledEvent;
    }

    /// <summary>
    /// Revert back to the default GestureRecognizer.
    /// </summary>
    public void ResetGestureRecognizers()
    {
        // Default to the navigation gestures.
        Transition(null);
    }

    /// <summary>
    /// Transition to a new GestureRecognizer.
    /// </summary>
    /// <param name="newRecognizer">The GestureRecognizer to transition to.</param>
    public void Transition(UnityEngine.XR.WSA.Input.GestureRecognizer newRecognizer)
    {
        if (newRecognizer == null)
        {
            return;
        }

        if (ActiveRecognizer != null)
        {
            if (ActiveRecognizer == newRecognizer)
            {
                return;
            }

            ActiveRecognizer.CancelGestures();
            ActiveRecognizer.StopCapturingGestures();
        }

        newRecognizer.StartCapturingGestures();
        ActiveRecognizer = newRecognizer;
    }

    private void ManipulationRecognizer_ManipulationStartedEvent(UnityEngine.XR.WSA.Input.InteractionSourceKind source, Vector3 position, Ray ray)
    {
        if (HandsManager.Instance.FocusedGameObject != null)
        {
            IsManipulating = true;

            ManipulationPosition = position;

            HandsManager.Instance.FocusedGameObject.SendMessageUpwards("PerformManipulationStart", position);
        }
    }

    private void ManipulationRecognizer_ManipulationUpdatedEvent(UnityEngine.XR.WSA.Input.InteractionSourceKind source, Vector3 position, Ray ray)
    {
        if (HandsManager.Instance.FocusedGameObject != null)
        {
            IsManipulating = true;

            ManipulationPosition = position;

            HandsManager.Instance.FocusedGameObject.SendMessageUpwards("PerformManipulationUpdate", position);
        }
    }

    private void ManipulationRecognizer_ManipulationCompletedEvent(UnityEngine.XR.WSA.Input.InteractionSourceKind source, Vector3 position, Ray ray)
    {
        IsManipulating = false;
    }

    private void ManipulationRecognizer_ManipulationCanceledEvent(UnityEngine.XR.WSA.Input.InteractionSourceKind source, Vector3 position, Ray ray)
    {
        IsManipulating = false;
    }
}
