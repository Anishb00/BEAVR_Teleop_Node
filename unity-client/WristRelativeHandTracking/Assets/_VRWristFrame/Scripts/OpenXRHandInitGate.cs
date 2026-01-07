using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SubsystemsImplementation;
using UnityEngine.XR;
using UnityEngine.XR.Hands;

public class OpenXRHandInitGate : MonoBehaviour
{
    [Tooltip("Things to enable only after hand tracking is actually running/tracked.")]
    public GameObject[] enableWhenReady;

    [Tooltip("Seconds before giving up and enabling anyway (optional).")]
    public float timeoutSeconds = 8f;

    void Awake()
    {
        // Start disabled so you don't get 'random' visuals depending on timing.
        foreach (var go in enableWhenReady)
            if (go) go.SetActive(false);
    }
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    IEnumerator Start()
    {
        float start = Time.realtimeSinceStartup;

        // 1) Ensure XR is running
        yield return new WaitUntil(() => XRSettings.isDeviceActive || Time.realtimeSinceStartup - start > timeoutSeconds);

        // 2) Get the XRHandSubsystem (provided by XR Hands + OpenXR Hand Tracking feature)
        XRHandSubsystem handSubsystem = null;

        while (handSubsystem == null && Time.realtimeSinceStartup - start < timeoutSeconds)
        {
            var subsystems = new List<XRHandSubsystem>();
            SubsystemManager.GetSubsystems(subsystems);
            if (subsystems.Count > 0)
                handSubsystem = subsystems[0];

            yield return null;
        }

        // 3) Make sure it's running
        if (handSubsystem != null && !handSubsystem.running)
            handSubsystem.Start();

        // 4) Wait until at least one hand is tracked (so visuals/joints won't be null/identity)
        bool anyTracked = false;
        while (!anyTracked && Time.realtimeSinceStartup - start < timeoutSeconds)
        {
            if (handSubsystem != null)
            {
                anyTracked =
                    handSubsystem.leftHand.isTracked ||
                    handSubsystem.rightHand.isTracked;
            }
            yield return null;
        }

        // 5) Enable your hand visualizer(s)
        foreach (var go in enableWhenReady)
            if (go) go.SetActive(true);
    }
}


