using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR.Hands;

public class HandTrackingData : MonoBehaviour
{
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    XRHandSubsystem m_HandSubsystem;

    void Start()
    {
        var handSubsystems = new List<XRHandSubsystem>();
        SubsystemManager.GetSubsystems(handSubsystems);

        for (var i = 0; i < handSubsystems.Count; i++) 
        {
            XRHandSubsystem handSubsystem = handSubsystems[i];
            if (handSubsystem.running) 
            {
                m_HandSubsystem = handSubsystem;
            }
        }
        
        //Subscribes onUpdateHands method to run on every hand update
        if (m_HandSubsystem != null)
        {
            m_HandSubsystem.updatedHands += OnUpdateHands;
        }
    }

    void OnUpdateHands(XRHandSubsystem subsystem,
        XRHandSubsystem.UpdateSuccessFlags updateSuccessFlags,
        XRHandSubsystem.UpdateType updateType) 
    {
        XRHand hand = subsystem.leftHand;
        //XRHand hand = subsystem.rightHand;
        for (var i = XRHandJointID.BeginMarker.ToIndex(); i < XRHandJointID.EndMarker.ToIndex(); i++) 
        {
            var trackingData = hand.GetJoint(XRHandJointIDUtility.FromIndex(i));

            if (trackingData.TryGetPose(out Pose pose))
            {
                // displayTransform is some GameObject's Transform component
                Debug.Log($"[MyLog] Position: {pose.position}");
                Debug.Log($"[MyLog] Rotation: {pose.rotation}");
            }
        }
    }
}
