using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR.Hands;

public class WristRelativeHandData : MonoBehaviour
{
    // Added handedness to the event
    public event Action<string, string, Vector3, Quaternion> OnWristRelativeJointUpdate;
    private PosePublisher m_PosePublisher;
    private XRHandSubsystem m_Subsystem;

    void Update()
    {
        if (m_Subsystem == null || !m_Subsystem.running)
        {
            var subsystems = new List<XRHandSubsystem>();
            SubsystemManager.GetSubsystems(subsystems);

            foreach (var subsystem in subsystems)
            {
                if (subsystem.running)
                {
                    m_Subsystem = subsystem;
                    m_Subsystem.updatedHands += OnUpdatedHands;
                    break;
                }
            }
        }
    }

    void OnUpdatedHands(XRHandSubsystem subsystem,
        XRHandSubsystem.UpdateSuccessFlags updateSuccessFlags,
        XRHandSubsystem.UpdateType updateType)
    {
        if (updateType != XRHandSubsystem.UpdateType.Dynamic)
            return;

        // Only processing right hand for now until able to perform bimanual teleoperation
        ProcessHand(subsystem.rightHand, "Right");
        //ProcessHand(subsystem.leftHand, "Left");
    }

    void ProcessHand(XRHand hand, string handedness)
    {
        if (!hand.isTracked)
            return;

        var wristJoint = hand.GetJoint(XRHandJointID.Wrist);
        if (!wristJoint.TryGetPose(out var wristPose))
            return;

        Quaternion inverseWristRotation = Quaternion.Inverse(wristPose.rotation);

        m_PosePublisher = GetComponent<PosePublisher>();

        // Creates wrist-centered displacement vectors and then applies inverse wrist rotation
        // to express vectors and orientation data in wrist frame
        for (int i = XRHandJointID.BeginMarker.ToIndex(); i < XRHandJointID.EndMarker.ToIndex(); i++)
        {
            var jointId = XRHandJointIDUtility.FromIndex(i);
            var joint = hand.GetJoint(jointId);

            if (!joint.TryGetPose(out var jointPose))
                continue;

            Vector3 wristToJoint_W = jointPose.position - wristPose.position;
            Vector3 relativePosition = inverseWristRotation * wristToJoint_W;      

            Quaternion relativeRotation = inverseWristRotation * jointPose.rotation;

            // Update hand labels
            OnWristRelativeJointUpdate?.Invoke(handedness, jointId.ToString(), relativePosition, relativeRotation);
            
            // Send to NetMQ publisher 
            if (m_PosePublisher != null)
            {
                m_PosePublisher.OnJointUpdate(handedness, jointId.ToString(), relativePosition, relativeRotation);
            }
        }

        // Publish complete hand data after all joints processed
        if (m_PosePublisher != null)
        {
            m_PosePublisher.PublishHand(handedness, wristPose);
        }
    }

    void OnDestroy()
    {
        if (m_Subsystem != null)
        {
            m_Subsystem.updatedHands -= OnUpdatedHands;
        }
    }
}