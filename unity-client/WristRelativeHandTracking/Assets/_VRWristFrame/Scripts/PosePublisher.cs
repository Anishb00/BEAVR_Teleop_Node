using NetMQ;
using NetMQ.Sockets;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR.Hands;

public class PosePublisher : MonoBehaviour
{
    private PublisherSocket m_Publisher;
    private int m_FrameIndex = 0;
    private int m_EpisodeIndex = 0;
    private float m_EpisodeStartTime;

    [SerializeField]
    private string m_Address = "tcp://*:5555";

    // Only store right hand fingertip positions
    private Dictionary<string, Vector3> m_RightFingerTips = new Dictionary<string, Vector3>();

    void Start()
    {
        AsyncIO.ForceDotNet.Force();
        m_Publisher = new PublisherSocket();
        m_Publisher.Bind(m_Address);
        m_EpisodeStartTime = Time.time;
        Debug.Log("[MYLOG] I have instantiated the Pose Publisher");
    }

    public void OnJointUpdate(string handedness, string jointName, Vector3 relativePosition, Quaternion relativeRotation)
    {
        // Only process right hand data
        if (handedness != "Right")
            return;

        // Store fingertip positions
        if (jointName == "ThumbTip") m_RightFingerTips["thumb"] = relativePosition;
        else if (jointName == "IndexTip") m_RightFingerTips["index"] = relativePosition;
        else if (jointName == "MiddleTip") m_RightFingerTips["middle"] = relativePosition;
        else if (jointName == "RingTip") m_RightFingerTips["ring"] = relativePosition;
        else if (jointName == "LittleTip") m_RightFingerTips["little"] = relativePosition;
    }

    public void PublishHand(string handedness, Pose wristPose)
    {
        // Only publish right hand data
        if (handedness != "Right")
            return;

        if (m_RightFingerTips.Count < 5)
            return;

        string message = "{";
        message += $"\"episode_index\":{m_EpisodeIndex},";
        message += $"\"frame_index\":{m_FrameIndex},";
        message += $"\"timestamp\":{Time.time - m_EpisodeStartTime:F4},";
        message += "\"observation.state\":{";
        message += $"\"hand\":\"{handedness}\",";
        message += $"\"wrist_position\":[{wristPose.position.x:F4},{wristPose.position.y:F4},{wristPose.position.z:F4}],";
        message += $"\"wrist_rotation\":[{wristPose.rotation.x:F4},{wristPose.rotation.y:F4},{wristPose.rotation.z:F4},{wristPose.rotation.w:F4}],";
        message += "\"fingertips\":{";
        message += $"\"thumb\":[{m_RightFingerTips["thumb"].x:F4},{m_RightFingerTips["thumb"].y:F4},{m_RightFingerTips["thumb"].z:F4}],";
        message += $"\"index\":[{m_RightFingerTips["index"].x:F4},{m_RightFingerTips["index"].y:F4},{m_RightFingerTips["index"].z:F4}],";
        message += $"\"middle\":[{m_RightFingerTips["middle"].x:F4},{m_RightFingerTips["middle"].y:F4},{m_RightFingerTips["middle"].z:F4}],";
        message += $"\"ring\":[{m_RightFingerTips["ring"].x:F4},{m_RightFingerTips["ring"].y:F4},{m_RightFingerTips["ring"].z:F4}],";
        message += $"\"little\":[{m_RightFingerTips["little"].x:F4},{m_RightFingerTips["little"].y:F4},{m_RightFingerTips["little"].z:F4}]";
        message += "}},";

        float gripperValue = Vector3.Distance(m_RightFingerTips["thumb"], m_RightFingerTips["index"]);
        message += "\"action\":{";
        message += $"\"target_position\":[{wristPose.position.x:F4},{wristPose.position.y:F4},{wristPose.position.z:F4}],";
        message += $"\"target_rotation\":[{wristPose.rotation.x:F4},{wristPose.rotation.y:F4},{wristPose.rotation.z:F4},{wristPose.rotation.w:F4}],";
        message += $"\"gripper\":{gripperValue:F4}";
        message += "}}";

        m_Publisher.SendFrame(message);
        Debug.Log("[TestSubscriber] This is actually the pose publisher");
        m_FrameIndex++;
    }

    void OnDestroy()
    {
        m_Publisher?.Close();
        m_Publisher?.Dispose();
        NetMQConfig.Cleanup();
    }
}