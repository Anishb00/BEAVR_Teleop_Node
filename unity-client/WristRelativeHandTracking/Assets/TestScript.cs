using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR.Hands;

public class TestScript : MonoBehaviour
{
    XRHandSubsystem m_Subsystem;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        var subsystems = new List<XRHandSubsystem>();
        SubsystemManager.GetSubsystems(subsystems);
        m_Subsystem = subsystems[0];
        Debug.Log(m_Subsystem);
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
