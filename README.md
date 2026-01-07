# BEAVR_Teleop_Node
This Project will process wrist frame data provided by Meta Quest 3S to teleoperate humanoid robotic hands


## Unity Client Overview

This repository contains a Unity-based VR client for wrist-relative hand tracking and teleoperation.

### Unity Client Location

The Unity project is located at:


```
BEAVR_Teleop_node/
└── unity-client/
    └── WristRelativeHandTracking/
```


Open the project by selecting the `WristRelativeHandTracking/` directory in Unity Hub.

---

## Project Structure

All project-specific Unity content (scripts, scenes, prefabs, and assets) is organized under a single root folder:

```
BEAVR_Teleop_node/unity-client/WristRelativeHandTracking/Assets/_VRWristFrame/
```

## VR Wrist-Relative Hand Tracking Output

The Unity VR client publishes wrist-relative hand tracking data from the headset over the network for downstream teleoperation.

### Network Interface

- **Transport:** TCP
- **Host:** VR headset
- **Port:** `5555`
- **Direction:** Unity VR client → external teleoperation system (e.g., ROS)

---

## Hand Support Status

- ✅ **Currently published:** **Right hand only**
- ❌ **Left hand:** not yet supported

Although the data schema supports both hands, teleoperation currently only works with the right hand, so only right-hand data is actively published and consumed downstream.

---

## Data Format & Schema

Each frame is published as a single JSON record representing the current wrist pose and fingertip positions in a wrist-relative coordinate frame.

### Example Record

```json
{
  "episode_index": 0,
  "frame_index": 0,
  "timestamp": 0.0000,
  "observation.state": {
    "hand": "Left",
    "wrist_position": [0.4062, 0.7112, -0.2865],
    "wrist_rotation": [-0.5652, -0.3380, -0.6873, -0.3064],
    "fingertips": {
      "thumb":  [ 0.0939, -0.0305,  0.0983 ],
      "index":  [ 0.0363, -0.0263,  0.1798 ],
      "middle": [ -0.0079, -0.0472,  0.1820 ],
      "ring":   [ -0.0384, -0.0483,  0.1679 ],
      "little": [ -0.0771, -0.0440,  0.1318 ]
    }
  },
  "action": {
    "target_position": [0.4062, 0.7112, -0.2865],
    "target_rotation": [-0.5652, -0.3380, -0.6873, -0.3064],
    "gripper": 0.0999
  }
}



