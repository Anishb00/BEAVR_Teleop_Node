using UnityEngine;

public class JointLabelDisplay : MonoBehaviour
{
    private GameObject labelObject;
    private TextMesh labelText;
    private WristRelativeHandData wristRelativeData;
    private string myJointName;
    private string myHandedness;
    private Vector3 currentRelativePosition;
    private bool hasData = false;

    void Start()
    {
        labelObject = new GameObject("PositionLabel");
        labelObject.transform.SetParent(transform);
        labelObject.transform.localPosition = new Vector3(0.02f, 0.02f, 0);
        labelObject.transform.localScale = new Vector3(0.003f, 0.003f, 0.003f);

        labelText = labelObject.AddComponent<TextMesh>();
        labelText.fontSize = 50;
        labelText.anchor = TextAnchor.MiddleCenter;
        labelText.color = Color.white;

        myJointName = gameObject.name;

        // Determine handedness from parent hierarchy
        Transform parent = transform.parent;
        while (parent != null)
        {
            if (parent.name.Contains("Left"))
            {
                myHandedness = "Left";
                break;
            }
            else if (parent.name.Contains("Right"))
            {
                myHandedness = "Right";
                break;
            }
            parent = parent.parent;
        }

        Debug.Log($"[MyLog] Joint: {myJointName}, Hand: {myHandedness}");
    }

    void Update()
    {
        if (wristRelativeData == null)
        {
            wristRelativeData = FindFirstObjectByType<WristRelativeHandData>();
            if (wristRelativeData != null)
            {
                wristRelativeData.OnWristRelativeJointUpdate += HandleWristRelativeUpdate;
            }
        }
    }

    void HandleWristRelativeUpdate(string handedness, string jointName, Vector3 relativePosition, Quaternion relativeRotation)
    {
        // Match both handedness AND joint name
        if (myHandedness == handedness && myJointName == jointName)
        {
            if (myJointName == "IndexTip" || myJointName == "Wrist" || myJointName == "IndexProximal" || myJointName == "IndexIntermediate" || myJointName == "IndexDistal" || myJointName == "IndexMetacarpal") 
            {
                currentRelativePosition = relativePosition;
                hasData = true;
            }
        }
    }

    void LateUpdate()
    {
        if (labelText != null)
        {
            if (hasData)
            {
                labelText.text = $"({currentRelativePosition.x:F2}, {currentRelativePosition.y:F2}, {currentRelativePosition.z:F2})";
            }
            else
            {
                labelText.text = "...";
            }

            if (Camera.main != null)
            {
                labelText.transform.rotation = Quaternion.LookRotation(
                    labelText.transform.position - Camera.main.transform.position);
            }
        }
    }

    void OnDestroy()
    {
        if (wristRelativeData != null)
        {
            wristRelativeData.OnWristRelativeJointUpdate -= HandleWristRelativeUpdate;
        }

        if (labelObject != null)
            Destroy(labelObject);
    }
}