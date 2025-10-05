using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ChangeRobotPose : MonoBehaviour
{
    public Transform robotShoulderLink;
    public Transform robotUpperArmLink;
    public Transform robotForearmLink;
    public Transform robotWrist1Link;
    public Transform robotWrist2Link;
    public Transform robotWrist3Link;

    public void SetHomePose()
    {
        // Define the desired rotation for each joint in local space
        if (robotShoulderLink != null)
            robotShoulderLink.localRotation = Quaternion.Euler(0, 90, 0);
            
        if (robotUpperArmLink != null)
            robotUpperArmLink.localRotation = Quaternion.Euler(60, 0, -90);
            
        if (robotForearmLink != null)
            robotForearmLink.localRotation = Quaternion.Euler(0, -120, 0);
            
        if (robotWrist1Link != null)
            robotWrist1Link.localRotation = Quaternion.Euler(0, 60, 0);
            
        if (robotWrist2Link != null)
            robotWrist2Link.localRotation = Quaternion.Euler(-90, 0, -90);
            
        if (robotWrist3Link != null)
            robotWrist3Link.localRotation = Quaternion.Euler(90, 0, 90);
    }
}

