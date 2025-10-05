/* using UnityEngine;
using UnityEngine.XR;

public class SetHomePose : MonoBehaviour
{
    // Assign these in the Unity Inspector or find them dynamically in the script.
    public ArticulationBody shoulderPanJoint;
    public ArticulationBody shoulderLiftJoint;
    public ArticulationBody elbowJoint;
    public ArticulationBody wrist1Joint;
    public ArticulationBody wrist2Joint;
    public ArticulationBody wrist3Joint;

    void Start()
    {
        SetHomePoseValues();
    }

    void SetHomePoseValues()
    {
        // Set the joint angles to match the home pose (values from SRDF)
        SetJointAngle(shoulderPanJoint, 0.0f);      // robot_shoulder_pan_joint = 0
        SetJointAngle(shoulderLiftJoint, -1.3012f); // robot_shoulder_lift_joint = -1.3012
        SetJointAngle(elbowJoint, 0.0f);            // robot_elbow_joint = 0
        SetJointAngle(wrist1Joint, 0.3346f);        // robot_wrist_1_joint = 0.3346
        SetJointAngle(wrist2Joint, 1.55f);          // robot_wrist_2_joint = 1.55
        SetJointAngle(wrist3Joint, 1.52f);          // robot_wrist_3_joint = 1.52
    }

    void SetJointAngle(ArticulationBody joint, float angle)
    {
        // Apply the angle to the joint
        ArticulationDrive drive = joint.xDrive;
        drive.target = angle; // Set the target angle
        joint.xDrive = drive; // Apply the change
    }
} */



/* using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SetHomePose : MonoBehaviour
{
    public Transform robotShoulderLink;
    public Transform robotUpperArmLink;
    public Transform robotForearmLink;
    public Transform robotWrist1Link;
    public Transform robotWrist2Link;
    public Transform robotWrist3Link;

    // Define the home rotation angles for each joint
    private Vector3 shoulderHomeRotation = new Vector3(0, 0, 0); 
    private Vector3 upperArmHomeRotation = new Vector3(70, 0, -90); 
    private Vector3 forearmHomeRotation = new Vector3(0, 0, 0); 
    private Vector3 wrist1HomeRotation = new Vector3(0, -20, 0); 
    private Vector3 wrist2HomeRotation = new Vector3(-90, 0, -90); 
    private Vector3 wrist3HomeRotation = new Vector3(90, 0, 90); 

    void LateUpdate()
    {
        // Set each joint's rotation to the defined home pose when play starts
        ApplyHomePose();
    }

    void ApplyHomePose()
    {
        if (robotShoulderLink)
        {
            robotShoulderLink.localRotation = Quaternion.Euler(shoulderHomeRotation);
            Debug.Log("Shoulder set to: " + robotShoulderLink.localRotation.eulerAngles);
        }
        if (robotUpperArmLink)
        {
            robotUpperArmLink.localRotation = Quaternion.Euler(upperArmHomeRotation);
            Debug.Log("Upper Arm set to: " + robotUpperArmLink.localRotation.eulerAngles);
        }
        if (robotForearmLink)
        {
            robotForearmLink.localRotation = Quaternion.Euler(forearmHomeRotation);
            Debug.Log("Forearm set to: " + robotForearmLink.localRotation.eulerAngles);
        }
        if (robotWrist1Link)
        {
            robotWrist1Link.localRotation = Quaternion.Euler(wrist1HomeRotation);
            Debug.Log("Wrist1 set to: " + robotWrist1Link.localRotation.eulerAngles);
        }
        if (robotWrist2Link)
        {
            robotWrist2Link.localRotation = Quaternion.Euler(wrist2HomeRotation);
            Debug.Log("Wrist2 set to: " + robotWrist2Link.localRotation.eulerAngles);
        }
        if (robotWrist3Link)
        {
            robotWrist3Link.localRotation = Quaternion.Euler(wrist3HomeRotation);
            Debug.Log("Wrist3 set to: " + robotWrist3Link.localRotation.eulerAngles);
        }
    }
} */



/* using UnityEngine;
using System.Collections;

public class SetHomePose : MonoBehaviour
{
    public ArticulationBody robotShoulderLink;
    public ArticulationBody robotUpperArmLink;
    public ArticulationBody robotForearmLink;
    public ArticulationBody robotWrist1Link;
    public ArticulationBody robotWrist2Link;
    public ArticulationBody robotWrist3Link;

    // Define the home positions for each joint (in degrees)
    private float shoulderHomePosition = 0f;
    private float upperArmHomePosition = 70f;
    private float forearmHomePosition = 0f;
    private float wrist1HomePosition = -20f;
    private float wrist2HomePosition = -90f;
    private float wrist3HomePosition = 90f;

    public CustomController controller;
    public bool homePoseSet = false;

    void Start()
    {
        StartCoroutine(ApplyHomePoseWithDelay());
    }

    IEnumerator ApplyHomePoseWithDelay()
    {
        yield return new WaitForSeconds(0.1f);
        ApplyHomePose();
        homePoseSet = true;
        if (controller != null)
        {
            controller.enabled = true;
        }
    }

    void ApplyHomePose()
    {
        SetJointPosition(robotShoulderLink, shoulderHomePosition);
        SetJointPosition(robotUpperArmLink, upperArmHomePosition);
        SetJointPosition(robotForearmLink, forearmHomePosition);
        SetJointPosition(robotWrist1Link, wrist1HomePosition);
        SetJointPosition(robotWrist2Link, wrist2HomePosition);
        SetJointPosition(robotWrist3Link, wrist3HomePosition);
        Debug.Log("Applying home pose");
    }

    void SetJointPosition(ArticulationBody joint, float position)
    {
        if (joint != null)
        {
            ArticulationDrive drive = joint.xDrive;
            drive.target = position;
            joint.xDrive = drive;
            Debug.Log($"Setting {joint.name} to position: {position}");
        }
    }
} */
