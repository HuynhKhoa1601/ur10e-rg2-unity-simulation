/*using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.UrdfImporter.Control;

public class GoToHome : MonoBehaviour
{
    private Controller robotController;
    private ArticulationBody[] joints;
    private bool isMovingHome = false;
    private float moveSpeed = 30f;
    private float positionThreshold = 0.5f;

    // Joint control parameters - matching the Controller script
    private float stiffness = 10000f;
    private float damping = 100f;
    private float forceLimit = 1000f;

    // Default starting positions (adjust these values based on your robot's correct home position)
    private readonly float[] defaultTargetRotations = new float[]
    {
        0f,     // shoulder
        -45f,   // upper arm
        90f,    // forearm
        -45f,   // wrist 1
        -90f,   // wrist 2
        0f      // wrist 3
    };

    void Start()
    {
        robotController = GetComponent<Controller>();
        joints = GetComponentsInChildren<ArticulationBody>();
        SetupJoints();
    }

    private void SetupJoints()
    {
        int jointIndex = 0;
        foreach (ArticulationBody joint in joints)
        {
            if (joint.name.Contains("robot_") && !joint.name.Contains("base"))
            {
                ArticulationDrive drive = joint.xDrive;
                
                // Set the drive parameters
                drive.stiffness = stiffness;
                drive.damping = damping;
                drive.forceLimit = forceLimit;
                
                // Set the initial target
                if (jointIndex < defaultTargetRotations.Length)
                {
                    drive.target = defaultTargetRotations[jointIndex];
                    jointIndex++;
                }

                // Make sure we're using revolute joints
                joint.jointType = ArticulationJointType.RevoluteJoint;
                
                // Set the drive limits (matching the original controller's limits)
                drive.upperLimit = 180f;
                drive.lowerLimit = -180f;
                
                joint.xDrive = drive;
            }
        }
    }

    void Update()
    {
        if (isMovingHome)
        {
            bool allJointsInPosition = true;
            int jointIndex = 0;

            foreach (ArticulationBody joint in joints)
            {
                if (joint.name.Contains("robot_") && !joint.name.Contains("base"))
                {
                    if (jointIndex < defaultTargetRotations.Length)
                    {
                        ArticulationDrive drive = joint.xDrive;
                        float currentRotation = drive.target;
                        float targetRotation = defaultTargetRotations[jointIndex];

                        if (Mathf.Abs(currentRotation - targetRotation) > positionThreshold)
                        {
                            float newRotation = Mathf.MoveTowards(currentRotation, targetRotation, moveSpeed * Time.deltaTime);
                            drive.target = newRotation;
                            joint.xDrive = drive;
                            allJointsInPosition = false;
                        }

                        jointIndex++;
                    }
                }
            }

            if (allJointsInPosition)
            {
                isMovingHome = false;
                Debug.Log("Robot has reached home position");
                robotController.enabled = true;
            }
        }
    }

    public void MoveToHome()
    {
        robotController.enabled = false;
        isMovingHome = true;

        int jointIndex = 0;
        foreach (ArticulationBody joint in joints)
        {
            if (joint.name.Contains("robot_") && !joint.name.Contains("base"))
            {
                if (jointIndex < defaultTargetRotations.Length)
                {
                    ArticulationDrive drive = joint.xDrive;
                    drive.target = defaultTargetRotations[jointIndex];
                    joint.xDrive = drive;
                    jointIndex++;
                }
            }
        }
    }

    public bool IsInHomePosition()
    {
        return !isMovingHome;
    }
}*/
