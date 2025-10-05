using System;
using RosMessageTypes.Geometry;
using RosMessageTypes.Ur10eRg2Moveit;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.UrdfImporter;
using UnityEngine;

public class SourceDestinationPublisher : MonoBehaviour
{
    const int k_NumRobotJoints = 6;
    public static readonly string[] LinkNames =
    {
        "world/robot_base_link/robot_base_link_inertia/robot_shoulder_link",
        "/robot_upper_arm_link",
        "/robot_forearm_link",
        "/robot_wrist_1_link",
        "/robot_wrist_2_link",
        "/robot_wrist_3_link"
    };

    // Variables required for ROS communication
    [SerializeField]
    string m_TopicName = "/ur10e_rg2_joints";

    [SerializeField]
    GameObject m_UR10e;

    [SerializeField]
    GameObject m_Target;

    [SerializeField]
    GameObject m_TargetPlacement;

    readonly Quaternion m_PickOrientation = Quaternion.Euler(90, 90, 0);

    // Robot Joints
    UrdfJointRevolute[] m_JointArticulationBodies;

    // ROS Connector
    ROSConnection m_Ros;

	void Start()
    {

        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
	    m_Ros.RegisterPublisher<Ur10eMoveitJointsMsg>(m_TopicName);

        if (m_UR10e == null)
        {
            Debug.LogError("m_UR10e is not assigned in the Inspector!");
            return;
        }


        m_JointArticulationBodies = new UrdfJointRevolute[k_NumRobotJoints];

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += LinkNames[i];
            m_JointArticulationBodies[i] = m_UR10e.transform.Find(linkName).GetComponent<UrdfJointRevolute>();
        }
    }

    /*void Start()
	{
		// Get ROS connection static instance
		m_Ros = ROSConnection.GetOrCreateInstance();
		Debug.Log("m_Ros: " + (m_Ros != null)); // Check if m_Ros is initialized
		m_Ros.RegisterPublisher<Ur10eMoveitJointsMsg>(m_TopicName);

		// Ensure m_UR10e is assigned in the Inspector
		Debug.Log("m_UR10e: " + (m_UR10e != null));

		m_JointArticulationBodies = new UrdfJointRevolute[k_NumRobotJoints];
		
		var linkName = string.Empty;
		for (var i = 0; i < k_NumRobotJoints; i++)
		{
			linkName += LinkNames[i];
			var joint = m_UR10e.transform.Find(linkName);
			
			// Check if joint object was found
			Debug.Log("Link " + LinkNames[i] + " found: " + (joint != null));
			
			if (joint != null)
			{
				m_JointArticulationBodies[i] = joint.GetComponent<UrdfJointRevolute>();
				// Check if the component was attached
				Debug.Log("UrdfJointRevolute for link " + LinkNames[i] + ": " + (m_JointArticulationBodies[i] != null));
			}
			else
			{
				Debug.LogWarning("Link " + LinkNames[i] + " not found in m_UR10e.");
			}
		}
	}*/


    public void Publish()
    {
        if (m_UR10e == null || m_Target == null || m_TargetPlacement == null)
        {
            Debug.LogError("Assign all references in Inspector!");
            return;
        }

        Debug.Log("Button clicked! m_UR10e: " + m_UR10e.name);
    }



    //  public void Publish()
    //  {
    //      var sourceDestinationMessage = new Ur10eMoveitJointsMsg();

    //      for (var i = 0; i < k_NumRobotJoints; i++) 
    //{
    //	if (m_JointArticulationBodies[i] != null)
    //	{
    //		sourceDestinationMessage.joints[i] = m_JointArticulationBodies[i].GetPosition();
    //	}
    //	else
    //	{
    //		Debug.LogError("Joint at index " + i + " is null. Please check joint setup.");
    //	}
    //}


    //      sourceDestinationMessage.joints = new double[k_NumRobotJoints];
    //      for (var i = 0; i < k_NumRobotJoints; i++)
    //      {
    //          sourceDestinationMessage.joints[i] = m_JointArticulationBodies[i].GetPosition();
    //      }

    //      // Pick Pose
    //      sourceDestinationMessage.pick_pose = new PoseMsg
    //      {
    //          position = m_Target.transform.position.To<FLU>(),
    //          orientation = Quaternion.Euler(90, m_Target.transform.eulerAngles.y, 0).To<FLU>()
    //      };

    //      // Place Pose
    //      sourceDestinationMessage.place_pose = new PoseMsg
    //      {
    //          position = m_TargetPlacement.transform.position.To<FLU>(),
    //          orientation = m_PickOrientation.To<FLU>()
    //      };

    //      // Finally send the message to server_endpoint.py running in ROS
    //      m_Ros.Publish(m_TopicName, sourceDestinationMessage);
    //  }
}
