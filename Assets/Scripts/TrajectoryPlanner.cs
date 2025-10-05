using System;
using System.Collections;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.Ur10eRg2Moveit;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class TrajectoryPlanner : MonoBehaviour
{
    // Hardcoded variables
    const int k_NumRobotJoints = 6;
    const float k_JointAssignmentWait = 0.1f;
    const float k_PoseAssignmentWait = 0.5f;
    private const int isBigEndian = 0;
    private const int step = 4;
    private readonly Vector3 pickPoseOffset = new Vector3(0, 0.255f, 0);
    private readonly Vector3 placePoseOffset = new Vector3(0, 0.275f, 0);
    private readonly Quaternion pickOrientation = new Quaternion(-0.5f, -0.5f, 0.5f, -0.5f);

    // Variables required for ROS communication
    [SerializeField]
    string m_RosServiceName = "pose_estimation_srv";
    public string RosServiceName { get => m_RosServiceName; set => m_RosServiceName = value; }

    [SerializeField]
    GameObject m_UR10e;
    public GameObject UR10e { get => m_UR10e; set => m_UR10e = value; }
    [SerializeField]
    GameObject m_Target;
    public GameObject Target { get => m_Target; set => m_Target = value; }
    [SerializeField]
    GameObject m_TargetPlacement;
    public GameObject TargetPlacement { get => m_TargetPlacement; set => m_TargetPlacement = value; }

    // Assures that the gripper is always positioned to the side of the m_Target cube before grasping.
    readonly Quaternion m_PickOrientation = Quaternion.Euler(90, 90, 0);
    readonly Vector3 m_PickPoseOffset = Vector3.up * 0.1f;

    // Articulation Bodies
    ArticulationBody[] m_JointArticulationBodies;
    ArticulationBody m_LeftOuterGripper;
    ArticulationBody m_RightOuterGripper;

    // ROS Connector
    ROSConnection m_Ros;
    public Transform goal;

    // UI elements
    private Button InitializeButton;
    private Button RandomizeButton;
    private Button ServiceButton;
    private TMP_Text ActualPos;
    private TMP_Text ActualRot;
    private TMP_Text EstimatedPos;
    private TMP_Text EstimatedRot;
    private RenderTexture renderTexture;

    void Start()
    {
        if (m_UR10e == null) Debug.LogError("m_UR10e is not assigned!");
        Debug.Log("Initializing ROS connection...");
        
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterRosService<MoverServiceRequest, MoverServiceResponse>(m_RosServiceName);
        Debug.Log($"ROS Service Name: {m_RosServiceName}");
        m_Ros.RegisterRosService<PoseEstimationServiceRequest, PoseEstimationServiceResponse>("pose_estimation_srv");


        Debug.Log("Finding robot joint articulation bodies...");
        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += SourceDestinationPublisher.LinkNames[i];
            var articulationBody = m_UR10e.transform.Find(linkName).GetComponent<ArticulationBody>();
            if (articulationBody != null)
            {
                m_JointArticulationBodies[i] = articulationBody;
                Debug.Log($"Found articulation body for joint {i}: {linkName}");
            }
            else
            {
                Debug.LogError($"Failed to find articulation body for joint {i}: {linkName}");
            }
        }

        // Find the gripper joint
        var gripperJointPath = linkName + "/robot_flange/robot_tool0/gripper_onrobot_rg2_base_link";

        var rightOuterGripper = gripperJointPath + "/gripper_right_outer_knuckle/gripper_right_inner_finger";
        var leftOuterGripper = gripperJointPath + "/gripper_left_outer_knuckle/gripper_left_inner_finger";

        m_RightOuterGripper = m_UR10e.transform.Find(rightOuterGripper).GetComponent<ArticulationBody>();
        m_LeftOuterGripper = m_UR10e.transform.Find(leftOuterGripper).GetComponent<ArticulationBody>();

        if (m_RightOuterGripper != null && m_LeftOuterGripper != null)
        {
            Debug.Log($"Found gripper joint: {rightOuterGripper} and {leftOuterGripper}");
        }
        else
        {
            Debug.LogError($"Failed to find gripper joint at path: {rightOuterGripper} and {leftOuterGripper}");
        }




        // Assign UI elements
        //InitializeButton = GameObject.Find("ROSObjects/Canvas/ButtonPanel/DefaultButton").GetComponent<Button>();   
        //RandomizeButton = GameObject.Find("ROSObjects/Canvas/ButtonPanel/RandomButton").GetComponent<Button>();
        ServiceButton = GameObject.Find("Canvas/ServiceButton")?.GetComponent<Button>();
        GameObject positionPanel = GameObject.Find("Canvas/PositionPanel");
        ActualPos = positionPanel.transform.Find("ActualPosField")?.GetComponent<TMP_Text>();
        ActualRot = positionPanel.transform.Find("ActualRotField")?.GetComponent<TMP_Text>();
        EstimatedPos = positionPanel.transform.Find("EstPosField")?.GetComponent<TMP_Text>();
        EstimatedRot = positionPanel.transform.Find("EstRotField")?.GetComponent<TMP_Text>();

        if (Target == null) Debug.LogError("Target not assigned!");

        if (ServiceButton == null) Debug.LogError("ServiceButton not found!");
        if (ActualPos == null) Debug.LogError("ActualPos Text not found!");
        if (ActualRot == null) Debug.LogError("ActualRot Text not found!");
        if (EstimatedPos == null) Debug.LogError("EstimatedPos Text not found!");
        if (EstimatedRot == null) Debug.LogError("EstimatedRot Text not found!");

        // Initialize UI element values
        ActualPos.text = Target.transform.position.ToString();
        ActualRot.text = Target.transform.eulerAngles.ToString();
        EstimatedPos.text = "-";
        EstimatedRot.text = "-";

        // --- Initialize the render texture for camera capture ---
        renderTexture = new RenderTexture(Camera.main.pixelWidth, Camera.main.pixelHeight, 24,
                                          UnityEngine.Experimental.Rendering.GraphicsFormat.R8G8B8A8_SRGB);
        renderTexture.Create();
        Debug.Log("RenderTexture initialized for pose estimation.");
    }

    /// <summary>
    ///     Get the current values of the robot's joint angles.
    /// </summary>
    /// <returns>Ur10eMoveitJointsMsg</returns>
    Ur10eMoveitJointsMsg CurrentJointConfig()
    {
        var joints = new Ur10eMoveitJointsMsg();

        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            joints.joints[i] = m_JointArticulationBodies[i].jointPosition[0];
            Debug.Log($"Joint {i} position: {joints.joints[i]}");
        }

        return joints;
    }

    /// <summary>
    ///     Create a new MoverServiceRequest with the current values of the robot's joint angles,
    ///     the target cube's current position and rotation, and the targetPlacement position and rotation.
    ///     Call the MoverService using the ROSConnection and if a trajectory is successfully planned,
    ///     execute the trajectories in a coroutine.
    /// </summary>
    public void PublishJoints()
    {
        var request = new MoverServiceRequest();
        request.joints_input = CurrentJointConfig();

        // Pick Pose
        request.pick_pose = new PoseMsg
        {
            position = (Target.transform.position + m_PickPoseOffset).To<FLU>(),
            // The hardcoded x/z angles assure that the gripper is always positioned above the target cube before grasping.
            orientation = Quaternion.Euler(90, Target.transform.eulerAngles.y, 0).To<FLU>()
        };

        Debug.Log("Pick Pose:");
        Debug.Log($"Unity Target Position: {Target.transform.position}");
        Debug.Log($"Offset: {m_PickPoseOffset}");
        Debug.Log($"Unity Target Offset Position: {Target.transform.position + m_PickPoseOffset}");
        Debug.Log($"ROS Target Position - X: {request.pick_pose.position.x}, Y: {request.pick_pose.position.y}, Z: {request.pick_pose.position.z}");
        Debug.Log($"ROS Target Orientation - X: {request.pick_pose.orientation.x}, Y: {request.pick_pose.orientation.y}, Z: {request.pick_pose.orientation.z}, W: {request.pick_pose.orientation.w}");

        // Place Pose
        request.place_pose = new PoseMsg
        {
            position = (TargetPlacement.transform.position + m_PickPoseOffset).To<FLU>(),
            orientation = m_PickOrientation.To<FLU>()
        };

        Debug.Log("Place Pose:");
        Debug.Log($"Unity Placement Position: {TargetPlacement.transform.position}");
        Debug.Log($"Offset: {m_PickPoseOffset}");
        Debug.Log($"Pick Orientation: x: {m_PickOrientation.x}, y: {m_PickOrientation.y}, z: {m_PickOrientation.z}, w: {m_PickOrientation.w}");
        Debug.Log($"Unity Placement Offset Position: {TargetPlacement.transform.position + m_PickPoseOffset}");
        Debug.Log($"ROS Placement Position - X: {request.place_pose.position.x}, Y: {request.place_pose.position.y}, Z: {request.place_pose.position.z}");
        Debug.Log($"ROS Placement Orientation - X: {request.place_pose.orientation.x}, Y: {request.place_pose.orientation.y}, Z: {request.place_pose.orientation.z}, W: {request.place_pose.orientation.w}");

        // Final Step
        Debug.Log("Publishing joints to ROS service...");

        m_Ros.SendServiceMessage<MoverServiceResponse>(m_RosServiceName, request, TrajectoryResponse);
    }

    void TrajectoryResponse(MoverServiceResponse response)
    {
        if (response.trajectories.Length > 0)
        {
            Debug.Log("Trajectory returned.");
            Debug.Log($"Response: {JsonUtility.ToJson(response)}");
            StartCoroutine(ExecuteTrajectories(response));
        }
        else
        {
            Debug.LogError("No trajectory returned from MoverService.");
        }
    }

    /// <summary>
    ///     Execute the returned trajectories from the MoverService.
    ///     The expectation is that the MoverService will return four trajectory plans,
    ///     PreGrasp, Grasp, PickUp, and Place,
    ///     where each plan is an array of robot poses. A robot pose is the joint angle values
    ///     of the six robot joints.
    ///     Executing a single trajectory will iterate through every robot pose in the array while updating the
    ///     joint values on the robot.
    /// </summary>
    /// <param name="response"> MoverServiceResponse received from ur10e_moveit mover service running in ROS</param>
    /// <returns></returns>
    IEnumerator ExecuteTrajectories(MoverServiceResponse response)
    {
        if (response.trajectories != null)
        {
            // For every trajectory plan returned
            for (var poseIndex = 0; poseIndex < response.trajectories.Length; poseIndex++)
            {
                Debug.Log($"Executing trajectory plan {poseIndex}");

                // For every robot pose in trajectory plan
                foreach (var t in response.trajectories[poseIndex].joint_trajectory.points)
                {
                    var jointPositions = t.positions;
                    // Log the original joint positions in radians
                    Debug.Log($"Joint positions (radians): {string.Join(", ", jointPositions)}");

                    // Convert joint positions from radians to degrees
                    var result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

                    // Log the converted joint positions in degrees
                    Debug.Log($"Joint positions (degrees): {string.Join(", ", result)}");

                    // Set the joint values for every joint
                    for (var joint = 0; joint < m_JointArticulationBodies.Length; joint++)
                    {
                        var joint1XDrive = m_JointArticulationBodies[joint].xDrive;
                        joint1XDrive.target = result[joint];
                        m_JointArticulationBodies[joint].xDrive = joint1XDrive;
                    }

                    // Wait for robot to achieve pose for all joint assignments
                    yield return new WaitForSeconds(k_JointAssignmentWait);
                }

                // Close the gripper if completed executing the trajectory for the Grasp pose
                if (poseIndex == (int)Poses.Grasp)
                {
                    CloseGripper();
                }

                // Wait for the robot to achieve the final pose from joint assignment
                yield return new WaitForSeconds(k_PoseAssignmentWait);
            }

            // All trajectories have been executed, open the gripper to place the target cube
            OpenGripper();
        }
    }

    void CloseGripper()
    {
        // Set the target position to close the gripper
        var leftOuterDrive = m_LeftOuterGripper.xDrive;
        var rightOuterDrive = m_RightOuterGripper.xDrive;

        leftOuterDrive.target = Mathf.Clamp(-30.0f, leftOuterDrive.lowerLimit, leftOuterDrive.upperLimit);
        rightOuterDrive.target = Mathf.Clamp(-30.0f, rightOuterDrive.lowerLimit, rightOuterDrive.upperLimit);

        m_LeftOuterGripper.xDrive = leftOuterDrive;
        m_RightOuterGripper.xDrive = rightOuterDrive;
        Debug.Log("Closing gripper...");
    }

    void OpenGripper()
    {
        var leftOuterDrive = m_LeftOuterGripper.xDrive;
        var rightOuterDrive = m_RightOuterGripper.xDrive;

        leftOuterDrive.target = Mathf.Clamp(30.0f, leftOuterDrive.lowerLimit, leftOuterDrive.upperLimit);
        rightOuterDrive.target = Mathf.Clamp(30.0f, rightOuterDrive.lowerLimit, rightOuterDrive.upperLimit);

        m_LeftOuterGripper.xDrive = leftOuterDrive;
        m_RightOuterGripper.xDrive = rightOuterDrive;
        Debug.Log("Opening gripper...");
    }



    /// <summary>
    ///     Capture the main camera's render texture and convert to bytes.
    /// </summary>
    /// <returns>imageBytes</returns>
    private byte[] CaptureScreenshot()
    {
        Camera.main.targetTexture = renderTexture;
        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = renderTexture;
        Camera.main.Render();
        Texture2D mainCameraTexture = new Texture2D(renderTexture.width, renderTexture.height);
        mainCameraTexture.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0);
        mainCameraTexture.Apply();
        RenderTexture.active = currentRT;
        // Get the raw byte info from the screenshot
        byte[] imageBytes = mainCameraTexture.GetRawTextureData();
        Camera.main.targetTexture = null;
        return imageBytes;
    }

    /// <summary>
    ///     Create a new PoseEstimationServiceRequest with the captured screenshot as bytes and instantiates 
    ///     a sensor_msgs/image.
    ///
    ///     Call the PoseEstimationService using the ROSConnection and calls PoseEstimationCallback on the 
    ///     PoseEstimationServiceResponse.
    /// </summary>
    /// <param name="imageData"></param>
    private void InvokePoseEstimationService(byte[] imageData)
    {
        uint imageHeight = (uint)renderTexture.height;
        uint imageWidth = (uint)renderTexture.width;

        RosMessageTypes.Sensor.ImageMsg rosImage = new RosMessageTypes.Sensor.ImageMsg(new RosMessageTypes.Std.HeaderMsg(), imageWidth, imageHeight, "RGBA", isBigEndian, step, imageData);
        PoseEstimationServiceRequest poseServiceRequest = new PoseEstimationServiceRequest(rosImage);

        m_Ros.SendServiceMessage<PoseEstimationServiceResponse>("pose_estimation_srv", poseServiceRequest, PoseEstimationCallback);

    }


    /// <summary>
    ///     Button callback for the Pose Estimation
    /// </summary>
    //public void PoseEstimation()
    //{
    //    Debug.Log("Capturing screenshot...");

    //    //InitializeButton.interactable = false;
    //    //RandomizeButton.interactable = false;
    //    ServiceButton.interactable = false;
    //    ActualPos.text = target.transform.position.ToString();
    //    ActualRot.text = target.transform.eulerAngles.ToString();
    //    EstimatedPos.text = "-";
    //    EstimatedRot.text = "-";

    //    // Capture the screenshot and pass it to the pose estimation service
    //    byte[] rawImageData = CaptureScreenshot();
    //    InvokePoseEstimationService(rawImageData);
    //}
    public void PoseEstimation()
    {
        Debug.Log("Starting PoseEstimation...");

        Debug.Log("Capturing screenshot...");
        byte[] rawImageData = CaptureScreenshot();

        InvokePoseEstimationService(rawImageData);
    }


    void PoseEstimationCallback(PoseEstimationServiceResponse response)
    {
        Debug.Log(response);

        if (response != null)
        {
            Debug.Log("estimate motion");
            // The position output by the model is the position of the cube relative to the camera so we need to extract its global position 
            var estimatedPosition = Camera.main.transform.TransformPoint(response.estimated_pose.position.From<RUF>());
            var estimatedRotation = Camera.main.transform.rotation * response.estimated_pose.orientation.From<RUF>();

            PublishJoints(estimatedPosition, estimatedRotation);

            EstimatedPos.text = estimatedPosition.ToString();
            EstimatedRot.text = estimatedRotation.eulerAngles.ToString();
        }
        else
        {
            InitializeButton.interactable = true;
            RandomizeButton.interactable = true;
        }
    }

    public void PublishJoints(Vector3 targetPos, Quaternion targetRot)
    {
        MoverServiceRequest request = new MoverServiceRequest();
        request.joints_input = CurrentJointConfig();

        // Pick Pose
        request.pick_pose = new RosMessageTypes.Geometry.PoseMsg
        {
            position = (targetPos + pickPoseOffset).To<FLU>(),
            orientation = Quaternion.Euler(90, targetRot.eulerAngles.y, 0).To<FLU>()
        };

        // Place Pose
        request.place_pose = new RosMessageTypes.Geometry.PoseMsg
        {
            position = (goal.position + placePoseOffset).To<FLU>(),
            orientation = pickOrientation.To<FLU>()
        };

        m_Ros.SendServiceMessage<MoverServiceResponse>(m_RosServiceName, request, TrajectoryResponse);
    }

    enum Poses
    {
        PreGrasp,
        Grasp,
        PickUp,
        Place
    }
}