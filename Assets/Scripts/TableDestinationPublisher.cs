using RosMessageTypes.Geometry;
using RosMessageTypes.Shape;
using RosMessageTypes.Std;
using RosMessageTypes.Moveit;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace Assets.Scripts
{
    public class TableDestinationPublisher : MonoBehaviour
    {
        [SerializeField]
        string m_TopicName = "/collision_object";

        // [SerializeField]
        // string m_AttachTopicName = "/attached_collision_object";

        [SerializeField]
        GameObject m_Table;

        // ROS Connector
        ROSConnection m_Ros;

        [SerializeField]
        GameObject RobotBase;

        [SerializeField]
        GameObject m_Cube;

        void Start()
        {
            m_Ros = ROSConnection.GetOrCreateInstance();
            m_Ros.RegisterPublisher<CollisionObjectMsg>(m_TopicName);
            //yield return new WaitForSeconds(2.0f);
            //Publish();
            // m_Ros.RegisterPublisher<AttachedCollisionObjectMsg>(m_AttachTopicName);
        }

        public void Publish()
        {
            // Create the table pose
            var tablePose = new PoseMsg
            {
                position = m_Table.transform.position.To<FLU>(),
                orientation = m_Table.transform.rotation.To<FLU>()
            };
            tablePose.position.z -= 0.32;

            // Wrap in PoseStampedMsg for clarity
            var tablePoseStamped = new PoseStampedMsg
            {
                header = new HeaderMsg
                {
                    frame_id = "robot_base_link"
                },
                pose = tablePose
            };

            Debug.Log($"Table Coordinates: {tablePose.position.x}, {tablePose.position.y}, {tablePose.position.z}");

            // Create collision object
            var collisionObject = new CollisionObjectMsg
            {
                header = tablePoseStamped.header, // header goes here
                id = "table",
                operation = CollisionObjectMsg.ADD,
                primitive_poses = new PoseMsg[] { tablePoseStamped.pose }, // only the pose part
                primitives = new SolidPrimitiveMsg[]
                {
            new SolidPrimitiveMsg
            {
                type = SolidPrimitiveMsg.BOX,
                dimensions = new double[] { 5.0, 5.0, 0.64 }
            }
                }
            };

            // Publish
            m_Ros.Publish(m_TopicName, collisionObject);
            Debug.Log("Collision object published!");

            Debug.Log("Table collision object published!");

            // Get cube world size
            Vector3 cubeWorldSize = Vector3.Scale(
                m_Cube.GetComponent<MeshFilter>().sharedMesh.bounds.size,
                m_Cube.transform.lossyScale
            );

            // Small safety margin above table
            float safetyMargin = 0.01f; // 1 cm

            // Convert Unity position to local relative to robot base
            Vector3 cubeLocal = RobotBase.transform.InverseTransformPoint(m_Cube.transform.position);

            // Map Unity axes (x:right, y:up, z:forward) to ROS FLU (x:forward, y:left, z:up)
            Vector3 cubeFLU = new Vector3(
                cubeLocal.z,                // Unity forward → ROS X
                -cubeLocal.x,               // Unity right → ROS Y (flip)
                (float)(tablePose.position.z + 0.32 + cubeWorldSize.y / 2.0f + safetyMargin) // Table top + cube half-height + margin
            );

            // Create PoseMsg for cube
            PoseMsg cubePose = new PoseMsg
            {
                position = new PointMsg(cubeFLU.x, cubeFLU.y, cubeFLU.z),
                orientation = Quaternion.identity.To<FLU>() // no rotation
            };

            // Create collision primitive with proper FLU dimensions
            // Unity: x=width, y=height, z=depth
            // ROS FLU: x=forward(depth), y=left(width), z=up(height)
            SolidPrimitiveMsg cubePrimitive = new SolidPrimitiveMsg
            {
                type = SolidPrimitiveMsg.BOX,
                dimensions = new double[]
                {
                    (double)cubeWorldSize.z, // forward / depth
                    (double)cubeWorldSize.x, // left / width
                    (double)cubeWorldSize.y  // up / height
                }
            };

            //// Create CollisionObjectMsg
            //var cube = new CollisionObjectMsg
            //{
            //    header = new HeaderMsg { frame_id = "robot_base_link" },
            //    id = "cube_obstacle",
            //    operation = CollisionObjectMsg.ADD,
            //    primitive_poses = new PoseMsg[] { cubePose },
            //    primitives = new SolidPrimitiveMsg[] { cubePrimitive }
            //};

            //// Publish cube
            //m_Ros.Publish(m_TopicName, cube);
            //Debug.Log("Cube obstacle published (aligned with table)!");
        }
    }
}
