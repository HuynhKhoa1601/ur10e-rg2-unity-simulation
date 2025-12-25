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

        [SerializeField]
        GameObject MakerBot;

        // ROS Connector
        ROSConnection m_Ros;

        [SerializeField]
        GameObject RobotBase;

        [SerializeField]
        GameObject ObstacleCube;

        [SerializeField]
        Vector3 makerBotRotationCorrection = new Vector3(90, 0, 0);

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

            MeshFilter tableMesh = m_Table.GetComponent<MeshFilter>();

            Vector3 tableWorldSize = Vector3.Scale(
                tableMesh.sharedMesh.bounds.size,
                m_Table.transform.lossyScale
            );

            double sizeX = tableWorldSize.z; // forward
            double sizeY = tableWorldSize.x; // left
            double sizeZ = tableWorldSize.y; // up

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
                        dimensions = new double[]
                        {
                             sizeX,
                             sizeY,
                             sizeZ
                        }
                    }
                }
            };

            // Publish
            m_Ros.Publish(m_TopicName, collisionObject);
            Debug.Log("Collision object published!");

            Debug.Log("Table collision object published!");

            // Get cube world size
            Vector3 cubeWorldSize = Vector3.Scale(
                ObstacleCube.GetComponent<MeshFilter>().sharedMesh.bounds.size,
                ObstacleCube.transform.lossyScale
            );

            // Small safety margin above table
            float safetyMargin = 0.01f; // 1 cm

            // Convert Unity position to local relative to robot base
            Vector3 cubeLocal = RobotBase.transform.InverseTransformPoint(ObstacleCube.transform.position);

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
            var cube = new CollisionObjectMsg
            {
                header = new HeaderMsg { frame_id = "robot_base_link" },
                id = "cube_obstacle",
                operation = CollisionObjectMsg.ADD,
                primitive_poses = new PoseMsg[] { cubePose },
                primitives = new SolidPrimitiveMsg[] { cubePrimitive }
            };

            ////// Publish cube
            m_Ros.Publish(m_TopicName, cube);
            Debug.Log("Cube obstacle published (aligned with table)!");



            PublishMakerBot();

        }


        void PublishMakerBot()
        {
            MeshFilter meshFilter = MakerBot.GetComponent<MeshFilter>();
            if (meshFilter == null)
            {
                Debug.LogError("MakerBot has no MeshFilter!");
                return;
            }

            Mesh mesh = meshFilter.sharedMesh;

            // ---------- Create shape_msgs/Mesh ----------
            List<PointMsg> vertices = new List<PointMsg>();
            Vector3 scale = MakerBot.transform.lossyScale; // Get Unity world scale

            foreach (Vector3 v in mesh.vertices)
            {
                // Apply scale
                Vector3 scaledVertex = Vector3.Scale(v, scale);
                scaledVertex.x = -scaledVertex.x;
                vertices.Add(new PointMsg(scaledVertex.x, scaledVertex.y, scaledVertex.z));
            }

            List<MeshTriangleMsg> triangles = new List<MeshTriangleMsg>();
            int[] tris = mesh.triangles;
            for (int i = 0; i < tris.Length; i += 3)
            {
                triangles.Add(new MeshTriangleMsg
                {
                    vertex_indices = new uint[]
                    {
                (uint)tris[i],
                    (uint)tris[i + 2],
                (uint)tris[i + 1]
            
                    }
                });
            }

            MeshMsg meshMsg = new MeshMsg
            {
                vertices = vertices.ToArray(),
                triangles = triangles.ToArray()
            };

            // ---------- POSITION (floor-safe) ----------
            Vector3 makerLocal = RobotBase.transform.InverseTransformPoint(MakerBot.transform.position);

            Vector3 makerFLU = new Vector3(
                makerLocal.z,   // Unity forward → ROS X
                -makerLocal.x,  // Unity right   → ROS Y
                makerLocal.y    // Unity up      → ROS Z  (DO NOT TOUCH)
            );

            Quaternion makerRotLocal =
    Quaternion.Inverse(RobotBase.transform.rotation) *
    MakerBot.transform.rotation;

            // Extract Unity yaw only (safe)
            float unityYawDeg = makerRotLocal.eulerAngles.y;

            // Convert Unity yaw → ROS yaw (Z axis)
            Quaternion rosYaw =
    Quaternion.AngleAxis(-(unityYawDeg + 180), -Vector3.forward);


            PoseMsg makerPose = new PoseMsg
            {
                position = new PointMsg(makerFLU.x, makerFLU.y, makerFLU.z),
                orientation = new QuaternionMsg(
                    rosYaw.x,
                    rosYaw.y,
                    rosYaw.z,
                    rosYaw.w
                )
            };

            // ---------- Collision object ----------
            CollisionObjectMsg makerBotObject = new CollisionObjectMsg
            {
                header = new HeaderMsg { frame_id = "robot_base_link" },
                id = "makerbot",
                operation = CollisionObjectMsg.ADD,
                meshes = new MeshMsg[] { meshMsg },
                mesh_poses = new PoseMsg[] { makerPose }
            };

            Debug.Log(
    $"[UNITY] MakerBot local to robot (m): " +
    $"x={makerLocal.z:F3}, y={-makerLocal.x:F3}, z={makerLocal.y:F3}"
);


            m_Ros.Publish(m_TopicName, makerBotObject);
            Debug.Log("MakerBot collision mesh published (floor-safe, yaw-aligned)");
        }

    }
}
