/* using UnityEngine;
using Unity.Robotics.UrdfImporter.Control;

public class CustomController : Controller
{
    public SetHomePose setHomePose;
    private bool initialized = false;
    private ArticulationBody[] articulationChain;
    private Color[] prevColor;
    private int previousIndex;

    protected void Start()
    {
        articulationChain = this.GetComponentsInChildren<ArticulationBody>();
        if (setHomePose != null)
        {
            this.enabled = false;
        }

        previousIndex = selectedIndex = 1;
        this.gameObject.AddComponent<FKRobot>();
        
        int defDyanmicVal = 10;
        foreach (ArticulationBody joint in articulationChain)
        {
            joint.gameObject.AddComponent<JointControl>();
            joint.jointFriction = defDyanmicVal;
            joint.angularDamping = defDyanmicVal;
            ArticulationDrive currentDrive = joint.xDrive;
            currentDrive.forceLimit = forceLimit;
            joint.xDrive = currentDrive;
        }
        DisplaySelectedJoint(selectedIndex);
        StoreJointColors(selectedIndex);
    }

    protected void Update()
    {
        if (setHomePose == null || setHomePose.homePoseSet)
        {
            bool SelectionInput1 = Input.GetKeyDown("right");
            bool SelectionInput2 = Input.GetKeyDown("left");

            SetSelectedJointIndex(selectedIndex);
            UpdateDirection(selectedIndex);

            if (SelectionInput2)
            {
                SetSelectedJointIndex(selectedIndex - 1);
                Highlight(selectedIndex);
            }
            else if (SelectionInput1)
            {
                SetSelectedJointIndex(selectedIndex + 1);
                Highlight(selectedIndex);
            }

            UpdateDirection(selectedIndex);
        }
    }

    private void SetSelectedJointIndex(int index)
    {
        if (articulationChain.Length > 0)
        {
            selectedIndex = (index + articulationChain.Length) % articulationChain.Length;
        }
    }

    private void UpdateDirection(int jointIndex)
    {
        if (jointIndex < 0 || jointIndex >= articulationChain.Length)
        {
            return;
        }

        float moveDirection = Input.GetAxis("Vertical");
        JointControl current = articulationChain[jointIndex].GetComponent<JointControl>();
        if (previousIndex != jointIndex)
        {
            JointControl previous = articulationChain[previousIndex].GetComponent<JointControl>();
            previous.direction = RotationDirection.None;
            previousIndex = jointIndex;
        }

        if (current.controltype != control)
        {
            UpdateControlType(current);
        }

        if (moveDirection > 0)
        {
            current.direction = RotationDirection.Positive;
        }
        else if (moveDirection < 0)
        {
            current.direction = RotationDirection.Negative;
        }
        else
        {
            current.direction = RotationDirection.None;
        }
    }

    private void UpdateControlType(JointControl joint)
    {
        joint.controltype = control;
        ArticulationBody articulation = joint.GetComponent<ArticulationBody>();
        ArticulationDrive currentDrive = articulation.xDrive;
        currentDrive.stiffness = stiffness;
        currentDrive.damping = damping;
        articulation.xDrive = currentDrive;
    }

    private void Highlight(int selectedIndex)
    {
        if (selectedIndex == previousIndex || selectedIndex < 0 || selectedIndex >= articulationChain.Length)
        {
            return;
        }

        ResetJointColors(previousIndex);
        StoreJointColors(selectedIndex);
        DisplaySelectedJoint(selectedIndex);
        
        Renderer[] rendererList = articulationChain[selectedIndex].transform.GetChild(0).GetComponentsInChildren<Renderer>();
        foreach (var mesh in rendererList)
        {
            mesh.material.color = highLightColor;
        }
    }

    private void DisplaySelectedJoint(int selectedIndex)
    {
        if (selectedIndex < 0 || selectedIndex >= articulationChain.Length)
        {
            return;
        }
        selectedJoint = articulationChain[selectedIndex].name + " (" + selectedIndex + ")";
    }

    private void StoreJointColors(int index)
    {
        Renderer[] materialLists = articulationChain[index].transform.GetChild(0).GetComponentsInChildren<Renderer>();
        prevColor = new Color[materialLists.Length];
        for (int counter = 0; counter < materialLists.Length; counter++)
        {
            prevColor[counter] = materialLists[counter].material.color;
        }
    }

    private void ResetJointColors(int index)
    {
        Renderer[] previousRendererList = articulationChain[index].transform.GetChild(0).GetComponentsInChildren<Renderer>();
        for (int counter = 0; counter < previousRendererList.Length; counter++)
        {
            previousRendererList[counter].material.color = prevColor[counter];
        }
    }
} */
