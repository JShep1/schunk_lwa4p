#include <stdlib.h>
#include <Python.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "schunk_gripper_communication/schunk_gripper.h"
#include "moveit/move_group_interface/move_group.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/joint_model.h>
//#include <moveit/robot_state/joint_state_group.h>
#include <moveit/robot_state/robot_state.h>

//#include <planning_scene_interface.h>

bool set_motor(double motorvalue)
{

    std::stringstream ss;
    ss << motorvalue;
    char str1[50];
    const char* str = ss.str().c_str();
    strcpy(str1, "motorvalue = ");
    strcat(str1, str);
    strcat(str1, "\nprint motorvalue \n");
    ROS_INFO("Running Python Script");
    Py_Initialize();
    PyRun_SimpleString(str1);
    Py_Finalize();
    ROS_INFO("Python Script Complete");

    ROS_INFO("Setting motor value to %f", motorvalue);
    ROS_INFO("Completed motor movement - returning.");
    return true;
}

bool plan_motion(){
    ROS_INFO("Motion planning in progress.");
    //declare the movegroup to be planned for
    moveit::planning_interface::MoveGroup group("Arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ros::NodeHandle node_handle;

    //start thread
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    //display publisher for visualizing motion plan
    moveit_msgs::DisplayTrajectory display_trajectory;
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path",1,true);
    
    //Output info to make sure everything's loaded and good
    ROS_INFO("Reference frame: %s.", group.getPlanningFrame().c_str());
    ROS_INFO("EE link: %s.", group.getEndEffectorLink().c_str());
/* 
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.58;
    target_pose1.position.y = -0.7;
    target_pose1.position.z = 1.0;
    group.setPoseTarget(target_pose1);
*/
    
    //get current state of joints and modify one for easy motion plan testing
    std::vector<double> group_variable_values;
    group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
    group_variable_values[0] = -1.0;
    group.setJointValueTarget(group_variable_values);
  
    //load the robot model 
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

    //print out the frame of the robot model to test
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    //get the joint model group which represents the robot model for 
    //the Arm group
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("Arm");
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

    //print out the joint values for testing
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for(std::size_t i = 0; i < joint_names.size(); i++){
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

    //enforce the joint limits for the currentstate
    kinematic_state->enforceBounds();

    //set the kinematic state to random to test forward kinematics
    //fk: random pose of arm -> eef pose
    kinematic_state->setToRandomPositions(joint_model_group);
    const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("arm_6_link");
    ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
    ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());


    //find the inverse kinematics
    //ik: random pose of eef -> pose of arm
    bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 10, 0.1);
    if(found_ik){
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        ROS_INFO("Found IK solution:");
        for(std::size_t i = 0; i < joint_names.size(); i++){
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        }
    }else{
        ROS_INFO("Did not find IK solution");
    
    }

    //instantiate a motion plan and plan for it with the Arm group
    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);
    sleep(5.0);
    if(1){
        //visualize the created motion plan
        ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
        display_trajectory.trajectory_start = my_plan.start_state_;
        display_trajectory.trajectory.push_back(my_plan.trajectory_);
        display_publisher.publish(display_trajectory);
        sleep(5.0);
    }

    return true;
}

bool choose_function(schunk_gripper_communication::schunk_gripper::Request &req,
                     schunk_gripper_communication::schunk_gripper::Response &res)
{
    res.motorvalue = req.motorvalue;
    std::string function_1 = "set_motor";
    std::string function_2 = "plan_motion";
    if(!function_1.compare((std::string)req.function_name)){
        ROS_INFO("Found function set_motor");
        if(set_motor(res.motorvalue)){
            ROS_INFO("Successfully set motor.");
            return true;
        }else{
            ROS_ERROR("Could not set motor. Returning.");
            return false;
        }
    }else if(!function_2.compare((std::string)req.function_name)){
        ROS_INFO("Found function plan_motion");
        if(plan_motion()){
            ROS_INFO("Successfully created motion plan.");
            return true;
        }else{
            ROS_ERROR("Could not create motion plan. Returning.");
            return false;
        }
        
    }
    else{
        //std::stringstream ss;
        //ss << (std::string)req.function_name;
        //char str1[50];
        //str
        //const char* str = ss.str().c_str();

        ROS_ERROR("No such function: %s", req.function_name.c_str());
        return false;
    }
    //std::string str = convertToStr<double>(&(res.motorvalue));
    //std::cout << str << std::endl;
/*
    std::stringstream ss;
    ss << (double)req.motorvalue;
    char str1[50];
    const char* str = ss.str().c_str();
    strcpy(str1, "motorvalue = ");
    strcat(str1, str);
    strcat(str1, "\nprint motorvalue \n");
    ROS_INFO("Running Python Script");
    //std::cout << str1 << std::endl;
    //ROS_INFO("%s",((double)res.motorvalue).str().c_str());
    Py_Initialize();
    PyRun_SimpleString(str1);
    //PyRun_SimpleString(ss.str());
    Py_Finalize();
    ROS_INFO("Python Script Complete");

    ROS_INFO("Setting motor value to %f", (double)res.motorvalue);
    ROS_INFO("Completed motor movement - returning.");
  */
    return true;
}




/*

bool set_motor(schunk_gripper_communication::schunk_gripper::Request &req,
               schunk_gripper_communication::schunk_gripper::Response &res)
{
    res.motorvalue = req.motorvalue;
    res.pythonfile = req.pythonfile;
    res.pythonfunction = req.pythonfunction;
    ROS_INFO("Setting motor value to %f", (double)res.motorvalue);
    ROS_INFO("Completed motor movement - returning.");
    return true;
}

bool set_motor(schunk_gripper_communication::schunk_gripper::Request &req,
               schunk_gripper_communication::schunk_gripper::Response &res)
{
   
    //python here
    PyObject *pName, *pModule, *pDict, *pFunc;
    PyObject *pArgs, *pValue;
    int i;

    
    res.pythonfile = req.pythonfile;
    res.pythonfunction = req.pythonfunction;
    res.motorvalue = req.motorvalue;
   
    std::string pythonfile = req.pythonfile;
    std::string pythonfunction = req.pythonfunction;

    ROS_INFO("Setting motor value to %f", (double)res.motorvalue);

    setenv("PYTHONPATH",".",1);

    Py_Initialize();
    pName = PyString_FromString(((std::string)res.pythonfile).c_str());
    pModule = PyImport_Import(pName);
    Py_DECREF(pName);

    if (pModule != NULL) {
        pFunc = PyObject_GetAttrString(pModule, ((std::string)res.pythonfunction).c_str());

        if (pFunc && PyCallable_Check(pFunc)) {
            pArgs = PyTuple_New(1); //num of args intended to pass to function
            pValue = PyFloat_FromDouble((double)res.motorvalue);
            PyTuple_SetItem(pArgs, 0, pValue);
            pValue = PyObject_CallObject(pFunc, pArgs);
            Py_DECREF(pArgs);
            if (pValue != NULL) {
                printf("Result of call: %ld\n", PyInt_AsLong(pValue));
                Py_DECREF(pValue);
            }
            else {
                Py_DECREF(pFunc);
                Py_DECREF(pModule);
                PyErr_Print();
                fprintf(stderr,"Call failed\n");
                return false;
            }
        }
        else {
            if (PyErr_Occurred())
                PyErr_Print();
            fprintf(stderr, "Cannot find function\n");
        }
        Py_XDECREF(pFunc);
        Py_DECREF(pModule);
    } else {
        PyErr_Print();
        fprintf(stderr, "Failed to load python file\n");
        return false;
    }
    Py_Finalize();
    

    ROS_INFO("Completed motor movement - returning.");
    
    return true;
}
*/


int main(int argc, char **argv)
{
    ros::init(argc, argv, "schunk_gripper_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("choose_function", choose_function);
    ROS_INFO("Ready to run a function.");
    ros::spin();
    return 0;
}
