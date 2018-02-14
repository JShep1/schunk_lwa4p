#include <stdlib.h>
#include <Python.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "schunk_gripper_communication/schunk_gripper.h"
#include "moveit/move_group_interface/move_group.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/planning_scene/planning_scene.h"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/joint_model.h>
//#include <moveit/robot_state/joint_state_group.h>
#include <moveit/robot_state/robot_state.h>
#include <eigen_conversions/eigen_msg.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <moveit/collision_detection/collision_common.h>
#include <Eigen/Geometry>
#include <schunk/set_schunk.h>
#include <sensor_msgs/JointState.h>

//#include <planning_scene_interface.h>
Schunk schunk;

bool init_schunk(){
    schunk.initialize();
    return true;
}

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

Eigen::Affine3d create_rotation_matrix(double ax, double ay, double az){
    Eigen::Affine3d rx = 
        Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
    Eigen::Affine3d ry = 
        Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(0, 1, 0)));
    Eigen::Affine3d rz = 
        Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(0, 0, 1)));

    return rz * ry * rx;
}


bool place_point(){

    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
   
    ros::Rate r(30); 
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    //ros::Rate r(30);
    float f = 0.0;

    while(ros::ok()){
        visualization_msgs::Marker points;
        points.header.frame_id = "/my_frame";
        points.header.stamp = ros::Time::now();
        points.ns = "points_and_lines";
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;

        points.id = 0;

        points.type = visualization_msgs::Marker::POINTS;

        points.scale.x = 0.2;
        points.scale.y = 0.2;

        points.color.g = 1.0f;
        points.color.a = 1.0;

        for(int32_t i = 0; i < 10; ++i){
            float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
            float z = 5 * cos(f + i / 100.0f * 2 * M_PI);

            geometry_msgs::Point p;
            p.x = (int32_t)i - 50;
            p.y = y;
            p.z = z;

            points.points.push_back(p);

            p.z += 1.0;

        }
        
        marker_pub.publish(points);

        r.sleep();
        f += 0.04;

    }
    return true;
}

bool create_matrix(double x, double y, double z, double rot_x, 
                   double rot_y, double rot_z, Eigen::Affine3d &m){

    Eigen::Affine3d r = create_rotation_matrix(rot_x, rot_y, rot_z);
    Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(x,y,z)));

    m = (t * r);
    return true;
}


bool plan_motion(){
   

    ROS_INFO("Motion planning in progress.");
    //declare the movegroup to be planned for
    moveit::planning_interface::MoveGroup group("Arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ros::NodeHandle node_handle;

    const char* plans[] = {
                    "SBLkConfigDefault",
                    "ESTkConfigDefault",
                    "LBKPIECEkConfigDefault",
                    "BKPIECEkConfigDefault",
                    "KPIECEkConfigDefault",
                    "RRTkConfigDefault",
                    "RRTConnectkConfigDefault",
                    "RRTstarkConfigDefault",
                    "TRRTkConfigDefault",
                    "PRMkConfigDefault",
                    "PRMstarkConfigDefault"
    };

    std::vector<std::string> planners(plans, plans + 11);

    //start thread
    ros::AsyncSpinner spinner(1);
    spinner.start();
    //display publisher for visualizing motion plan
    moveit_msgs::DisplayTrajectory display_trajectory;
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path",1,true);
    ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene",1);

    while(planning_scene_diff_publisher.getNumSubscribers() < 1){
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
    }
    
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = "base_link";
    attached_object.object.header.frame_id = "base_link";

    attached_object.object.id = "box";
    group.setStartState(*group.getCurrentState());
    double box_x = 0.5;
    double box_y = 0.5;
    double box_z = 0.5;

    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = box_x;
    pose.position.y = box_y;
    pose.position.z = box_z;


    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.1;
    primitive.dimensions[1] = 0.1;
    primitive.dimensions[2] = 0.1;

    attached_object.object.primitives.push_back(primitive);
    attached_object.object.primitive_poses.push_back(pose);
    attached_object.object.operation = attached_object.object.ADD;

    ROS_INFO("Adding the object into the world at the location of the end effector");

    moveit_msgs::PlanningScene moveit_planning_scene;
    moveit_planning_scene.world.collision_objects.push_back(attached_object.object);
    moveit_planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(moveit_planning_scene);
    //sleep_time.sleep();
/*
    ros::ServiceClient planning_scene_diff_client = node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
    planning_scene_diff_client.waitForExistence();

    moveit_msgs::ApplyPlanningScene srv;
    srv.request.scene = planning_scene;
    planning_scene_diff_client.call(srv);
*/
    //Output info to make sure everything's loaded and good
    ROS_INFO("Reference frame: %s.", group.getPlanningFrame().c_str());
    ROS_INFO("EE link: %s.", group.getEndEffectorLink().c_str());
    //group.setPlannerId("RRTConnectkConfigDefault");
/* 
*/
    
    //get current state of joints and modify one for easy motion plan testing

/*
*/ 
    /*
    std::vector<double> group_variable_values;
    group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
    group_variable_values[0] = -1.0;
    group.setJointValueTarget(group_variable_values);
    */
    //load the robot model 
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);

    //print out the frame of the robot model to test
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    //get the joint model group which represents the robot model for 
    //the Arm group
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("Arm");
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

    //print out the joint values for testing
/*
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for(std::size_t i = 0; i < joint_names.size(); i++){
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
*/
    //enforce the joint limits for the currentstate
    kinematic_state->enforceBounds();

    //set the kinematic state to random to test forward kinematics
    //fk: random pose of arm -> eef pose
  /*
    kinematic_state->setToRandomPositions(joint_model_group);
    const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("arm_6_link");
    ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
    ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());
*/
/*
    //find the inverse kinematics
    //ik: random pose of eef -> pose of arm
  */
    //const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("gripper_link");
    //

    /*
    */

    Eigen::Affine3d m;
    create_matrix(box_x,box_y,box_z,1,1,1,m);

    ROS_INFO_STREAM("Translation: " << m.translation());
    ROS_INFO_STREAM("Rotation: " << m.rotation());

    const Eigen::Affine3d &end_effector_state = m;

    bool found_ik = true;//kinematic_state->setFromIK(joint_model_group, end_effector_state, 10, 0.1);
    /*
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = box_x;
    target_pose1.position.y = box_y;
    target_pose1.position.z = box_z;
    */
      //TODO figure out constraints
    //planning_interface::MotionPlanRequest req;

    //req.group_name = "Arm";

    moveit::planning_interface::MoveGroup::Plan my_plan;

    std::vector<double> group_variable_values;
    group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
    //group_variable_values[0] = 0;
    //group_variable_values[1] = 1.5;
    unsigned int planner_count = 0;
    //while (true){
    /*
        if(planner_count >= planners.size()){
        ROS_INFO("All planners attempted and no solution found.");
        break;
    }
    */
    //group.setPlannerId(planners[planner_count]);
    //planner_count++;
    found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 10, 0.1);

    if(found_ik){
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        ROS_INFO("Found possible IK solution:");
        for(std::size_t i = 0; i < joint_names.size(); i++){
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
//            if(i < 6){
                group_variable_values[i] = joint_values[i];
//            }
        }
      /*  if(group_variable_values[1] > 1.5 || group_variable_values[1] < -1.5){
           //ROS_INFO("IK solution found to be invalid. Trying again.");
           continue;
        }else{
           //ROS_INFO("IK solution found to be valid.");
            break;
        }
        */
    }else{
         ROS_INFO("Did not find IK solution.");
        //continue;
    }
    //}
    group.setJointValueTarget(group_variable_values);

    //instantiate a motion plan and plan for it with the Arm group
    /*
       target_pose1.orientation.w = 1.0;
       target_pose1.position.x = 0.58;
       target_pose1.position.y = -0.7;
       target_pose1.position.z = 1.0;
       */


    /*
       tf::poseEigenToMsg(end_effector_state, target_pose1);
       */
    /*
    */

    /*
       group.setPoseTarget(target_pose1);
       */
    group.setPlanningTime(10.0);
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

    if(1){
        group.execute(my_plan);
    }


    group.setStartState(*group.getCurrentState());
    if(1){
        //visualize the created motion plan
        display_trajectory.trajectory.clear();
        display_publisher.publish(display_trajectory);
    }

    sleep(5.0);

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    planning_scene.checkSelfCollision(collision_request, collision_result);

    ROS_INFO_STREAM("Current state is "
            <<(collision_result.collision ? "in" : "not in")
            << " self collision");
/*
*/
    moveit_msgs::CollisionObject remove_object;
    remove_object.id = "box";
    remove_object.header.frame_id = "base_link";
    remove_object.operation = remove_object.REMOVE;



    ROS_INFO("Attaching the object to the eef and removing it from the world.");
    moveit_planning_scene.world.collision_objects.clear();
    moveit_planning_scene.world.collision_objects.push_back(remove_object);
    moveit_planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
    planning_scene_diff_publisher.publish(moveit_planning_scene);


/* Item picked - begin to place
 *
 *
 */
  /*
    kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene_two(kinematic_model);

    //print out the frame of the robot model to test
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    //get the joint model group which represents the robot model for 
    //the Arm group
    robot_state::RobotStatePtr kinematic_state_two(new robot_state::RobotState(kinematic_model));
    moveit::planning_interface::MoveGroup group_two("Arm");
    kinematic_state_two->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group_two = kinematic_model->getJointModelGroup("Arm");
    const std::vector<std::string> &joint_names_two = joint_model_group_two->getJointModelNames();
    std::vector<double> joint_values_two;
    kinematic_state_two->copyJointGroupPositions(joint_model_group_two, joint_values_two);

    //print out the joint values for testing
//
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for(std::size_t i = 0; i < joint_names.size(); i++){
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
//
    //enforce the joint limits for the currentstate
    kinematic_state_two->enforceBounds();

    //group.setStartState(*group.getCurrentState());
    Eigen::Affine3d end_mat;
    create_matrix(box_x-0.1,box_y-0.1,box_z-0.1,1,1,1,end_mat);

    const Eigen::Affine3d &end_effector_end_state = m;

    found_ik = kinematic_state_two->setFromIK(joint_model_group_two, end_effector_end_state, 10, 0.1);

    if(found_ik){
        kinematic_state->copyJointGroupPositions(joint_model_group_two, joint_values_two);
        ROS_INFO("Found possible IK solution for 2nd scenario:");
        for(std::size_t i = 0; i < joint_names_two.size(); i++){
            ROS_INFO("Joint %s: %f", joint_names_two[i].c_str(), joint_values_two[i]);
            if(i < 6){
                group_variable_values[i] = joint_values_two[i];
            }
        }
    //    if(group_variable_values[1] > 1.5 || group_variable_values[1] < -1.5){
           //ROS_INFO("IK solution found to be invalid. Trying again.");
           continue;
        }else{
           //ROS_INFO("IK solution found to be valid.");
            break;
        }
  //      
    }else{
         ROS_INFO("Did not find IK solution.");
        //continue;
    }
    //}
    group_two.setJointValueTarget(group_variable_values);
    moveit::planning_interface::MoveGroup::Plan my_plan_two;
    
    success = group_two.plan(my_plan_two);
    sleep(5.0);
    display_trajectory.trajectory.clear();
    if(1){
        //visualize the created motion plan
        ROS_INFO("Visualizing plan 2 (item place goal) %s",success?"":"FAILED");
        display_trajectory.trajectory_start = my_plan_two.start_state_;
        display_trajectory.trajectory.push_back(my_plan_two.trajectory_);
        display_publisher.publish(display_trajectory);
        sleep(5.0);
    }
    if(1){
        group_two.execute(my_plan_two);
    }
*/

    return true;
}

bool execute_motion(){
    //group.move();
    return true;
}

bool choose_function(schunk_gripper_communication::schunk_gripper::Request &req,
        schunk_gripper_communication::schunk_gripper::Response &res)
{
    res.motorvalue = req.motorvalue;
    std::string function_1 = "set_motor";
    std::string function_2 = "plan_motion";
    std::string function_3 = "execute_motion";
    std::string function_4 = "init_schunk";
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
    else if(!function_3.compare((std::string)req.function_name)){
        ROS_INFO("Found function execute_motion");
        if(execute_motion()){
            ROS_INFO("Successfully executed motion.");
            return true;
        }else{
            ROS_ERROR("Could not execute motion. Returning.");
            return false;
        }

    }
    else if(!function_4.compare((std::string)req.function_name)){
        ROS_INFO("Found function init_schunk");
        if(init_schunk()){
            ROS_INFO("Successfully executed motion.");
            return true;
        }else{
            ROS_ERROR("Could not execute motion. Returning.");
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
