#include <stdlib.h>
//#include <Python.h>
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
#include <moveit/robot_state/robot_state.h>
#include <eigen_conversions/eigen_msg.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <moveit/collision_detection/collision_common.h>
#include <Eigen/Geometry>
#include <schunk/set_schunk.h>
#include <sensor_msgs/JointState.h>
#include <boost/filesystem.hpp>
#include <icl_hardware_canopen/CanOpenController.h>
#include <icl_hardware_canopen/SchunkPowerBallNode.h>

using namespace icl_hardware::canopen_schunk;

Schunk::Schunk(ros::NodeHandle* nodehandle):node_handle(*nodehandle)
{
    this->priv_nh = nodehandle;
    this->initialize();
}

Schunk::Schunk(){

}

Schunk::~Schunk(){

}

//update the eef with its current state
void Schunk::set_eef(){
    const Eigen::Affine3d &eef_state = kinematic_state_->getGlobalLinkTransform("arm_6_link");
    tf::poseEigenToMsg(eef_state, eef_pose_);
}

//get the eef pose
geometry_msgs::Pose Schunk::get_eef(){
    const Eigen::Affine3d &eef_state = kinematic_state_->getGlobalLinkTransform("arm_6_link");
    geometry_msgs::Pose eef_pose;
    tf::poseEigenToMsg(eef_state, eef_pose);
    return eef_pose;
}

//get the joint angles that the robotic arm is still at
std::vector<double> Schunk::get_joint_angles(){
    std::vector<double> joint_values;
    kinematic_state_->copyJointGroupPositions(joint_model_group_, joint_values);
    return joint_values;
}

//update the planning time
void Schunk::set_planning_time(double time){
    group_->setPlanningTime(time);
}


//init function, shouldn't need to be modified, additional
//initializations might be needed though
void Schunk::initialize(){
    ROS_INFO("instantiating Schunk arm");

    priv_nh->getParam("chain_names", chain_names);

    priv_nh->param<std::string>("can_device_name", can_device_name, "auto");

    priv_nh->param<float>  ("ppm_profile_velocity", ppm_config.profile_velocity,         0.2);

    priv_nh->param<float>  ("ppm_profile_acceleration",   ppm_config.profile_acceleration,     0.2);

    priv_nh->param<bool>   ("ppm_use_relative_targets",   ppm_config.use_relative_targets,     false);

    priv_nh->param<bool>   ("ppm_change_set_immediately", ppm_config.change_set_immediately,   true);

    priv_nh->param<bool>   ("ppm_use_blending",           ppm_config.use_blending,             true);

    group_.reset(new moveit::planning_interface::MoveGroup("Arm"));
    group_->setPlanningTime(5);


    got_robot_state = false;
    got_plan = false;
    have_joint_vals = false;
    schunk_is_init = false;
    is_motion_simulated = false;

    display_publisher_ = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path",1,true);
    planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene",1);

    robot_model_loader_ = robot_model_loader::RobotModelLoader("robot_description");
    robot_model_ = robot_model_loader_.getModel();
    planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));

    kinematic_state_.reset(new robot_state::RobotState(robot_model_));
    kinematic_state_->setToDefaultValues();
    kinematic_state_->enforceBounds();


    joint_model_group_ = robot_model_->getJointModelGroup("Arm");
    joint_names_ = joint_model_group_->getJointModelNames();

    reset_joint_values();

}

bool Schunk::shutdown_schunk(){

  schunk_is_init = false;

  try
  {
    for (size_t i = 0; i < chain_handles.size(); ++i)
    {
      chain_handles[i]->disableNodes();
    }
  }
  catch (const ProtocolException& e)
  {
    ROS_ERROR_STREAM ( "Error while disabling nodes: " << e.what());
  }
  return true;
}

bool Schunk::send_to_pos(std::vector<double> joint_angles){
 ds402::eModeOfOperation mode = ds402::MOO_PROFILE_POSITION_MODE;

for (size_t i = 0; i < chain_handles.size(); ++i)
  {
    try {
      ROS_INFO_STREAM ("Setting up Profile Position mode for chain " << chain_handles[i]->getName());
      chain_handles[i]->setupProfilePositionMode(ppm_config);
      chain_handles[i]->enableNodes(mode);
    }
    catch (const ProtocolException& e)
    {
      ROS_ERROR_STREAM ("Caught ProtocolException while enabling nodes from chain " <<
        chain_handles[i]->getName() << ". Nodes from this group won't be enabled.");
      continue;
    }
    ROS_INFO_STREAM ("Enabled nodes from chain " << chain_handles[i]->getName());
    std::vector<DS301Node::Ptr> nodes = chain_handles[i]->getNodes();
    std::vector<float> targets (nodes.size(), 0.0);
    targets[0] = (float)joint_angles[0];
    targets[1] = (float)joint_angles[1];
    targets[2] = (float)joint_angles[2];
    targets[3] = (float)joint_angles[3];
    targets[4] = (float)joint_angles[4];
    targets[5] = (float)joint_angles[5];
    chain_handles[i]->setTarget(targets);
  }
  my_controller->enablePPMotion();

  std::vector<bool> foo;

  while ( true )
  {
    size_t num_reached = 0;
    try {
      my_controller->syncAll();
    }
    catch (const std::exception& e)
    {
      ROS_ERROR_STREAM (e.what());
      return false;
    }
    usleep(10000);

    for (size_t i = 0; i < chain_handles.size(); ++i)
    {
      if (chain_handles[i]->isTargetReached(foo))
      {
        num_reached++;
      }
    }
    if (num_reached == chain_handles.size())
    {
      break;
    }
  }

  LOGGING_INFO (CanOpen, "All targets reached" << endl);
  return true;
}

//initialize actual schunk arm
bool Schunk::init_schunk()
{
// Create a canopen controller
    ROS_INFO_STREAM("can device name: " << can_device_name);
  ROS_INFO_STREAM("1");
  try
  {
    my_controller = boost::make_shared<CanOpenController>(can_device_name);
  }
  catch (const DeviceException& e)
  {
    ROS_ERROR_STREAM ("Initializing CAN device failed. Reason: " << e.what());
    ROS_INFO ("Shutting down now.");
    return false;
  }
  // Load SCHUNK powerball specific error codes
  char const* tmp = std::getenv("CANOPEN_RESOURCE_PATH");
  ROS_INFO_STREAM("2");
  if (tmp == NULL)
  {
    LOGGING_WARNING_C(
        CanOpen,
        CanOpenController,
        "The environment variable 'CANOPEN_RESOURCE_PATH' could not be read. No Schunk specific error codes will be used." << endl);
  }
  else
  {
    std::string emcy_emergency_errors_filename = boost::filesystem::path(tmp / boost::filesystem::path("EMCY_schunk.ini")).string();
    EMCY::addEmergencyErrorMap( emcy_emergency_errors_filename, "schunk_error_codes");
  }

  ROS_INFO_STREAM("3");

  // Get chain configuration from parameter server
  ROS_INFO_STREAM ("Can device identifier: " << can_device_name);
  ROS_INFO_STREAM ("Found " << chain_names.size() << " chains");

  // parse the robot configuration
  for (size_t i = 0; i < chain_names.size(); ++i)
  {
    std::string name = "chain_" + chain_names[i];
    my_controller->addGroup<DS402Group>(chain_names[i]);
    chain_handles.push_back(my_controller->getGroup<DS402Group>(chain_names[i]));
    std::vector<int> chain;
    try
    {
      priv_nh->getParam(name, chain);
    }
    catch (ros::InvalidNameException e)
    {
      ROS_ERROR_STREAM("Parameter Error!");
    }
    if (chain.size() == 0)
    {
      ROS_ERROR_STREAM("Did not find device list for chain " << chain_names[i] << ". Make sure, that an entry " << name << " exists.");
      continue;
    }
    ROS_INFO_STREAM ("Found chain with name " << name << " and " << chain.size() << " nodes");
    chain_configurations[name] = chain;
    for (size_t j = 0; j < chain.size(); ++j)
    {
      my_controller->addNode<SchunkPowerBallNode>(chain[j], chain_names[i]);

      std::string joint_name = "";
      std::string mapping_key = "~node_mapping_" + boost::lexical_cast<std::string>( chain[j]);
      ros::param::get(mapping_key, joint_name);

      joint_name_mapping[joint_name] =  static_cast<uint8_t>(chain[j]);
      joint_msg.name.push_back(joint_name);
    }
  }

  // initialize all nodes, by default this will start ProfilePosition mode, so we're good to enable nodes
  try {
    my_controller->initNodes();
  }
  catch (const ProtocolException& e)
  {
    ROS_ERROR_STREAM ("Caught ProtocolException while initializing devices: " << e.what());
    ROS_INFO ("Going to shut down now");
    exit (-1);
  }
  catch (const PDOException& e)
  {
    ROS_ERROR_STREAM ("Caught PDOException while initializing devices: " << e.what());
    ROS_INFO ("Going to shut down now");
    exit (-1);
  }
  schunk_is_init = true;
  return true;
}

//print the input transform
void Schunk::print_transform(tf::Transform trans, std::string info)
 {
   ROS_INFO_STREAM(info << " position: " << trans.getOrigin().getX() << " "
                   << trans.getOrigin().getY() << " "<< trans.getOrigin().getZ());
   ROS_INFO_STREAM(info <<" orientation: " << trans.getRotation().getX() << " "
                   << trans.getRotation().getY() << " "<< trans.getRotation().getZ() << " "
                   << trans.getRotation().getW());
}

//print the input pose
void Schunk::print_pose(geometry_msgs::Pose pose, std::string info)
{
   ROS_INFO_STREAM(info << " position: " << pose.position.x << " "
                   << pose.position.y << " "<< pose.position.z);
   ROS_INFO_STREAM(info << " orientation: " << pose.orientation.x << " "
                   << pose.orientation.y << " "<< pose.orientation.z << " "
                     << pose.orientation.w);
}

//get the inverse kinematics of the arm for the
//input eef pose, not needed to be explicitly called
//as plan_motion calls it
bool Schunk::getIK(geometry_msgs::Pose eef_pose){

    if(got_robot_state){
        kinematic_state_->setJointGroupPositions(joint_model_group_, joint_values_);
        got_robot_state = false;
    }else{
        ROS_WARN("[Schunk::getIK] Using the default robot joint state NOT current");
    }


    kinematic_state_->enforceBounds();

    Eigen::Affine3d pose_in;
    tf::poseMsgToEigen(eef_pose, pose_in);

    print_pose(eef_pose, "eef_pose");

    ROS_INFO_STREAM("eigen affine: " << pose_in.matrix());
    bool found_ik = kinematic_state_->setFromIK(joint_model_group_, pose_in, 10, 0.5);

    return found_ik;
}

//update values of joints
void Schunk::jointStatesCallback(std::vector<double> joint_values){
    //copy from joint_values to joint_values_
    joint_values_.clear();
    for(unsigned int i = 0; i < joint_values.size(); i++){
        joint_values_.push_back(joint_values[i]);
    }

    got_robot_state = true;
}

//create a trajectory from one point to another with time amount of time
//in between the two positions
trajectory_msgs::JointTrajectoryPointPtr Schunk::create_traj_point(std::vector<double> joint_values, double time){

    trajectory_msgs::JointTrajectoryPointPtr traj_point(new trajectory_msgs::JointTrajectoryPoint);

    traj_point->time_from_start = ros::Duration(time);


    for(unsigned int i = 0; i < joint_values.size(); i++){
        traj_point->positions.push_back(joint_values[i]);
        traj_point->velocities.push_back(0.0);
        traj_point->accelerations.push_back(0.0);
    }


    return traj_point;
}

//plan motion, only plan for input joint angles
bool Schunk::plan_motion(std::vector<double> joint_angles){

    group_->setJointValueTarget(joint_angles);
    bool success = group_->plan(plan_);

    sleep(5.0);

    if(got_plan){
        return success;
    }

    got_plan = success;

    have_joint_vals = success;
    return success;

}


//plan a motion for the input eef pose
bool Schunk::plan_motion(geometry_msgs::Pose eef_pose){

    if(!this->getIK(eef_pose)){
        ROS_INFO("Could not find IK solution!");
        got_plan = false;
        return false;
    }

    ROS_INFO("Found IK solution!");

    kinematic_state_->copyJointGroupPositions(joint_model_group_, joint_values_);
    for(std::size_t i = 0; i < joint_values_.size(); i++){
        ROS_INFO("Joint %s: %f", joint_names_[i].c_str(), joint_values_[i]);
    }


    group_->setJointValueTarget(joint_values_);
    bool success = group_->plan(plan_);

    sleep(5.0);

    if(got_plan){
        return success;
    }

    got_plan = success;
    have_joint_vals = success;
    return success;
}

bool Schunk::execute_motion(){
    if (!have_joint_vals){
        ROS_INFO("No IK Solution provided.  Create motion plan.");
        return false;
    }
    if (!is_motion_simulated){
      ROS_INFO("Motion plan has not yet been simulated! Please simulate motion.");
      return false;
    }

    if (!schunk_is_init){
        ROS_INFO("Schunk robot is not initialized.  Please initialize Schunk before attempting to execute a motion plan.");
        return false;
    }

    ROS_INFO("Joint Angles to execute: ");
    std::vector<double> to_execute;
    for(std::size_t i = 0; i < joint_values_.size(); i++){
        ROS_INFO("Joint %s: %f", joint_names_[i].c_str(), joint_values_[i]);
    }

    double joint_val = joint_values_[0]-2.24;
    if(joint_val < -2.95){
      to_execute.push_back(joint_val+2*M_PI);
    }else{
      to_execute.push_back(joint_val);//-2.24
    }

    ROS_INFO_STREAM("joint 1 value: " << to_execute[0]);

    to_execute.push_back(joint_values_[1]+0.45);
    to_execute.push_back(joint_values_[3]+0.49);
    to_execute.push_back(joint_values_[2]+0.22);
    to_execute.push_back(joint_values_[4]-0.10);
    to_execute.push_back(joint_values_[5]);

    send_to_pos(to_execute);

    have_joint_vals = false;
    is_motion_simulated = false;
    return true;
}

//simulate the motion, only works if a motion plan was previously calculated
bool Schunk::simulate_motion(){

    if (!got_plan){
        ROS_INFO("Plan not ready! Cannot execute motion.");
        return false;
    }
    std::vector<double> joint_values;
    display_trajectory_.trajectory.clear(); // clear trajectory vector, not sure if this is needed or does anything

    group_->execute(plan_);
    got_plan = false;
    is_motion_simulated = true;
    group_->clearPoseTargets();
    kinematic_state_->copyJointGroupPositions(joint_model_group_, joint_values);
    jointStatesCallback(joint_values);

    set_eef();

    ROS_INFO("Motion successfully executed!");
    sleep(5.0);
    is_motion_simulated = true;
    return true;
}

//reset the current trajectory that the arm will enact
void Schunk::reset_trajectory(){
    display_trajectory_.trajectory.clear();
}

//display the trajectory in RViz
void Schunk::display_trajectory(){
    if (!got_plan){
        ROS_INFO("Plan not ready! Cannot display trajectory.");
        return;
    }

    display_trajectory_.trajectory_start = plan_.start_state_;
    display_trajectory_.trajectory.push_back(plan_.trajectory_);

    display_publisher_.publish(display_trajectory_);
    std::cout << "after publisher" << std::endl;
}

//create a pose for the input values
geometry_msgs::Pose Schunk::create_pose(double pos_x, double pos_y, double pos_z, double orient_x, double orient_y, double orient_z, double orient_w){
    geometry_msgs::Pose pose;
    pose.orientation.w = orient_w;
    pose.orientation.x = orient_x;
    pose.orientation.y = orient_y;
    pose.orientation.z = orient_z;
    pose.position.x = pos_x;
    pose.position.y = pos_y;
    pose.position.z = pos_z;
    return pose;
}

//add a collision object to the world
void Schunk::add_object_to_world(moveit_msgs::CollisionObject object){
    ROS_INFO("Adding object to world");
    collision_objects.push_back(object);
    ROS_INFO("Adding object to collision object vector");
    planning_scene_interface.addCollisionObjects(collision_objects);
    kinematic_state_->update();
    ROS_INFO("Object successfully added");
    sleep(2.0);

    return;

}

//create a primitize box, note that the primitive can be changed to other primitives
moveit_msgs::CollisionObject Schunk::create_box(std::string id, double size, geometry_msgs::Pose pose, std::vector<double> dims){
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "base_link";

    for(unsigned int i = 0; i < id.size(); i++){
        collision_object.id.push_back(id[i]);
    }

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(size*3);
    primitive.dimensions[0] = dims[0];
    primitive.dimensions[1] = dims[1];
    primitive.dimensions[2] = dims[2];


    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    collision_object.operation = collision_object.ADD;
    return collision_object;
}

//remove an object from the scene
void Schunk::remove_object_from_world(std::string id){
    std::vector<std::string> object_ids;
    object_ids.push_back(id);
    planning_scene_interface.removeCollisionObjects(object_ids);
    sleep(4.0);
    return;
}

//attach the object to the robot
void Schunk::add_object_to_robot(std::string id){
    group_->attachObject(id);
    sleep(4.0);
}

//remove the object from the robot
void Schunk::remove_object_from_robot(std::string id){
    group_->detachObject(id);
    sleep(4.0);
}

//set the arm to random values
void Schunk::randomize_joint_values(){
    kinematic_state_->setToRandomPositions(joint_model_group_);
    set_eef();
}

//print out the joint values that the arm is currently at
void Schunk::print_joint_values(){
    std::vector<double> joint_values;
    kinematic_state_->copyJointGroupPositions(joint_model_group_, joint_values);

    for(std::size_t i = 0; i < joint_values.size(); i++){
        ROS_INFO("Joint %s: %f", joint_names_[i].c_str(), joint_values[i]);
    }


}

//reset the values that the arm is currently at - set to home
void Schunk::reset_joint_values(){
    std::vector<double> joint_values;
    kinematic_state_->copyJointGroupPositions(joint_model_group_, joint_values);

    for(std::size_t i = 0; i < joint_values.size(); i++){
        joint_values[i] = 0;
    }
    kinematic_state_->setJointGroupPositions(joint_model_group_, joint_values);
    joint_values_.clear();
    for(unsigned int i = 0; i < joint_values.size(); i++){
        joint_values_.push_back(joint_values[i]);
    }
    set_eef();

}



