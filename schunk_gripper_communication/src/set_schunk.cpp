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


Schunk::Schunk(ros::NodeHandle* nodehandle):node_handle(*nodehandle)
{
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

    group_.reset(new moveit::planning_interface::MoveGroup("Arm"));
    group_->setPlanningTime(5);


    got_robot_state = false;
    got_plan = false;

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

    return success;
}

//execute the motion, only works if a motion plan was previously calculated
bool Schunk::execute_motion(){

    if (!got_plan){
        ROS_INFO("Plan not ready! Cannot execute motion.");
        return false;
    }
    std::vector<double> joint_values;
    display_trajectory_.trajectory.clear(); // clear trajectory vector, not sure if this is needed or does anything

    group_->execute(plan_);
    got_plan = false;
    group_->clearPoseTargets();
    kinematic_state_->copyJointGroupPositions(joint_model_group_, joint_values);
    jointStatesCallback(joint_values);

    set_eef();

    ROS_INFO("Motion successfully executed!");
    sleep(5.0);
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



