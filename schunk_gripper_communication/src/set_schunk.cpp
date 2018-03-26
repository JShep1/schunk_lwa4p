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


Schunk::Schunk(ros::NodeHandle* nodehandle):node_handle(*nodehandle)
{
    this->initialize();
}   

Schunk::Schunk(){

}

Schunk::~Schunk(){

}

void Schunk::set_eef(){
    const Eigen::Affine3d &eef_state = kinematic_state_->getGlobalLinkTransform("arm_6_link");
    tf::poseEigenToMsg(eef_state, eef_pose_);
}

void Schunk::initialize(){
    ROS_INFO("instantiating Schunk arm");

    group_.reset(new moveit::planning_interface::MoveGroup("Arm"));
    group_->setPlanningTime(5);

    //joint_states_sub_ = node_handle.subscribe<sensor_msgs::JointState>("joint_states",1,boost::bind(&Schunk::jointStatesCallback, this, _1));

    //joint_states_sub_ = node_handle.subscribe("joint_states",1, Schunk::jointStatesCallback);

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
    //set_eef();

    //joint_state_group_ = kinematic_state_->getJointStateGroup("Arm");

    //     group.setStateState(*group.getCurrentState());
}

void Schunk::print_transform(tf::Transform trans, std::string info)
 {
   ROS_INFO_STREAM(info << " position: " << trans.getOrigin().getX() << " "
                   << trans.getOrigin().getY() << " "<< trans.getOrigin().getZ());
   ROS_INFO_STREAM(info <<" orientation: " << trans.getRotation().getX() << " "
                   << trans.getRotation().getY() << " "<< trans.getRotation().getZ() << " "
                   << trans.getRotation().getW());
}

void Schunk::print_pose(geometry_msgs::Pose pose, std::string info)
{
   ROS_INFO_STREAM(info << " position: " << pose.position.x << " "
                   << pose.position.y << " "<< pose.position.z);
   ROS_INFO_STREAM(info << " orientation: " << pose.orientation.x << " "
                   << pose.orientation.y << " "<< pose.orientation.z << " "
                     << pose.orientation.w);
}
 
bool Schunk::getIK(geometry_msgs::Pose eef_pose){

    if(got_robot_state){
        //kinematic_state_->setStateValues(*robot_state_);
        kinematic_state_->setJointGroupPositions(joint_model_group_, joint_values_);
        got_robot_state = false;
    }else{
        ROS_WARN("[Schunk::getIK] Using the default robot joint state NOT current");
    }


    kinematic_state_->enforceBounds();
    
    Eigen::Affine3d pose_in;
    tf::poseMsgToEigen(eef_pose, pose_in);

    print_pose(eef_pose, "eef_pose");
    bool found_ik = kinematic_state_->setFromIK(joint_model_group_, pose_in, 10, 0.5);    

    return found_ik;
}

void Schunk::jointStatesCallback(std::vector<double> joint_values){
    //robot_state_ = msg;
    //kinematic_state_->setJointGroupPositions(joint_model_group_, joint_values);
    //copy from joint_values to joint_values_
    joint_values_.clear();
    for(unsigned int i = 0; i < joint_values.size(); i++){
        joint_values_.push_back(joint_values[i]);
    }

    got_robot_state = true;
}

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

    //display_trajectory();
    
    sleep(5.0);

    

    got_plan = success;
    return success;
}

bool Schunk::execute_motion(){
    if (!got_plan){
        ROS_INFO("Plan not ready! Cannot execute motion.");
        return false;
    }
    
    display_trajectory_.trajectory.clear(); // clear trajectory vector, not sure if this is needed or does anything - current attempt at moving to position 2 from a position 1 where position 1 is the pos moved to after home pos

    group_->execute(plan_);
    got_plan = false;
    group_->clearPoseTargets();
    set_eef();
    ROS_INFO("Motion successfully executed!");
    return true;
}

void Schunk::reset_trajectory(){
    //TODO set display_trajectory_.trajectory_start to default state
    display_trajectory_.trajectory.clear();
}

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

geometry_msgs::Pose Schunk::create_pose(double x, double y, double z, double w){
    geometry_msgs::Pose pose;
    pose.orientation.w = w;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    return pose;
}

void Schunk::add_object_to_world(moveit_msgs::CollisionObject object, int index){
    ROS_INFO("Adding object to world"); 
    collision_objects.push_back(object);
    ROS_INFO("Adding object to collision object vector");
    kinematic_state_->update(); 
    planning_scene_interface.addCollisionObjects(collision_objects);
    ROS_INFO("Object successfully added"); 
    sleep(2.0); 
    return;
    //moveit_planning_scene_.world.collision_objects.push_back(attached_objects[attached_objects.size()-1].object);
    //moveit_planning_scene_.is_diff = true;
    //planning_scene_diff_publisher.publish(moveit_planning_scene_);

}



moveit_msgs::CollisionObject Schunk::create_box(std::string id, double size, geometry_msgs::Pose pose){
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "base_link";//group_->getPlanningFrame();
    
    for(unsigned int i = 0; i < id.size(); i++){
        collision_object.id.push_back(id[i]);
    }
    
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(size);
    primitive.dimensions[0] = 0.1;
    primitive.dimensions[1] = 0.1;
    primitive.dimensions[2] = 0.1;

   
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    collision_object.operation = collision_object.ADD;
    /*
    moveit_msgs::AttachedCollisionObject object;
    object.link_name = "base_link";
    object.object.header.frame_id = "base_link";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(size);
    primitive.dimensions[0] = 0.1;
    primitive.dimensions[1] = 0.1;
    primitive.dimensions[2] = 0.1;
    
    object.object.primitives.push_back(primitive);
    object.object.primitive_poses.push_back(pose);
    object.object.operation = object.object.ADD;
*/
    return collision_object;
}

void Schunk::remove_object_from_world(int index){
    std::vector<std::string> object_ids;
    object_ids.push_back(collision_objects[index].id);    
    planning_scene_interface.removeCollisionObjects(object_ids);
    sleep(4.0);
    return;
    /*
    moveit_msgs::CollisionObject remove_object;
    remove_object.id = id; // NOTICE: bugs may persist - copying of id may just be shallow copy within local frame
    //remove_object.header.frame_id = "base_link";
    remove_object.header.frame_id = group_.getPlanningFrame();
    remove_object.operation = remove_object.REMOVE;
    
    remove_objects[index] = remove_object;

    moveit_planning_scene_.world.collision_objects.push_back(remove_object);
    moveit_planning_scene_.robot_state.attached_collision_objects.push_back(attached_objects[index]);

    planning_scene_diff_publisher.publish(moveit_planning_scene_);
*/
}

void Schunk::add_object_to_robot(int index){
    group_->attachObject(collision_objects[index].id);
    sleep(4.0);
}

void Schunk::remove_object_from_robot(int index){
    group_->detachObject(collision_objects[index].id);
    sleep(4.0);
}

void Schunk::randomize_joint_values(){
    kinematic_state_->setToRandomPositions(joint_model_group_);
    set_eef();
}

void Schunk::print_joint_values(){
    std::vector<double> joint_values;
    kinematic_state_->copyJointGroupPositions(joint_model_group_, joint_values);
    
    for(std::size_t i = 0; i < joint_values.size(); i++){
        ROS_INFO("Joint %s: %f", joint_names_[i].c_str(), joint_values[i]);
    }
    

}

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



