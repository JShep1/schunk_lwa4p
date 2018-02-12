#ifndef SET_SCHUNK_
#define SET_SCHUNK_

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


class Schunk{
    public:
     //   void move_to_pose(int init_indx, int end_indx, double sleep_duration);
     //   void move_to_after_pose();
     //   void get_eef_pose();

        geometry_msgs::Pose before_eef_pose;
        geometry_msgs::Pose eef_pose;
        geometry_msgs::Pose after_eef_pose;

        std::vector<geometry_msgs::Pose> poses;
        std::vector<double> delays;
        std::vector<moveit_msgs::AttachedCollisionObject> attached_objects;

        Schunk();

        ~Schunk();

        void initialize();

private:
    ros::Subscriber joint_states_sub_;
    ros::NodeHandle node_handle;
    ros::Publisher display_publisher;
    ros::Publisher planning_scene_diff_publisher;
    //sensor_msgs::JointState::ConstPtr robot_state_;
    bool got_robot_state;

    robot_model_loader::RobotModelLoader robot_model_loader_;
    robot_model::RobotModelPtr robot_model_;
    robot_state::RobotStatePtr kinematic_state_;
    //robot_state::JointStateGroup* joint_state_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    boost::scoped_ptr<moveit::planning_interface::MoveGroup> group_;
    moveit_msgs::DisplayTrajectory display_trajectory;

    //void jointStatesCallback(const sensor_msgs::JointStateConstPtr& msg);
   // trajectory_msgs::JointTrajectoryPointPtr create_traj_point(std::vector<double> joint_values, double time);
 //   bool getIK(geometry_msgs::Pose eef_pose);


};

#endif
