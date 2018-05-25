#ifndef SET_SCHUNK_
#define SET_SCHUNK_

#include <stdlib.h>
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
#include <sensor_msgs/JointState.h>
#include <boost/filesystem.hpp>
#include <icl_hardware_canopen/CanOpenController.h>
#include <icl_hardware_canopen/SchunkPowerBallNode.h>

using namespace icl_hardware::canopen_schunk;

class Schunk{
  public:

    bool getIK(geometry_msgs::Pose eef_pose);
    bool plan_motion(geometry_msgs::Pose eef_pose);
    bool plan_motion(std::vector<double> joint_angles);
    bool simulate_motion();
    bool execute_motion();
    bool shutdown_schunk();
    bool send_to_pos(std::vector<double> joint_angles);
    geometry_msgs::Pose get_eef();
    std::vector<double> get_joint_angles();
    void print_transform(tf::Transform trans, std::string info);
    void print_pose(geometry_msgs::Pose pose, std::string info);
    void display_trajectory();
    void jointStatesCallback(std::vector<double> joint_values);
    void add_object_to_world(moveit_msgs::CollisionObject object);
    void remove_object_from_world(std::string id);
    void add_object_to_robot(std::string id);
    void remove_object_from_robot(std::string id);
    void randomize_joint_values();
    void print_joint_values();
    void reset_joint_values();
    void reset_trajectory();
    void set_planning_time(double time);
    bool init_schunk();

    geometry_msgs::Pose create_pose(double pos_x, double pos_y, double pos_z, double orient_x, double orient_y, double orient_z, double orient_w); 

    moveit_msgs::CollisionObject create_box(std::string id, double size, geometry_msgs::Pose pose, std::vector<double> dims);

    geometry_msgs::Pose before_eef_pose_;
    geometry_msgs::Pose eef_pose_;
    geometry_msgs::Pose after_eef_pose_;
    std::vector<double> joint_values_;

    std::vector<geometry_msgs::Pose> poses;
    std::vector<double> delays;
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    Schunk(ros::NodeHandle* nodehandle);
    Schunk();

    ~Schunk();

    void initialize();

  private:
    void set_eef();
    trajectory_msgs::JointTrajectoryPointPtr create_traj_point(std::vector<double> joint_values, double time);

    ros::Subscriber joint_states_sub_;
    ros::NodeHandle node_handle;
    ros::NodeHandle* priv_nh;
    ros::Publisher display_publisher_;
    ros::Publisher planning_scene_diff_publisher;
    bool got_robot_state;
    bool got_plan;
    bool have_joint_vals;
    bool schunk_is_init;
    bool is_motion_simulated;

    robot_model_loader::RobotModelLoader robot_model_loader_;
    robot_model::RobotModelPtr robot_model_;
    robot_state::RobotStatePtr kinematic_state_;
    const robot_state::JointModelGroup* joint_model_group_;
    std::vector<std::string> joint_names_;


    boost::scoped_ptr<planning_scene::PlanningScene> planning_scene_;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_msgs::PlanningScene moveit_planning_scene_;
    boost::scoped_ptr<moveit::planning_interface::MoveGroup> group_;
    moveit_msgs::DisplayTrajectory display_trajectory_;
    moveit::planning_interface::MoveGroup::Plan plan_;

    CanOpenController::Ptr my_controller;
    std::string can_device_name;
    std::vector<std::string> chain_names;
    std::map<std::string, std::vector<int> > chain_configurations;
    std::vector<DS402Group::Ptr> chain_handles;
    sensor_msgs::JointState joint_msg;
    ds402::ProfilePositionModeConfiguration ppm_config;
    std::map<std::string, uint8_t> joint_name_mapping;





};

#endif
