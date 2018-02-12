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


        Schunk::Schunk()
        {

        }   

        Schunk::~Schunk(){

        }

        void Schunk::initialize(){
            ROS_INFO("instantiating Schunk arm");

//            robot_state_.reset(new sensor_msgs::JointState());
            //group = new moveit::planning_interface::MoveGroup("Arm");
            group_.reset(new moveit::planning_interface::MoveGroup("Arm"));
            group_->setPlanningTime(5);
            
            display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path",1,true);
            planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene",1);
            //attached_object.link_name = "base_link";
            //attached_object.object.header.frame_id = "base_link";
            //attached_object.object.id = "box";

       //     group.setStateState(*group.getCurrentState());
        }



