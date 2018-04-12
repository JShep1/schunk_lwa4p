
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
#include <schunk_test/schunk_test.h>
#include <schunk/set_schunk.h>

        //Schunk(ros::NodeHandle* nodehandle);

SchunkTest::SchunkTest(Schunk* schunk){
    this->schunk = schunk;
}

SchunkTest::SchunkTest(){

}

SchunkTest::~SchunkTest(){

}

void SchunkTest::initialize(){
    
}

