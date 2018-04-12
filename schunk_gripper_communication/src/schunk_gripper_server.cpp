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
//#include <moveit/robot_state/joint_state_group.h>
#include <moveit/robot_state/robot_state.h>
#include <eigen_conversions/eigen_msg.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <moveit/collision_detection/collision_common.h>
#include <Eigen/Geometry>
#include <schunk/set_schunk.h>
//#include <schunk_test/schunk_test.h>
#include <sensor_msgs/JointState.h>

//#include <planning_scene_interface.h>
//Schunk schunk;
boost::shared_ptr<Schunk> schunk;
//boost::shared_ptr<SchunkTest> schunk_test;
int num_boxes = 0;
int current_boxes = 0;

bool init_schunk(){
    return true;
}

bool test_motion_plan(double numtests){
    //TODO implement test that calls for numtests number
    //     of motion plans and reports percentages of success
    return false;
}

bool test_box_place(double numtests){
    //TODO implement test that calls for numtests number
    //     of create_box's and reports percentages of success
    return false;
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
    
  //  Py_Initialize();
  //  PyRun_SimpleString(str1);
  //  Py_Finalize();
    
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

bool plan_motion(double x, double y, double z, double xx, double yy, double zz){
    bool is_rand = false, is_eef_plan = false, success = false; 

    if (x == -1500 && y == -1500 && z == -1500){
        is_rand = true;
    }

    if (xx == -1500 && yy == -1500 && zz == -1500){
        is_eef_plan = true;
    }

    geometry_msgs::Pose eef_pose;

    if(!is_eef_plan && !is_rand){
        //we want to plan for a joint angle configuration
        ROS_INFO("Joint angle planning initiated"); 
        std::vector<double> joint_angles;
        joint_angles.push_back(x); 
        joint_angles.push_back(y); 
        joint_angles.push_back(z); 
        joint_angles.push_back(xx); 
        joint_angles.push_back(yy); 
        joint_angles.push_back(zz); 
    
        success = schunk->plan_motion(joint_angles);
        
    }else if(is_rand){
        
        schunk->print_pose(schunk->eef_pose_, "Before randomize");
    
        schunk->randomize_joint_values();
        schunk->print_pose(schunk->eef_pose_, "After randomize");
        eef_pose = 
            schunk->create_pose(
                    schunk->eef_pose_.position.x,
                    schunk->eef_pose_.position.y,
                    schunk->eef_pose_.position.z,
                    schunk->eef_pose_.orientation.x,
                    schunk->eef_pose_.orientation.y,
                    schunk->eef_pose_.orientation.z,
                    schunk->eef_pose_.orientation.w
                    );
        
        schunk->reset_joint_values();
        success = schunk->plan_motion(eef_pose);

    }else{
        ROS_INFO("EEF pose planning initiated");
        eef_pose = schunk->create_pose(x,y,z,0,0,0,1);
        //z is orient about y or x
        //w orient is rot about z
        success = schunk->plan_motion(eef_pose);
        
    }

    if(success){
        ROS_INFO("Found motion plan");
        return true;
        //schunk.execute_motion();
    }
    else{
        ROS_INFO("No found motion plan");
        return false;
    }
}


bool execute_motion(){
    return schunk->execute_motion();
}

bool create_box(double x, double y, double z, std::string id){

    std::string box_name;

    for(unsigned int i = 0; i < id.size(); i++){
        box_name.push_back(id[i]);
    }
    
    current_boxes++;

    geometry_msgs::Pose box_pose = schunk->create_pose(x,y,z,0,0,0,1);

    moveit_msgs::CollisionObject box = schunk->create_box(box_name, 1, box_pose);

    
    schunk->add_object_to_world(box);

    if(current_boxes == 1){
        //increase planning time if collision objects are now in scene
        schunk->set_planning_time(10.0);
    }
    //TODO might need to set planning time to 10 here with group_->setPlanningTime(10.0);

    return true;
}

bool remove_box(std::string id){
    
    schunk->remove_object_from_world(id); 
    if(current_boxes > 0){
        current_boxes--;
    }

    if(!current_boxes){
        //set planning time back to normal
        schunk->set_planning_time(5.0);
    }
    return true;
}

bool pick_box(std::string id){
    
    schunk->add_object_to_robot(id); 
 
    return true;
}

bool place_box(std::string id){
    
    schunk->remove_object_from_robot(id);

    return true;
}

bool choose_function(schunk_gripper_communication::schunk_gripper::Request &req,
        schunk_gripper_communication::schunk_gripper::Response &res)
{
    std::vector<std::string> function_names;
    function_names.push_back("set_motor");
    function_names.push_back("plan_motion");
    function_names.push_back("execute_motion");
    function_names.push_back("create_box");
    function_names.push_back("remove_box");
    function_names.push_back("pick_box");
    function_names.push_back("place_box");
    
    if(!function_names[0].compare((std::string)req.function_name)){
        res.motorvalue = req.motorvalue;
        ROS_INFO("Found function set_motor");
        if(set_motor(res.motorvalue)){
            ROS_INFO("Successfully set motor.");
            return true;
        }else{
            ROS_ERROR("Could not set motor. Returning.");
            return false;
        }
    }else if(!function_names[1].compare((std::string)req.function_name)){
        ROS_INFO("Found function plan_motion");
        bool success = false;

        if ((req.plan_xx + 1500) < 0.001 && (req.plan_xx + 1500) > 0.001){
            success = plan_motion(req.plan_x, req.plan_y, req.plan_z, -1500, -1500, -1500);
        }else{
            success = plan_motion(req.plan_x, req.plan_y, req.plan_z, req.plan_xx, req.plan_yy, req.plan_zz);
        }

        if(success){
            ROS_INFO("Successfully created motion plan.");
            return true;
        }else{
            ROS_ERROR("Could not create motion plan. Returning.");
            return false;
        }

    } 
    
    else if(!function_names[2].compare((std::string)req.function_name)){
        ROS_INFO("Found function execute_motion");
        if(execute_motion()){
            ROS_INFO("Successfully executed motion.");
            return true;
        }else{
            ROS_ERROR("Could not execute motion. Returning.");
            return false;
        }

    }
    
    else if(!function_names[3].compare((std::string)req.function_name)){
        ROS_INFO("Found function create_box");
        if(create_box(req.box_x, req.box_y, req.box_z, req.id)){
            ROS_INFO("Successfully created box.");
            return true;
        }else{
            ROS_ERROR("Could not place box. Returning.");
            return false;
        }

    }
    
    else if(!function_names[4].compare((std::string)req.function_name)){
        ROS_INFO("Found function remove_box");
        if(remove_box(req.id)){
            ROS_INFO("Successfully removed box.");
            return true;
        }else{
            ROS_ERROR("Could not remove box. Returning.");
            return false;
        }

    }
    else if(!function_names[5].compare((std::string)req.function_name)){
        ROS_INFO("Found function pick_box");
        if(pick_box(req.id)){
            ROS_INFO("Successfully removed box.");
            return true;
        }else{
            ROS_ERROR("Could not remove box. Returning.");
            return false;
        }

    }
    else if(!function_names[6].compare((std::string)req.function_name)){
        ROS_INFO("Found function place_box");
        if(place_box(req.id)){
            ROS_INFO("Successfully removed box.");
            return true;
        }else{
            ROS_ERROR("Could not remove box. Returning.");
            return false;
        }

    }

    else{
        ROS_ERROR("No such function: %s", req.function_name.c_str());
        return false;
    }
    return true;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "schunk_gripper_server");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    schunk.reset(new Schunk(&n));
//    schunk_test.reset(new SchunkTest(&schunk));
    //Schunk schunk(&n);
    //(&schunk)->~Schunk();
    //new (&schunk) Schunk(&n); 
    
    //geometry_msgs::Pose eef_pose = schunk->create_pose(1,1,1,1,1,1,1);
    //schunk->print_pose(eef_pose, "eef from main");
    //sleep(7.0);
    

    /*
    for(int i = 0; i < 10; i++){ 
        schunk.print_pose(schunk.eef_pose_, "schunk's before eef");
        schunk.print_joint_values();
        schunk.randomize_joint_values();
        //std::cout << "eef #" << i << std::endl;
        schunk.print_pose(schunk.eef_pose_, "schunk's after eef");
        schunk.print_joint_values();
        
        geometry_msgs::Pose ik_eef_pose = schunk.create_pose(schunk.eef_pose_.position.x,schunk.eef_pose_.position.y,schunk.eef_pose_.position.z,schunk.eef_pose_.orientation.w);
        
        schunk.reset_joint_values();
        if(schunk.plan_motion(ik_eef_pose)){
            std::cout << "Found motion plan" << std::endl;
            //schunk.execute_motion();
        }
        else{
            std::cout << "No found motion plan" <<std::endl;
            i--;
        }

        sleep(5.0);
        ROS_INFO_STREAM( "----------------" << i << "------------------------");
        if(schunk.getIK(ik_eef_pose)){
            std::cout << "IK found with solution" <<std::endl;
            schunk.print_joint_values();
        }else{
            std::cout << "IK not found" <<std::endl;
        }
    }
        */

    //TODO: 
    //      DONE - Generate random joint values for each joint and get the eef pose
    //      DONE - Get multiple eef poses and attempt to plan for each one
    //      - start testing with plan_motion function and visualization in rviz
    //      - make above tests into a unit test class or function
    
    //geometry_msgs::Pose eef_pose = schunk.create_pose(0.5, 0.5, 0.5, 1);
    //moveit_msgs::CollisionObject box = schunk.create_box("box1", 1, eef_pose);
    
    //schunk.add_object_to_world(box, 0);
        

    //schunk.plan_motion(eef_pose);
    //ROS_INFO("Planned motion");
    ros::ServiceServer service = n.advertiseService("choose_function", choose_function);
    ROS_INFO("Ready to run a function.");
    ros::spin();
    return 0;
}

