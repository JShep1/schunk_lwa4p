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
#include <time.h>


//#include <planning_scene_interface.h>
//Schunk schunk;
boost::shared_ptr<Schunk> schunk;
//boost::shared_ptr<SchunkTest> schunk_test;
int num_boxes = 0;
int current_boxes = 0;

bool pick_box(std::string id);
bool place_box(std::string id);
bool remove_box(std::string id);
bool create_box(double x, double y, double z, 
        double xx, double yy, double zz, double ww, 
        double dim1, double dim2, double dim3, 
        double size, std::string id);
bool execute_motion();
bool plan_motion(double x, double y, double z, double xx, double yy, double zz);


bool init_schunk(){
    return true;
}

double frand(double fmin, double fmax){
    double f = (double) rand() / RAND_MAX;
    return fmin + f * (fmax-fmin);
}

bool test_planning(unsigned int numtests){
    
    unsigned int num_successes = 0;
    
    for(unsigned int i = 0; i < numtests; i++){
        if(plan_motion(-1500,-1500,-1500,-1500,-1500,-1500)){
            num_successes++;
        }
    }    

    double percentage_success = ((double)num_successes/numtests)*100;

    ROS_INFO_STREAM("Motion plan tests executed with "<<percentage_success<<"\% success");

    return true;
}

bool test_box(unsigned int numtests){
    
    srand(time(0));    

    for (unsigned int i = 0; i < numtests; i++){
        std::stringstream ss;
        ss << i;
        std::string box_id = ss.str();
        double x = frand(-1,1);
        double y = frand(-1,1);
        double z = frand(-1,1);
        bool is_removed = create_box(x, y, z, 0, 0, 0, 1, 0.1, 0.1, 0.1, 1, box_id);
        
        if(!is_removed){
            ROS_INFO("Test box failed.");
            return false;
        }
        
    }

    sleep(2.0);
    
    for (unsigned int i = 0; i < numtests; i++){
        std::stringstream ss;
        ss << i;
        std::string box_id = ss.str();
        bool is_removed = remove_box(box_id);;
        
        if(!is_removed){
            ROS_INFO("Test box failed.");
            return false;
        }
        
    }

    return true;
}

bool test_1(){
    
    bool found_plan = false;
    
    double waypoints[7][6] =  {
        {-0.71,  0.47,  2.51,  0.0, -0.41, 1.12},
        {-0.71,  0.30,  2.44,  0.0, -0.51, 1.12},
        {-0.71,  0.30,  2.44,  0.0, -0.62, 1.12},
        { 0.82,  0.30,  2.44,  0.0, -0.62, 1.12},
        { 0.82, -0.59,  2.44,  0.0, -1.45, 1.12},
        { 0.82, -0.59,  2.44,  0.0, -1.63, 1.12},
        {  0.0,   0.0,   0.0,  0.0,   0.0,  0.0}
    };
    
    bool box1 = create_box(0.3, -0.3, 0.25, 0, 0, 1, 1, 0.05, 0.05, 0.05, 1, "box1");
    bool box2 = create_box(0.3, -0.325, 0.12, 0, 0, 1, 1, 0.19, 0.25, 0.215, 1, "box2");
    bool box3 = create_box(0.4, 0.4, 0.03, 0, 0, 1, 1, 0.3, 0.25, 0.04, 1, "box3");

    if(!box1 || !box2 || !box3){
        ROS_INFO("Test 1 Unsuccessful. Returning.");
        return false;
    }

    for(unsigned int i = 0; i < sizeof(waypoints)/sizeof(waypoints[0]); i++){
        found_plan = plan_motion(waypoints[i][0],waypoints[i][1],
                                 waypoints[i][2],waypoints[i][3],
                                 waypoints[i][4],waypoints[i][5]);    

        if(!found_plan){
            ROS_INFO("Test 1 Unsuccessful. Returning.");
            return false;
        }

        execute_motion();
        
        if (i == 1){
            pick_box("box1");
        }else if(i == 4){
            place_box("box1");
        }
        
        
    }
   
    sleep(5.0);
    
    remove_box("box1");
    remove_box("box2");
    remove_box("box3");
 
    return true;

}

bool test_2(){

    bool found_plan = false;

    double waypoints[12][6] =  {
        {5.07,  0.0,   0.0,  0.0,   0.0,  0.0},
        {5.12, 1.76,  0.08, 1.66,  2.30, 2.90},
        {5.00, 1.76,  0.08, 1.66,  2.35, 2.90},
        {5.00, 1.65,  0.08, 1.48,  2.35, 2.72},
        {5.00, 1.06, -0.45, 1.48,  0.17, 2.72},
        {5.00, 0.29, -1.22, 1.48,  0.17, 2.72},
        {0.64, 0.29, -1.22, 1.48, -1.54, 2.72},
        {0.64, 1.30, -0.22, 1.48, -1.54, 2.72},
        //{0.75, 1.47, -0.16, 1.48, -2.13, 2.72},
        {0.87, 1.77,  0.08, 1.48, -2.31, 2.66},
        {0.52, 1.77,  0.08, 1.48, -2.31, 2.66},
        {0.52, 1.40,  0.08, 1.48, -2.31, 2.66},
        {0.0,   0.0,   0.0,  0.0,   0.0,  0.0}
    };

    bool box1 = create_box(0.4,  -0.25, 0.105, 0, 0, 1.1, 1, 0.06, 0.06,  0.1, 1, "box1");
    bool box2 = create_box(0.15, -0.45,  0.03, 0, 0, 1.1, 1,  0.5, 0.85, 0.04, 1, "box2");
    bool box3 = create_box(0.15,  -0.4,  0.15, 0, 0, 1.1, 1,  0.2,  0.2, 0.25, 1, "box3");


    if(!box1 || !box2 || !box3){
        ROS_INFO("Test 1 Unsuccessful. Returning.");
        return false;
    }

    for(unsigned int i = 0; i < sizeof(waypoints)/sizeof(waypoints[0]); i++){
        found_plan = plan_motion(waypoints[i][0],waypoints[i][1],
                                 waypoints[i][2],waypoints[i][3],
                                 waypoints[i][4],waypoints[i][5]);    

        if(!found_plan){
            ROS_INFO("Test 1 Unsuccessful. Returning.");
            return false;
        }

        execute_motion();
        
        if (i == 2){
            pick_box("box1");
        }else if(i == 8){
            place_box("box1");
        }
        
        
    }
   
    sleep(5.0);
    
    remove_box("box1");
    remove_box("box2");
    remove_box("box3");
 
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

        schunk->print_pose(schunk->eef_pose_, "Before joint angles set");
        success = schunk->plan_motion(joint_angles);
        schunk->print_pose(schunk->eef_pose_, "After joint angles set");

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

bool create_box(double x, double y, double z, double xx, double yy, double zz, double ww, double dim1, double dim2, double dim3, double size, std::string id){

    std::string box_name;
    std::vector<double> dims;

    for(unsigned int i = 0; i < id.size(); i++){
        box_name.push_back(id[i]);
    }

    dims.push_back(dim1);
    dims.push_back(dim2);
    dims.push_back(dim3);


    current_boxes++;

    geometry_msgs::Pose box_pose = schunk->create_pose(x,y,z,xx,yy,zz,ww);

    moveit_msgs::CollisionObject box = schunk->create_box(box_name, size, box_pose, dims);


    schunk->add_object_to_world(box);

    if(current_boxes == 1){
        //increase planning time if collision objects are now in scene
        schunk->set_planning_time(10.0);
    }

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

bool query_joints(schunk_gripper_communication::schunk_gripper::Response &res){
    std::vector<double> joint_angles = schunk->get_joint_angles();
    res.eef_joint_1 = joint_angles[1];
    res.eef_joint_2 = joint_angles[2];
    res.eef_joint_3 = joint_angles[3];
    res.eef_joint_4 = joint_angles[4];
    res.eef_joint_5 = joint_angles[5];
    res.eef_joint_6 = joint_angles[6];

    return true;
}

bool query_eef(schunk_gripper_communication::schunk_gripper::Response &res){
    geometry_msgs::Pose eef_pose = schunk->eef_pose_;
    res.eef_joint_1 = eef_pose.position.x; 
    res.eef_joint_2 = eef_pose.position.y; 
    res.eef_joint_3 = eef_pose.position.z; 
    res.eef_joint_4 = eef_pose.orientation.x; 
    res.eef_joint_5 = eef_pose.orientation.y; 
    res.eef_joint_6 = eef_pose.orientation.z; 
    res.eef_orient_w = eef_pose.orientation.w; 

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
    function_names.push_back("query_eef");
    function_names.push_back("query_joints");
    function_names.push_back("test_1");
    function_names.push_back("test_2");
    function_names.push_back("test_box");
    function_names.push_back("test_planning");

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
        if(create_box(req.box_x, req.box_y, req.box_z, req.plan_xx, req.plan_yy, req.plan_zz, req.plan_ww, req.plan_x, req.plan_y, req.plan_z, req.box_size, req.id)){
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

    else if(!function_names[7].compare((std::string)req.function_name)){
        ROS_INFO("Found function query_eef");
        if(query_eef(res)){
            ROS_INFO("Successfully queried eef pose.");
            return true;
        }else{
            ROS_ERROR("Could not query eef pose. Returning.");
            return false;
        }

    }

    else if(!function_names[8].compare((std::string)req.function_name)){
        ROS_INFO("Found function query_joints");
        if(query_joints(res)){
            ROS_INFO("Successfully queried joint angles.");
            return true;
        }else{
            ROS_ERROR("Could not query joint angles. Returning.");
            return false;
        }

    }

    else if(!function_names[9].compare((std::string)req.function_name)){
        ROS_INFO("Found function test_1");
        if(test_1()){
            ROS_INFO("Successfully executed test 1 scenario.");
            return true;
        }else{
            ROS_ERROR("Test 1 could not execute. Returning.");
            return false;
        }

    }

    else if(!function_names[10].compare((std::string)req.function_name)){
        ROS_INFO("Found function test_2");
        if(test_2()){
            ROS_INFO("Successfully executed test 2 scenario.");
            return true;
        }else{
            ROS_ERROR("Test 2 could not execute. Returning.");
            return false;
        }

    }
    
    else if(!function_names[11].compare((std::string)req.function_name)){
        ROS_INFO("Found function test_box");
        
        int numtests = req.index; 

        if(test_box(numtests)){
            ROS_INFO("Successfully executed box test.");
            return true;
        }else{
            ROS_ERROR("Test 2 could not execute. Returning.");
            return false;
        }

    }
    
    else if(!function_names[12].compare((std::string)req.function_name)){
        ROS_INFO("Found function test_planning");
        
        int numtests = req.index;     
        
        if(test_planning(numtests)){
            ROS_INFO("Successfully executed planning test.");
            return true;
        }else{
            ROS_ERROR("Test 2 could not execute. Returning.");
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

    ros::ServiceServer service = n.advertiseService("choose_function", choose_function);
    ROS_INFO("Ready to run a function.");
    ros::spin();

    return 0;
}

