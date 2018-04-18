#include "ros/ros.h"
#include "schunk_gripper_communication/schunk_gripper.h"
#include <cstdlib>
#include <stdlib.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "schunk_gripper_client");
    schunk_gripper_communication::schunk_gripper srv;

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

    bool is_eef_query = false, is_joints_query = false;

    std::string cmd_arg = argv[1];


    if(!function_names[0].compare(cmd_arg)){
        //set motor function
        if (argc == 3){
            srv.request.function_name = argv[1];
            srv.request.motorvalue = atof(argv[2]);
        }else{
            ROS_ERROR("Invalid set_motor call");
            ROS_ERROR("Usage: schunk_gripper_client set_motor *motor value*");
            return 0;
        }
    }
    else if(!function_names[1].compare(cmd_arg)){
        //plan motion function
        if(argc == 2){
            //no coordinates supplied so random plan
            srv.request.function_name = argv[1];
            srv.request.plan_x = -1500;
            srv.request.plan_y = -1500;
            srv.request.plan_z = -1500;            
            srv.request.plan_xx = -1500;            
            srv.request.plan_yy = -1500;            
            srv.request.plan_zz = -1500;            
        }else if(argc == 5){
            //x, y, and z supplied so go there
            srv.request.function_name = argv[1];
            double x = atof(argv[2]);
            double y = atof(argv[3]);
            double z = atof(argv[4]);
            srv.request.plan_xx = -1500;            
            srv.request.plan_yy = -1500;            
            srv.request.plan_zz = -1500;            

            srv.request.plan_x = x; 
            srv.request.plan_y = y; 
            srv.request.plan_z = z; 

        }else if(argc == 8){

            srv.request.function_name = argv[1];
            double x = atof(argv[2]);
            double y = atof(argv[3]);
            double z = atof(argv[4]);
            double xx = atof(argv[5]);
            double yy = atof(argv[6]);
            double zz = atof(argv[7]);

            srv.request.plan_x = x; 
            srv.request.plan_y = y; 
            srv.request.plan_z = z; 
            srv.request.plan_xx = xx; 
            srv.request.plan_yy = yy; 
            srv.request.plan_zz = zz; 

        }else{
            //Invalid number of args
            ROS_ERROR("Invalid plan_motion call");
            ROS_ERROR("For random plan:");
            ROS_ERROR("Usage: schunk_gripper_client plan_motion");
            ROS_ERROR("For specified plan using eef coords:");
            ROS_ERROR("Usage: schunk_gripper_client plan_motion *x_val* *y_val* *z_val*");
            ROS_ERROR("For specified plan using joint angles:");
            ROS_ERROR("Usage: schunk_gripper_client plan_motion *joint_val_1* *val_2* *val_3* *val_4* *val_5* *val_6*");
            return 0;

        }
    }
    else if(!function_names[2].compare(cmd_arg)){
        //execute motion function
        srv.request.function_name = argv[1];

    }
    else if(!function_names[3].compare(cmd_arg)){
        //create box function
        srv.request.function_name = argv[1];
        if(argc == 14){

            double box_x = atof(argv[2]);
            double box_y = atof(argv[3]);
            double box_z = atof(argv[4]);
            double box_xx = atof(argv[5]);
            double box_yy = atof(argv[6]);
            double box_zz = atof(argv[7]);
            double box_ww = atof(argv[8]);

            double dim_1 = atof(argv[9]);
            double dim_2 = atof(argv[10]);
            double dim_3 = atof(argv[11]);
            
            double size = atof(argv[12]);

            std::string id = argv[13];

            srv.request.box_x = box_x; 
            srv.request.box_y = box_y; 
            srv.request.box_z = box_z; 

            srv.request.plan_xx = box_xx;
            srv.request.plan_yy = box_yy;
            srv.request.plan_zz = box_zz;
            srv.request.plan_ww = box_ww;

            srv.request.plan_x = dim_1;
            srv.request.plan_y = dim_2;
            srv.request.plan_z = dim_3;

            srv.request.box_size = size;

            srv.request.id = id;

        }else{
            ROS_ERROR("Invalid create_box call");
            ROS_ERROR("Usage: schunk_gripper_client create_box *x_val* *y_val* *z_val* *x_orient* *y_orient* *z_orient* *w_orient* *dim_1* *dim_2* *dim_3* *box_size* *id*");
            return 0;
        }

    }
    else if(!function_names[4].compare(cmd_arg)){
        //remove box function
        srv.request.function_name = argv[1];
        if (argc == 3){
            srv.request.id = argv[2];
        }else{
            ROS_ERROR("Invalid remove_box call");
            ROS_ERROR("Usage: schunk_gripper_client remove_box *id*");
            return 0;
        }

    }
    else if(!function_names[5].compare(cmd_arg)){
        //pick box function
        srv.request.function_name = argv[1];
        if (argc == 3){
            srv.request.id = argv[2];
        }else{
            ROS_ERROR("Invalid pick_box call");
            ROS_ERROR("Usage: schunk_gripper_client pick_box *id*");
            return 0;
        }

    }
    else if(!function_names[6].compare(cmd_arg)){
        //place box function
        srv.request.function_name = argv[1];
        if (argc == 3){
            srv.request.id = argv[2];
        }else{
            ROS_ERROR("Invalid place_box call");
            ROS_ERROR("Usage: schunk_gripper_client place_box *id*");
            return 0;
        }

    }

    else if(!function_names[7].compare(cmd_arg)){
        //query eef function
        is_eef_query = true;
        srv.request.function_name = argv[1];
        if (argc != 2){
            ROS_ERROR("Invalid query_eef call");
            ROS_ERROR("Usage: schunk_gripper_client query_eef");
            return 0;
        }

    }

    else if(!function_names[8].compare(cmd_arg)){
        //query joint angles function
        is_joints_query = true;
        srv.request.function_name = argv[1];
        if (argc != 2){
            ROS_ERROR("Invalid query_joints call");
            ROS_ERROR("Usage: schunk_gripper_client query_joints");
            return 0;
        }

    }
    
    else if(!function_names[9].compare(cmd_arg)){
        //test 1 function
        srv.request.function_name = argv[1];

    }
    
    else if(!function_names[10].compare(cmd_arg)){
        //test 2 function
        srv.request.function_name = argv[1];

    }
    
    else if(!function_names[11].compare(cmd_arg)){
        //test box function
        srv.request.function_name = argv[1];
        if (argc == 3){
            srv.request.index = atoi(argv[2]);
            if(srv.request.index < 1){
                ROS_ERROR("Invalid number for numtests");
                return 0;
            }
        }else{
            ROS_ERROR("Invalid test_box call");
            ROS_ERROR("Usage: schunk_gripper_client test_box *numtests*");
            return 0;
        }

    }
    
    else if(!function_names[12].compare(cmd_arg)){
        //test plan function
        srv.request.function_name = argv[1];
        if (argc == 3){
            srv.request.index = atoi(argv[2]);
            if(srv.request.index < 1){
                ROS_ERROR("Invalid number for numtests");
                return 0;
            }
        }else{
            ROS_ERROR("Invalid test_planning call");
            ROS_ERROR("Usage: schunk_gripper_client test_planning *numtests*");
            return 0;
        }

    }

    else{
        ROS_ERROR("Invalid function call.");    
        ROS_ERROR("Valid function calls are set_motor, plan_motion, execute_motion, create_box");   
        return 0; 
    }

    //setenv("PYTHONPATH",".",1);
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<schunk_gripper_communication::schunk_gripper>("choose_function");


    //ROS_INFO("input: %s %s %f",srv.request.pythonfile, srv.request.pythonfunction,(double)srv.request.motorvalue);

    if (client.call(srv)){
        ROS_INFO("Function %s executed successfully", srv.request.function_name.c_str());
        if(is_eef_query){
            //TODO print eef pos and orient
            ROS_INFO_STREAM("eef x pos: " << srv.response.eef_joint_1);
            ROS_INFO_STREAM("eef y pos: " << srv.response.eef_joint_2);
            ROS_INFO_STREAM("eef z pos: " << srv.response.eef_joint_3);
            ROS_INFO_STREAM("eef x orient: " << srv.response.eef_joint_4);
            ROS_INFO_STREAM("eef y orient: " << srv.response.eef_joint_5);
            ROS_INFO_STREAM("eef z orient: " << srv.response.eef_joint_6);
            ROS_INFO_STREAM("eef w orient: " << srv.response.eef_orient_w);
        }
        else if(is_joints_query){
            //TODO print joint angles
            ROS_INFO_STREAM("Joint 1: " << srv.response.eef_joint_1);
            ROS_INFO_STREAM("Joint 2: " << srv.response.eef_joint_2);
            ROS_INFO_STREAM("Joint 3: " << srv.response.eef_joint_3);
            ROS_INFO_STREAM("Joint 4: " << srv.response.eef_joint_4);
            ROS_INFO_STREAM("Joint 5: " << srv.response.eef_joint_5);
            ROS_INFO_STREAM("Joint 6: " << srv.response.eef_joint_6);

        }        
    }else{
        ROS_ERROR("Function %s failed to execute", srv.request.function_name.c_str());
    }

    return 0;

}
