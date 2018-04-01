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
        }else if(argc == 5){
            //x, y, and z supplied so go there
            srv.request.function_name = argv[1];
            double x = atof(argv[2]);
            double y = atof(argv[3]);
            double z = atof(argv[4]);
            
            srv.request.plan_x = x; 
            srv.request.plan_y = y; 
            srv.request.plan_z = z; 

        }else{
            //Invalid number of args
            ROS_ERROR("Invalid plan_motion call");
            ROS_ERROR("For random plan:");
            ROS_ERROR("Usage: schunk_gripper_client plan_motion");
            ROS_ERROR("For specified plan:");
            ROS_ERROR("Usage: schunk_gripper_client plan_motion *x_val* *y_val* *z_val*");
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
        if(argc == 5){
            double box_x = atof(argv[2]);
            double box_y = atof(argv[3]);
            double box_z = atof(argv[4]);
            
            if(box_x <= 0 || box_y <= 0 || box_z <= 0){
                ROS_ERROR("Invalid motor values. Try again.");
                return 0;
            }

            srv.request.box_x = box_x; 
            srv.request.box_y = box_y; 
            srv.request.box_z = box_z; 
        
        }else{
            ROS_ERROR("Invalid create_box call");
            ROS_ERROR("Usage: schunk_gripper_client create_box *x_val* *y_val* *z_val*");
            return 0;
        }

    }else{
        ROS_ERROR("Invalid function call.");    
        ROS_ERROR("Valid function calls are set_motor, plan_motion, execute_motion, create_box");   
        return 0; 
    }

    //setenv("PYTHONPATH",".",1);
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<schunk_gripper_communication::schunk_gripper>("choose_function");

    
    //ROS_INFO("input: %s %s %f",srv.request.pythonfile, srv.request.pythonfunction,(double)srv.request.motorvalue);

    while(1){
        if (client.call(srv)){
            ROS_INFO("Function %s executed successfully", srv.request.function_name.c_str());
            break;
        }else{
            ROS_ERROR("Function %s failed to execute", srv.request.function_name.c_str());
            //ROS_INFO("Calling function %s again", srv.request.function_name.c_str());

            //continue;
            break;
        }
    }
    return 0;

}
