#include "ros/ros.h"
#include "schunk_gripper_communication/schunk_gripper.h"
#include <cstdlib>
#include <stdlib.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "schunk_gripper_client");
    
        if (argc != 3){
            ROS_INFO("usage: schunk_gripper_client *function name* *motor value*");
            return 1;
        }
    
    //setenv("PYTHONPATH",".",1);
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<schunk_gripper_communication::schunk_gripper>("choose_function");

    schunk_gripper_communication::schunk_gripper srv;
    srv.request.function_name = argv[1];
    srv.request.motorvalue = atof(argv[2]);
    
    //ROS_INFO("input: %s %s %f",srv.request.pythonfile, srv.request.pythonfunction,(double)srv.request.motorvalue);

    while(1){
        if (client.call(srv)){
            ROS_INFO("Function %s executed successfully", srv.request.function_name.c_str());
            break;
        }else{
            ROS_ERROR("Function %s failed to execute", srv.request.function_name.c_str());
            ROS_INFO("Calling function %s again", srv.request.function_name.c_str());

            continue;
        }
    }
    return 0;

}
