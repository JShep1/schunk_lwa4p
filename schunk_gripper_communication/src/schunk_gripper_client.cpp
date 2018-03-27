#include "ros/ros.h"
#include "schunk_gripper_communication/schunk_gripper.h"
#include <cstdlib>
#include <stdlib.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "schunk_gripper_client");
    schunk_gripper_communication::schunk_gripper srv;
    

        if(argc == 5){
            double box_x = atof(argv[2]);
            double box_y = atof(argv[3]);
            double box_z = atof(argv[4]);
            
            if(box_x <= 0 || box_y <= 0 || box_z <= 0){
                ROS_INFO("Invalid values input. Try again.");
                return 0;
            }

            srv.request.box_x = box_x; 
            srv.request.box_y = box_y; 
            srv.request.box_z = box_z; 
        }else if (argc == 3){
            srv.request.motorvalue = atof(argv[2]);
            return 1;
        }else if (argc == 2){
            srv.request.motorvalue = -1;
        }else{
            ROS_INFO("usage: schunk_gripper_client *function name* OPTIONAL-*motor value*");
            return 0;
        }
            srv.request.function_name = argv[1];
    
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
