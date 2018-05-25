#!/bin/bash

#rosrun schunk_gripper_communication schunk_gripper_client plan_motion 0.0557729 0.241261 0.238582 -0.0080936 0.00598691 0.699692 0.714373 
#rosrun schunk_gripper_communication schunk_gripper_client plan_motion 0.0951655 0.5467 0.419534 -0.0284695 0.0139533 0.65619 0.753929

rosrun schunk_gripper_communication schunk_gripper_client plan_motion 0.11247 0.670281 0.412233 0.0416113 0.726271 0.683848 0.0561275

#new in Shnk RF:
#(wxyz): 0.0561275, 0.0416113, 0.726271, 0.683848
#(xyz): 0.11247, 0.670281, 0.412233 


#(wxyz): 0.753929, -0.0284695, 0.0139533, 0.65619
#   (xyz): 0.0951655, 0.5467, 0.419534

#wp1:
#    (wxyz): 0.714373, -0.0080936, 0.00598691, 0.699692
#(xyz): 0.0557729, 0.241261, 0.238582 
#wp2:
#(wxyz): 0.714373, -0.0080936, 0.00598691, 0.699692
#   (xyz): 0.0572053, 0.310129, 0.237212 




#rosrun schunk_gripper_communication schunk_gripper_client plan_motion 0.0572053, 0.310129, 0.237212 0.714373, -0.0080936, 0.00598691, 0.699692


