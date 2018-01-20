#include <stdlib.h>
#include <Python.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "schunk_gripper_communication/schunk_gripper.h"

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
    Py_Initialize();
    PyRun_SimpleString(str1);
    Py_Finalize();
    ROS_INFO("Python Script Complete");

    ROS_INFO("Setting motor value to %f", motorvalue);
    ROS_INFO("Completed motor movement - returning.");
    return true;
}

bool plan_motion(){
    ROS_INFO("Motion planning in progress.");
    ROS_INFO("Motion planning complete.");
    return true;
}

bool choose_function(schunk_gripper_communication::schunk_gripper::Request &req,
                     schunk_gripper_communication::schunk_gripper::Response &res)
{
    res.motorvalue = req.motorvalue;
    std::string function_1 = "set_motor";
    std::string function_2 = "plan_motion";
    if(!function_1.compare((std::string)req.function_name)){
        ROS_INFO("Found function set_motor");
        if(set_motor(res.motorvalue)){
            ROS_INFO("Successfully set motor.");
            return true;
        }else{
            ROS_ERROR("Could not set motor. Returning.");
            return false;
        }
    }else if(!function_2.compare((std::string)req.function_name)){
        ROS_INFO("Found function plan_motion");
        if(plan_motion()){
            ROS_INFO("Successfully created motion plan.");
            return true;
        }else{
            ROS_ERROR("Could not create motion plan. Returning.");
            return false;
        }
        
    }
    else{
        //std::stringstream ss;
        //ss << (std::string)req.function_name;
        //char str1[50];
        //str
        //const char* str = ss.str().c_str();

        ROS_ERROR("No such function: %s", req.function_name.c_str());
        return false;
    }
    //std::string str = convertToStr<double>(&(res.motorvalue));
    //std::cout << str << std::endl;
/*
    std::stringstream ss;
    ss << (double)req.motorvalue;
    char str1[50];
    const char* str = ss.str().c_str();
    strcpy(str1, "motorvalue = ");
    strcat(str1, str);
    strcat(str1, "\nprint motorvalue \n");
    ROS_INFO("Running Python Script");
    //std::cout << str1 << std::endl;
    //ROS_INFO("%s",((double)res.motorvalue).str().c_str());
    Py_Initialize();
    PyRun_SimpleString(str1);
    //PyRun_SimpleString(ss.str());
    Py_Finalize();
    ROS_INFO("Python Script Complete");

    ROS_INFO("Setting motor value to %f", (double)res.motorvalue);
    ROS_INFO("Completed motor movement - returning.");
  */
    return true;
}




/*

bool set_motor(schunk_gripper_communication::schunk_gripper::Request &req,
               schunk_gripper_communication::schunk_gripper::Response &res)
{
    res.motorvalue = req.motorvalue;
    res.pythonfile = req.pythonfile;
    res.pythonfunction = req.pythonfunction;
    ROS_INFO("Setting motor value to %f", (double)res.motorvalue);
    ROS_INFO("Completed motor movement - returning.");
    return true;
}

bool set_motor(schunk_gripper_communication::schunk_gripper::Request &req,
               schunk_gripper_communication::schunk_gripper::Response &res)
{
   
    //python here
    PyObject *pName, *pModule, *pDict, *pFunc;
    PyObject *pArgs, *pValue;
    int i;

    
    res.pythonfile = req.pythonfile;
    res.pythonfunction = req.pythonfunction;
    res.motorvalue = req.motorvalue;
   
    std::string pythonfile = req.pythonfile;
    std::string pythonfunction = req.pythonfunction;

    ROS_INFO("Setting motor value to %f", (double)res.motorvalue);

    setenv("PYTHONPATH",".",1);

    Py_Initialize();
    pName = PyString_FromString(((std::string)res.pythonfile).c_str());
    pModule = PyImport_Import(pName);
    Py_DECREF(pName);

    if (pModule != NULL) {
        pFunc = PyObject_GetAttrString(pModule, ((std::string)res.pythonfunction).c_str());

        if (pFunc && PyCallable_Check(pFunc)) {
            pArgs = PyTuple_New(1); //num of args intended to pass to function
            pValue = PyFloat_FromDouble((double)res.motorvalue);
            PyTuple_SetItem(pArgs, 0, pValue);
            pValue = PyObject_CallObject(pFunc, pArgs);
            Py_DECREF(pArgs);
            if (pValue != NULL) {
                printf("Result of call: %ld\n", PyInt_AsLong(pValue));
                Py_DECREF(pValue);
            }
            else {
                Py_DECREF(pFunc);
                Py_DECREF(pModule);
                PyErr_Print();
                fprintf(stderr,"Call failed\n");
                return false;
            }
        }
        else {
            if (PyErr_Occurred())
                PyErr_Print();
            fprintf(stderr, "Cannot find function\n");
        }
        Py_XDECREF(pFunc);
        Py_DECREF(pModule);
    } else {
        PyErr_Print();
        fprintf(stderr, "Failed to load python file\n");
        return false;
    }
    Py_Finalize();
    

    ROS_INFO("Completed motor movement - returning.");
    
    return true;
}
*/


int main(int argc, char **argv)
{
    ros::init(argc, argv, "schunk_gripper_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("choose_function", choose_function);
    ROS_INFO("Ready to run a function.");
    ros::spin();
    return 0;
}
