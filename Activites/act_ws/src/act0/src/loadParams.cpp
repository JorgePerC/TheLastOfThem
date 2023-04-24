#include <ros/ros.h>

#include <iostream>

#include <std_msgs/Int64.h>

int cppParam = 0;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "rosParam");
    ros::NodeHandle nh("~");
    
                            // Group/param from .yaml
                                                // C++ Variable
                                                        // Default value
    nh.param<int>("myGroup/param1", cppParam, 0);
    
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        /* code for loop body */
        std::cout << "The param value is: " << cppParam << std::endl;
        ros::spinOnce();
        loop_rate.sleep();
    }
      
}