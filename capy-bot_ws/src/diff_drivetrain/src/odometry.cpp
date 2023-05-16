#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "dodmetry");
    ros::NodeHandle nh;

    ros::Subscriber inputSubs = nh.subscribe<std_msgs::Float32>("cmd_vel", 10, velocityCallback);
    
    while(ros::ok()){
        
    }
}