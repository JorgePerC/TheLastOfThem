#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Int64.h>


void callback_chatter(const std_msgs::Int64 &msg){
    ROS_INFO("I heard: [%s]", msg.data);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle nh;

    
    ros::Subscriber sub = nh.subscribe("chatter", 10, callback_chatter);
    
    ros::spin();

    return 0;
}