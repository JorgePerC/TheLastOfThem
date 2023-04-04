#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Int64.h>


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle nh;

    ros::Publisher chatter_pub = nh.advertise<std_msgs::Int64>("chatter", 10);
    
    ros::Rate loop_rate(10);

    int count = 0;

    while(ros::ok()){
        std_msgs::Int64 msg;
        msg.data = count++;

        chatter_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;   
}

