#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <custom_msgs/point.h>
#include <custom_msgs/pixel.h>
    // Note how we import .h file, and not .msgs


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "cumstomTalker");
    ros::NodeHandle nh;

    ros::Publisher chatter_pub = nh.advertise<custom_msgs::point>("chatter", 10);
    
    ros::Rate loop_rate(10);

    int count = 0;

    while(ros::ok()){
        custom_msgs::point msg;
        msg.pimg.u.data = 0;
        msg.pimg.v.data = 1;

        msg.pose.position.x = count++;
        msg.pose.position.y = count--;
        msg.pose.position.z = count++;

        msg.pose.orientation.w = 0;
        

        chatter_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;   
}

