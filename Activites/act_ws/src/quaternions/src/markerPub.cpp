#include <iostream>
#include <stdio.h>

#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle nh;

    ros::Publisher chatter_pub = nh.advertise<std_msgs::Int64>("chatter", 10);

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/particula", 10);
    
    ros::Rate loop_rate(10);

    visualization_msgs::Marker msg;

    // Tiempo simulado
    float timer = 0.0;

    // Definir características globales del marker
        // La figura viene de la documentación
    msg.id = 0;
    msg.type = 2; // Decir que es una esfera
    msg.header.frame_id = "worldPlane";// Marco de referencia, respecto a un plano. 

    msg.color.r = 0.8; 
    msg.color.g = 0.4;
    msg.color.b = 0.3;
    msg.color.a = 0.8; //Opacity
    
    msg.scale.x = 1.0;
    msg.scale.y = 2.0;
    msg.scale.z = 1.0;

    // Quaternion based
    msg.pose.orientation.w = 1.0;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;


    while(ros::ok()){
        
        msg.header.stamp = ros::Time::now ();

        msg.pose.position.x = 5.0*sin(timer);
        msg.pose.position.y = 3.0*cos(timer);
        msg.pose.position.z = cos(timer) + sin (timer);

        timer += 1.0/15.0;

        marker_pub.publish(msg);

        loop_rate.sleep();
    }
    return 0;   
}

