#include <ros/ros.h>

#include <stdio.h>
#include <iostream>

#include <std_msgs/Int64.h>
#include <geometry_msgs/PointStamped.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// Convierte entre imagen de ROS y Np matrix
cv_bridge::CvImage cv_bridge;
image_transport::Publisher img_pub;

void actionUponImage(const sensor_msgs::ImageConstPtr &msg)
{
    cv:: Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

    cv_bridge.image = image;

    cv_bridge.header.stamp = ros::Time::now();
    
    cv_bridge.encoding = sensor_msgs::image_encodings::BGR8;

    // This is format independent, based on the one we declared. 
    img_pub.publish(cv_bridge.toImageMsg()); 
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "camera_stalker");
    ros::NodeHandle nh ("~");

    ros::Rate loop_rate(25);

    // Crear objetos del tipo ImageTransport
        // Objeto para enviar los mensajes por CV_bridge
    image_transport::ImageTransport it(nh);

    img_pub = it.advertise("camera/gray", 1);

    image_transport::Subscriber img_sub = it.subscribe("/camera/rgb", 1, actionUponImage);

    while (ros::ok())
    {
        /* code for loop body */

        ros::spinOnce();
        loop_rate.sleep();
    }
    
}