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

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "camera_view");
    ros::NodeHandle nh ("~");

    ros::Rate loop_rate(25);

    // Crear objetos del tipo ImageTransport
        // Objeto para enviar los mensajes por CV_bridge
    image_transport::ImageTransport it(nh);

    image_transport::Publisher img_pub = it.advertise("/camera/rgb", 1);

    // Convierte entre imagen de ROS y Np matrix
    cv_bridge::CvImage cv_b;

    // Video capture (camera object)
    cv::VideoCapture camIn;

    // Start recieving camera input
    camIn.open(0);

    while (ros::ok())
    {
        /* code for loop body */
        cv::Mat img;
        
        camIn.read(img);

        cv::resize(img, img, cv::Size(640,480));

        // Automacally converts the image to ROS msg
        cv_b.image = img;

        cv_b.header.stamp = ros::Time::now();
        
        cv_b.encoding = sensor_msgs::image_encodings::BGR8;
    
        // This is format independent, based on the one we declared. 
        img_pub.publish(cv_b.toImageMsg());

        ros::spinOnce();
        loop_rate.sleep();
    }

    camIn.release();
    
}