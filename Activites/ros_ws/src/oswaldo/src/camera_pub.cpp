//Libreria base de ROS
#include <ros/ros.h>

// Librerias base
#include <stdio.h>
#include <iostream>

// Incluir librerias de mensajes
#include <std_msgs/Int64.h>
#include <geometry_msgs/PointStamped.h>

// Agregar dependencias de OpenCV
# include <opencv2/highgui/highgui.hpp>
# include <opencv2/core.hpp>
# include <opencv2/imgproc.hpp>

// Agregar dependencias de ROS para manipulacion de imagenes
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// Estructura de mensajes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

int main(int argc, char** argv){
    //Inicializar nodo
    ros::init(argc, argv, "cpp_camera");

    //Iniciar nodehandle
    ros::NodeHandle n("~");

    // Crear objeto de image transport
    image_transport::ImageTransport it(n);

    // Informar a nodehandle de publicaciones
    image_transport::Publisher img_pub = it.advertise("/camara/rgb", 1);

    // Crear cv-bridge
    cv_bridge::CvImage cv_b;

    // Crear objeto de videoCapture de OpenCV
    cv::VideoCapture input_camera;

    // Abrir camara
    input_camera.open(0);

    // Inicializar indice de publicacion (timer)
    ros::Rate loop_rate(15);

    // Crear ciclo
    while(ros::ok()){

        // Obtener frame de camara
        cv::Mat image;
        input_camera.read(image);

        // Redimendionar imagen
        cv::resize(image, image, cv::Size(640, 480));

        // Asignar imagen a cv_bridge
        cv_b.image = image;
        cv_b.header.stamp = ros::Time::now();
        cv_b.encoding = sensor_msgs::image_encodings::BGR8;

        // Publicar imagen (convertir OpenCV-ROS usando cv_bridge)
        img_pub.publish(cv_b.toImageMsg());

        // Hacer llamado a retardo
        loop_rate.sleep();
    }

    return 0;
}