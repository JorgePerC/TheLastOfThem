//Libreria base de ROS
#include <ros/ros.h>
#include <stdio.h>
#include <iostream>

// Incluir librerias de mensajes
#include <std_msgs/Int64.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv){
    //Inicializar nodo
    ros::init(argc, argv, "Nodo");

    //Iniciar nodehandle
    ros::NodeHandle n("~");

    // Crear instancia de clase Publisher
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/particula", 1);

    // Mensaje
    std::cout << "Hello world" << std::endl;

    // Inicializar indice de publicacion (timer)
    ros::Rate loop_rate(15);

    // Inicializar mensaje
    visualization_msgs::Marker msg;

    // Definir propiedades basicas del marcador
    msg.id = 0;
    msg.type = 2;

    // Definir marca de referencia (capa de visualizacion de RVIZ)
    msg.header.frame_id = "world";

    //Definir color de la esfera
    msg.color.r = 0.8; 
    msg.color.g = 0.7;
    msg.color.b = 0.6;
    msg.color.a = 0.5;

    // Definir escala del objeto
    msg.scale.x = 1.0;
    msg.scale.y = 1.0;
    msg.scale.z = 1.0;

    // Orientacion del objeto
    msg.pose.orientation.w = 1.0;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;

    // Tiempo simulado
    float timer = 0.0;

    // Crear ciclo
    while(ros::ok()){
        // Llenar informacion temporal del mensaje
        msg.header.stamp = ros::Time::now();

        // Lenar traslacion del punto
        msg.pose.position.x = 5.0*sin(timer);
        msg.pose.position.y = 3.0*cos(timer);
        msg.pose.position.z = sin(timer) + cos(timer);

        // Aumentar timer
        timer = timer + 1.0/15.0;

        // Incovar a publisher
        marker_pub.publish(msg);

        // Hacer llamado a retardo
        loop_rate.sleep();
    }

    return 0;
}