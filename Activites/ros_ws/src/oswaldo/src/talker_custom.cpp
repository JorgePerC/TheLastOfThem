//Libreria base de ROS
#include <ros/ros.h>
#include <stdio.h>
#include <iostream>

// Incluir librerias de mensajes
#include <std_msgs/Int64.h>
#include <geometry_msgs/PointStamped.h>
#include <custom_msgs/point.h>

int main(int argc, char** argv){
    //Inicializar nodo
    ros::init(argc, argv, "Nodo");

    //Iniciar nodehandle
    ros::NodeHandle n("~");

    // Crear instancia de clase Publisher
    ros::Publisher emisor = n.advertise<custom_msgs::point>("/talker/msg", 1);

    // Inicializar indice de publicacion (timer)
    ros::Rate loop_rate(15);

    // Inicializar mensaje
    custom_msgs::point msg;

    // Tiempo simulado
    float t = 0.0;

    // Crear ciclo
    while(ros::ok()){
        // Llenar atributos
        msg.pimg.u.data = 50;
        msg.pimg.v.data = 100;
        msg.pose.position.x = sin(t);
        msg.pose.position.y = cos(t);
        msg.pose.position.z = 2*sin(t);
        msg.header.stamp = ros::Time::now();

        // Publicar
        emisor.publish(msg);

        // Tiempo simulado
        t = t + 1.0/15.0;

        // Hacer llamado a retardo
        loop_rate.sleep();
    }

    return 0;
}