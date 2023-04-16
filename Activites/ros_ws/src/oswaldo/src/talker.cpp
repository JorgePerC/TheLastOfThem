//Libreria base de ROS
#include <ros/ros.h>
#include <stdio.h>
#include <iostream>

// Incluir librerias de mensajes
#include <std_msgs/Int64.h>
#include <geometry_msgs/PointStamped.h>

int main(int argc, char** argv){
    //Inicializar nodo
    ros::init(argc, argv, "Nodo");

    //Iniciar nodehandle
    ros::NodeHandle n("~");

    // Crear instancia de clase Publisher
    ros::Publisher emisor = n.advertise<std_msgs::Int64>("michael_jackson/entero", 1);
    ros::Publisher punto = n.advertise<geometry_msgs::PointStamped>("/freddy_mercury/punto", 1);

    // Mensaje
    std::cout << "Hello world" << std::endl;

    // Inicializar indice de publicacion (timer)
    ros::Rate loop_rate(15);

    // Inicializar mensaje
    std_msgs::Int64 msg;
    geometry_msgs::PointStamped msg_t;

    // Crear contador
    int counter = 0;
    float timer = 0.0;

    // Crear ciclo
    while(ros::ok()){
        // Fijar datos del mensaje
        msg.data = counter;
        counter++;

        // Llenar informacion temporal del mensaje
        msg_t.header.stamp = ros::Time::now();

        // LLenar traslacion del punto
        msg_t.point.x = sin(timer);
        msg_t.point.y = cos(timer);
        msg_t.point.z = -2.0*sin(timer);

        // Aumentar timer
        timer = timer + 1.0/15.0;

        // Invocar al publisher
        emisor.publish(msg);
        punto.publish(msg_t);

        // Hacer llamado a retardo
        loop_rate.sleep();
    }

    return 0;
}