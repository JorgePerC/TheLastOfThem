//Libreria base de ROS
#include <ros/ros.h>
#include <stdio.h>
#include <iostream>

// Incluir librerias de mensajes
#include <std_msgs/Int64.h>

// Inicializar callback
void IntCallBack(const std_msgs::Int64 msg);

int main(int argc, char** argv){
    //Inicializar nodo
    ros::init(argc, argv, "Subs");

    //Iniciar nodehandle
    ros::NodeHandle n("~");

    // Inicializar subscriber
    ros::Subscriber subs = n.subscribe("Nodo/michael_jackson/entero", 1, IntCallBack);

    // Inicializar indice de publicacion (timer)
    ros::Rate loop_rate(15);

    // Crear ciclo
    while(ros::ok()){
        // Hacer llamado a los callbacks
        ros::spinOnce();

        // Hacer llamado a retardo
        loop_rate.sleep();
    }

    return 0;
}

// Implementar callback
void IntCallBack(const std_msgs::Int64 msg){
    std::cout << "recibi: " << msg.data << std::endl;
}