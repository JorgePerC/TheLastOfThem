//Libreria base de ROS
#include <ros/ros.h>
#include <stdio.h>
#include <iostream>

// Incluir librerias de mensajes
#include <std_msgs/Int64.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>

// Inicializar parametros
double cr, cg, cb, ca; // Color
double ox, oy, oz; // Orbita
double sx, sy, sz; // Escala

int main(int argc, char** argv){
    //Inicializar nodo
    ros::init(argc, argv, "Nodo");

    //Iniciar nodehandle
    ros::NodeHandle n("~");

    // Leer parametros (color)
    n.param<double>("color/r", cr, 1.0);
    n.param<double>("color/g", cg, 1.0);
    n.param<double>("color/b", cb, 1.0);
    n.param<double>("color/a", ca, 1.0);

    // Leer parametros (orbit)
    n.param<double>("orbit/x", ox, 1.0);
    n.param<double>("orbit/y", oy, 1.0);
    n.param<double>("orbit/z", oz, 1.0);

    // Leer parametros
    n.param<double>("size/x", sx, 1.0);
    n.param<double>("size/y", sy, 1.0);
    n.param<double>("size/z", sz, 1.0);

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
    msg.color.r = cr; 
    msg.color.g = cg;
    msg.color.b = cb;
    msg.color.a = ca;

    // Definir escala del objeto
    msg.scale.x = sx;
    msg.scale.y = sy;
    msg.scale.z = sz;

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
        msg.pose.position.x = ox*sin(timer);
        msg.pose.position.y = oy*cos(timer);
        msg.pose.position.z = oz*sin(timer) + cos(timer);

        // Aumentar timer
        timer = timer + 1.0/15.0;

        // Incovar a publisher
        marker_pub.publish(msg);

        // Hacer llamado a retardo
        loop_rate.sleep();
    }

    return 0;
}