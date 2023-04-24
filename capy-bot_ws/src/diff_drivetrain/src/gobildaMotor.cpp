#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
//#include <gpio

#define maxFreq 1050
#define minFreq 1950

float vel;

void velocityCallback(const std_msgs::Float32 &msg)
{
    // Aquire data
    vel = msg.data;
    // Limit in case it exceeds threshold
    if (vel > 1.0){
        vel = 1.0;
    }
    if (vel < -1.0){
        vel = -1.0;
    }
}

float velocity2Frequency (float v){
    // Desfasamos para que siempre sea positivo
    float t = v + 1.0;
    // Hacemos el mapeo
    return 1050 + t*900/2;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "gobildaMotor");
    ros::NodeHandle nh;

    ros::Subscriber inputSubs = nh.subscribe<std_msgs::Float32>("cmd_vel", 10, velocityCallback);
    
    while(ros::ok()){
        
    }
}