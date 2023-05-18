#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>

// Global variables
geometry_msgs::Twist userGeometryCommand;
bool newCommandReceived = false;

// Handle user commands callback function
void commandCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    userGeometryCommand = *msg;
    newCommandReceived = true;
}

int main(int argc, char** argv)
{
    // Initialize ROS node, Name: remote_control_node
    ros::init(argc, argv, "remote_control_node");
    ros::NodeHandle nh;

    // Create publisher; topic: turtlebot/cmd_vel
    ros::Publisher commandPublisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Create subscriber; topic: remote_control/cmd_vel
    ros::Subscriber commandSubscriber = nh.subscribe<geometry_msgs::Twist>("/remote_control/cmd_vel", 10, commandCallback);

    
    ros::Rate loopRate(10); 
    while (ros::ok())
    {
        // if new user command has been received, publish the command and reset control variable
        if (newCommandReceived)
        {
            commandPublisher.publish(userGeometryCommand);
            newCommandReceived = false;
        }

        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
