#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <iostream>

using tcp = boost::asio::ip::tcp;
namespace beast = boost::beast;
namespace http = beast::http;
// bool hasBeenPressed = false;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

MoveBaseClient* moveBaseClient;

// ROS publisher to send user commands
ros::Publisher commandPublisher;
double linearVelocity, angularVelocity;
// Handler for incoming HTTP requests
void handle_request(const http::request<http::string_body>& req, http::response<http::string_body>& res)
{

    // Extract the command from the request
    std::string command = req.body();
    
    // Parse the command and create the Twist message
    geometry_msgs::Twist twistMsg;
   
   // if move_base then handle move_base request
   // if remote control then handle geometry::msgs cmd_vel request.

    if (command.find("move_base") == 0) // looking for move_base request
    {
        // if(hasBeenPressed){
        //     hasBeenPressed = false;
        // }

        std::string moveBaseCommand = command.substr(10);  // Remove "move_base," from the command string
        std::istringstream iss(moveBaseCommand);
        std::string x, y, w; // string variable to hold the commands coming from the server
        if (std::getline(iss, x, ',') && std::getline(iss, y, ',') && std::getline(iss, w, ','))  
        {
            // Convert x, y, and w values to appropriate data types
            double goalX = std::stod(x); // use string to double function to convert string literal of command into double 
            double goalY = std::stod(y);
            double goalW = std::stod(w);

            // Create a MoveBaseClient instance if not does not already exist
            if (!moveBaseClient) 
            {
                moveBaseClient = new MoveBaseClient("move_base", true);
                // Wait for action server 
                moveBaseClient->waitForServer();
            }

            // Create move_base action goal and give the target pose 
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = goalX;
            goal.target_pose.pose.position.y = goalY;
            goal.target_pose.pose.orientation.w = goalW;

            // Send to action server
            moveBaseClient->sendGoal(goal);

            // Wait for the action to complete
            moveBaseClient->waitForResult();

            if (moveBaseClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("Goal reached successfully"); // indicate to the user that the action finished successfully and the goal was reached
            } else {
                ROS_WARN("Failed to reach the goal"); // else warn the user that something went wrong
            }

            // Cleanup moveBaseClient object
            delete moveBaseClient;
            moveBaseClient = nullptr;

            res.result(http::status::ok); // send OK back to server to indicate successfull transmission
            res.set(http::field::content_type, "text/plain"); // set the content type of data
            res.body() = "Move base command received successfully"; // send successfull receive of data back to server.

        }
    }
    else
    {
        // else the teleop function has been chosen by the user and handle the commands
        if (command == "forward")
        {
                // Small increment in linear velocity by 0.1 each time forward button has been pressed
                //if robot angularvelocity are greater or less than 0, set the linear velocity to forward motion and keep the angular velocity
                if(angularVelocity > 0.0001 || angularVelocity < -0.0001){

                    linearVelocity += 0.1;
                    twistMsg.linear.x = linearVelocity; 
                    twistMsg.angular.z = angularVelocity;

                }
                else{ //else set linear velocity to forward motion and ensure angular velocity is 0

                    linearVelocity += 0.1;
                    twistMsg.linear.x = linearVelocity; 
                    twistMsg.angular.z = 0.0;

                }

        }
        else if (command == "backward")
        {  
            if(angularVelocity > 0.0001 || angularVelocity < -0.0001){
                linearVelocity -= 0.1;
                twistMsg.linear.x = linearVelocity;
                twistMsg.angular.z = angularVelocity;
            }else{
                linearVelocity -= 0.1;
                twistMsg.linear.x = linearVelocity;
                twistMsg.angular.z = 0.0;
            }
            
            // if(linearVelocity > 0.0)
            // {
            //     linearVelocity -= 0.1;
            // }
            // else
            // {
            //     linearVelocity -= 0.1;
            // }
            
            // twistMsg.linear.x = linearVelocity;  // Set the linear velocity for backward motion
        }
        else if (command == "right"){
            // angularVelocity -= 1.0;  
            // twistMsg.angular.z = angularVelocity;
            double linearVelocityThreshold = 0.0001;  // set a threshold as the linear velocity might not be exactly zero
            
            if (std::abs(linearVelocity) < linearVelocityThreshold) {
                // If the linear velocity is apprx. zero, change the angular velocity only
                angularVelocity -= 0.1;   
                twistMsg.angular.z = angularVelocity;
                twistMsg.linear.x = 0.0;  // Ensure robot is not moving linearly
            } else {
                // else the robot is already in motion linearly, the robot will move both linearly and angularly. resulting in a circle pattern
                twistMsg.linear.x = linearVelocity;

                // Check which way the robot is moving to determine the turn direction
                if (linearVelocity > 0) { // if the robot is moving forward
                    angularVelocity -= 0.1;   // Set angular velocity for right a turn
                } else {
                    angularVelocity += 0.1;   // else if moving backwards set angular velocity for left a right the other way
                }

                twistMsg.angular.z = angularVelocity;
            }
        }

        /* ALSO WORKS */
        // else if (command == "left"){   

        //     if(linearVelocity < 0.0001){            //this if statement is for if the robot is in motion linearly make it also turn
        //         twistMsg.linear.x = linearVelocity;
        //         angularVelocity += 0.1;         // Set the angular velocity for left turn while in motion
        //         twistMsg.angular.z = angularVelocity;
            
            
        //     }
        //     else{

        //         angularVelocity += 1.0;         // Else set only angular velocity if linear velocity is 0 so the robot only turns in its place
        //         twistMsg.angular.z = angularVelocity;
        //         twistMsg.linear.x = 0.0;

        //     }
        // }

        else if (command == "left") {
            double linearVelocityThreshold = 0.0001;  
            
            if (std::abs(linearVelocity) < linearVelocityThreshold) {
                
                angularVelocity += 0.1;   
                twistMsg.angular.z = angularVelocity;
                twistMsg.linear.x = 0.0; 
            } else {
            
                twistMsg.linear.x = linearVelocity;

                if (linearVelocity > 0) { 
                    angularVelocity += 0.1; 
                } else {
                    angularVelocity -= 0.1; 
                }

                twistMsg.angular.z = angularVelocity;
            }
        }


        else if (command == "stop")
        {
            // Reset linear and angular velocity and stop the robot completely
            linearVelocity = 0.0;
            angularVelocity = 0.0;
            twistMsg.linear.x = linearVelocity;
            twistMsg.angular.z = angularVelocity;

        }
        // Publish the Twist message; Topic: /remote_control/cmd_vel
        commandPublisher.publish(twistMsg);
        
        // handle HTTP specific stuff
        res.result(http::status::ok); // send OK back to server to indicate successfull transmission
        res.set(http::field::content_type, "text/plain"); // set the content type of data
        res.body() = "Move base command received successfully"; // send successfull receive of data back to server.
        // hasBeenPressed = true;
       
    }

    
}


int main(int argc, char** argv)
{
    // Initialize ROS node; Name : web_server_node
    ros::init(argc, argv, "web_server_node");
    ros::NodeHandle nh;

    // Create publisher; topic: /remote_control/cmd_vel
    commandPublisher = nh.advertise<geometry_msgs::Twist>("/remote_control/cmd_vel", 10);

    // Set up HTTP configuration
    boost::asio::io_context ioc;
    tcp::acceptor acceptor(ioc, tcp::endpoint(tcp::v4(), 8080));

    // Accept incoming connections and handle requests asynchronously
    while (ros::ok())
    {
        // Create and accept new connection via socket
        tcp::socket socket(ioc);
        acceptor.accept(socket);

        // create buffer for storing, read request from server and handle the request
        beast::flat_buffer buffer; 
        http::request<http::string_body> request;
        http::read(socket, buffer, request);
        http::response<http::string_body> response;

        handle_request(request, response);

        http::write(socket, response); //write the response to the socket
    }

    return 0;
}
