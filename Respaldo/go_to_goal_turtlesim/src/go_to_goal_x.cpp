/// @file go_to_goal_x.cpp
/// @author Addison Sears-Collins
/// @date May 4, 2021
/// 
/// This program takes the coordinates of the turtlesim robot as input
/// and outputs a velocity command as a Twist message. The velocity command
/// will be calculated so as to get the turtlesim robot to move to a goal 
/// x coordinate that can be any number from 0 to 11.
 
#include <cstdlib> // Use for the absolute value method abs()
#include <iostream> // Enables command line input and output
 
#include "ros/ros.h" // Necessary header files for ROS
#include "geometry_msgs/Twist.h" // Twist messages (linear & angular velocity)
#include "geometry_msgs/Pose2D.h" // x, y position and theta orientation
#include "turtlesim/Pose.h" // x, y, theta, linear & angular velocity
 
// Remove the need to use std:: prefix
using namespace std;
 
// Key variable declarations 
geometry_msgs::Twist velCommand; // Linear and angular velocity in m/s 
geometry_msgs::Pose2D current; // Current x, y, and theta 
geometry_msgs::Pose2D desired; // Desired x, y, and theta 
 
// Goal x-value, which can be any number from 0 to 11 (inclusive)
const double GOAL = 1.3;
 
// The gain K, which is used to calculate the linear velocity
const double K_l = 1.0;
 
// The distance threshold in meters that will determine when 
// the turtlesim robot successfully reaches the goal.
const double distanceTolerance = 0.1;
 
// Initialized variables and take care of other setup tasks
void setup() {
 
  // Desired x goal coordinate
  desired.x = GOAL;
   
  // Initialize the Twist message.
  // Initial linear and angular velocities are 0 m/s and rad/s, respectively.
  velCommand.linear.x = 0.0;
  velCommand.linear.y = 0.0;
  velCommand.linear.z = 0.0;
  velCommand.angular.x = 0.0;
  velCommand.angular.y = 0.0;
  velCommand.angular.z = 0.0;
}
 
// Get the distance between the current x coordinate and 
// the desired x coordinate.
double getDistanceToGoal() {
  return desired.x - current.x;
}
 
// If we haven't yet reached the goal, set the velocity value.
// Otherwise, stop the robot.
void setVelocity() {
  if (abs(getDistanceToGoal()) > distanceTolerance) {
 
    // The magnitude of the robot's velocity is directly
    // proportional to the distance the robot is from the 
    // goal.
    velCommand.linear.x = K_l * getDistanceToGoal();
  }
  else {
    cout << "Goal has been reached!" << endl << endl;
    velCommand.linear.x = 0;
  }
}
 
// This callback function updates the current position and 
// orientation of the robot. 
void updatePose(const turtlesim::PoseConstPtr &currentPose) {
  current.x = currentPose->x;
  current.y = currentPose->y;
  current.theta = currentPose->theta;
}
 
int main(int argc, char **argv) {
 
  setup();  
 
  // Initiate ROS
  ros::init(argc, argv, "go_to_goal_x");
     
  // Create the main access point to communicate with ROS
  ros::NodeHandle node;
 
  // Subscribe to the robot's pose
  // Hold no messages in the queue. Automatically throw away 
  // any messages that are received that are not able to be
  // processed quickly enough.
  // Every time a new pose is received, update the robot's pose.
  ros::Subscriber currentPoseSub =
    node.subscribe("turtle1/pose", 0, updatePose);
 
  // Publish velocity commands to a topic.
  // Hold no messages in the queue. Automatically throw away 
  // any messages that are received that are not able to be
  // processed quickly enough.
  ros::Publisher velocityPub =
    node.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 0);
 
  // Specify a frequency that want the while loop below to loop at
  // In this case, we want to loop 10 cycles per second
  ros::Rate loop_rate(10); 
 
  // Keep running the while loop below as long as the ROS Master is active. 
  while (ros::ok()) {
 
    // Here is where we call the callbacks that need to be called.
    ros::spinOnce();
 
    // After we call the callback function to update the robot's pose, we 
    // set the velocity values for the robot.
    setVelocity();
 
    // Publish the velocity command to the ROS topic
    velocityPub.publish(velCommand);
 
    // Print the output to the console
    cout << "Current x = " << current.x << endl
         << "Desired x = " << desired.x <<  endl
         << "Distance to Goal = " << getDistanceToGoal() << " m" << endl
         << "Linear Velocity (x) = " << velCommand.linear.x << " m/s" << endl
         << endl;
 
    // Sleep as long as we need to to make sure that we have a frequency of
    // 10Hz
    loop_rate.sleep();
  }
 
  return 0;
}