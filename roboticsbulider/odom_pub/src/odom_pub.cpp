#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include <iostream>
#include <ncurses.h>

// Callback function for the left encoder subscriber
void encoderLeftCallback(const std_msgs::Int32::ConstPtr& msg) {
    ROS_INFO("Encoder Left Count: %d", msg->data);
}

// Callback function for the right encoder subscriber
void encoderRightCallback(const std_msgs::Int32::ConstPtr& msg) {
    ROS_INFO("Encoder Right Count: %d", msg->data);
}

int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "motor_control_node");
    // Create a NodeHandle to communicate with the ROS system
    ros::NodeHandle nh;

    // Set up publishers for motor A and motor B speed commands
    ros::Publisher motorAPub = nh.advertise<std_msgs::Int16>("motorA", 10);
    ros::Publisher motorBPub = nh.advertise<std_msgs::Int16>("motorB", 10);

    // Set up subscribers for the encoder readings
    ros::Subscriber encoderLeftSub = nh.subscribe<std_msgs::Int32>("encoderLeft", 10, encoderLeftCallback);
    ros::Subscriber encoderRightSub = nh.subscribe<std_msgs::Int32>("encoderRight", 10, encoderRightCallback);

    // Initialize ncurses
    initscr();
    cbreak();
    noecho();
    timeout(100);  // Non-blocking getch with timeout
    keypad(stdscr, TRUE);

    // Set the loop rate to 10 Hz
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        std_msgs::Int16 motorA;
        std_msgs::Int16 motorB;

        // Read keyboard input
        int ch = getch();
        switch (ch) {
            case 'f':  // Move forward
                motorA.data = 255;
                motorB.data = 255;
                break;
            case 'r':  // Move reverse
                motorA.data = -255;
                motorB.data = -255;
                break;
            default:
                motorA.data = 0;
                motorB.data = 0;
                // No valid key pressed
                break;
        }

        // Publish the motor speeds
        motorAPub.publish(motorA);
        motorBPub.publish(motorB);

        // Handle all pending callbacks
        ros::spinOnce();

        // Sleep for the remaining time to maintain the loop rate
        loop_rate.sleep();
    }

    // End ncurses mode
    endwin();

    return 0;
}