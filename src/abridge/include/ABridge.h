#ifndef ABRIDGE_H
#define ABRIDGE_H

//ROS libraries
#include <ros/ros.h>
#include <tf/tf.h>

//ROS messages
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>

//Local header
#include "USBSerial.h"

class ABridge {
public:

    ABridge();
    virtual ~ABridge();

    //ROS publishers
    ros::Publisher imuPublish;
    ros::Publisher odomPublish;
    ros::Publisher sonarLeftPublish;
    ros::Publisher sonarCenterPublish;
    ros::Publisher sonarRightPublish;

    //ROS subscriber
    ros::Subscriber commandVelocitySubscriber;

    //ROS timer
    ros::Timer publishTimer;

    //Callback handlers
    void commandVelocityHandler(const geometry_msgs::Twist message);
    void serialActivityTimer(const ros::TimerEvent& e);

    //Variables
    std::string tfPrefix;
    USBSerial usbSerial;

};

#endif /* ABRIDGE_H */
