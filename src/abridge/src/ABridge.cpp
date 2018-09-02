#include "ABridge.h"

ABridge::ABridge() {
}

ABridge::~ABridge() {
}

void ABridge::commandVelocityHandler(const geometry_msgs::Twist message) {
    //Format data into C-style string and send movement command over serial
    char moveCmd[30];
    sprintf(moveCmd, "v,%f,%f\n", message.linear.x, message.angular.z); 
    usbSerial.sendData(moveCmd);
}

void ABridge::serialActivityTimer(const ros::TimerEvent& e) {
    //Send data request command over serial
    char dataCmd[] = "d\n";
    usbSerial.sendData(dataCmd);

    //Create ROS messages to hold serial response data
    sensor_msgs::Imu imu;
    nav_msgs::Odometry odom;
    sensor_msgs::Range sonarLeft;
    sensor_msgs::Range sonarCenter;
    sensor_msgs::Range sonarRight;

    //Parse Arduino response data and store in ROS messages
    std::string str = usbSerial.readData();
    std::istringstream oss(str);
    std::string sentence;
    
    while (getline(oss, sentence, '\n')) {
        std::istringstream wss(sentence);
        std::string word;

        std::vector<std::string> dataSet;
        while (getline(wss, word, ',')) {
                dataSet.push_back(word);
        }

        if (dataSet.size() >= 3 && dataSet.at(1) == "1") {
            if ((dataSet.at(0) == "IMU") && (dataSet.size() == 11)) {
                imu.header.stamp = ros::Time::now();
                imu.linear_acceleration.x = atof(dataSet.at(2).c_str());
                imu.linear_acceleration.y = 0; //atof(dataSet.at(3).c_str());
                imu.linear_acceleration.z = atof(dataSet.at(4).c_str());
                imu.angular_velocity.x = atof(dataSet.at(5).c_str());
                imu.angular_velocity.y = atof(dataSet.at(6).c_str());
                imu.angular_velocity.z = atof(dataSet.at(7).c_str());
                imu.orientation = tf::createQuaternionMsgFromRollPitchYaw(atof(dataSet.at(8).c_str()), atof(dataSet.at(9).c_str()), atof(dataSet.at(10).c_str()));
            }
            else if ((dataSet.at(0) == "ODOM") && (dataSet.size() == 8)) {
                odom.header.stamp = ros::Time::now();
                odom.pose.pose.position.x = atof(dataSet.at(2).c_str());
                odom.pose.pose.position.y = atof(dataSet.at(3).c_str());
                odom.pose.pose.position.z = 0.0;
                odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(atof(dataSet.at(4).c_str()));
                odom.twist.twist.linear.x = atof(dataSet.at(5).c_str());
                odom.twist.twist.linear.y = atof(dataSet.at(6).c_str());
                odom.twist.twist.angular.z = atof(dataSet.at(7).c_str());
            }
            else if ((dataSet.at(0) == "USL") && (dataSet.size() == 3)) {
                sonarLeft.header.stamp = ros::Time::now();
                sonarLeft.range = atof(dataSet.at(2).c_str()) / 100.0;
            }
            else if ((dataSet.at(0) == "USC") && (dataSet.size() == 3)) {
                sonarCenter.header.stamp = ros::Time::now();
                sonarCenter.range = atof(dataSet.at(2).c_str()) / 100.0;
            }
            else if ((dataSet.at(0) == "USR") && (dataSet.size() == 3)) {
                sonarRight.header.stamp = ros::Time::now();
                sonarRight.range = atof(dataSet.at(2).c_str()) / 100.0;
            }
        }
    }

    //Set ROS frame IDs
    imu.header.frame_id = tf::resolve(tfPrefix, "base_link");
    odom.header.frame_id = tf::resolve(tfPrefix, "odom");
    odom.child_frame_id = tf::resolve(tfPrefix, "base_link");
    sonarLeft.header.frame_id = tf::resolve(tfPrefix, "sonarLeft");
    sonarCenter.header.frame_id = tf::resolve(tfPrefix, "sonarCenter");
    sonarRight.header.frame_id = tf::resolve(tfPrefix, "sonarRight");

    // Publish data
    imuPublish.publish(imu);
    odomPublish.publish(odom);
    sonarLeftPublish.publish(sonarLeft);
    sonarCenterPublish.publish(sonarCenter);
    sonarRightPublish.publish(sonarRight);
}

int main(int argc, char **argv) { 
    ros::init(argc, argv, "abridge");
    ros::NodeHandle nh("~");

    //Instantiate ABridge
    ABridge aBridge;    

    //Open USB serial connection
    std::string devicePath;
    int baudRate;
    nh.param("devicePath", devicePath, std::string("/dev/ttyUSB0"));
    ROS_INFO_STREAM("Device path for serial communication is set to " << devicePath);
    nh.param("baudRate", baudRate, 115200);
    ROS_INFO_STREAM("Serial communication data rate is set to " << baudRate << " baud");
    if (aBridge.usbSerial.openUSBPort(devicePath, baudRate) <= 0) {
        ROS_ERROR_STREAM("Failed to successfully open a serial connection to device path " << devicePath);
    }
    
    //Connect publishers
    aBridge.imuPublish = nh.advertise<sensor_msgs::Imu>("imu", 10);
    aBridge.odomPublish = nh.advertise<nav_msgs::Odometry>("odom", 10);
    aBridge.sonarLeftPublish = nh.advertise<sensor_msgs::Range>("sonarLeft", 10);
    aBridge.sonarCenterPublish = nh.advertise<sensor_msgs::Range>("sonarCenter", 10);
    aBridge.sonarRightPublish = nh.advertise<sensor_msgs::Range>("sonarRight", 10);
    
    //Connect subscriber
    aBridge.commandVelocitySubscriber = nh.subscribe("commandVelocity", 10, &ABridge::commandVelocityHandler, &aBridge);
    
    //Initialize timer
    double timerPeriod;
    nh.param("timerPeriod", timerPeriod, 0.1);
    ROS_INFO_STREAM("Time interval between serial data requests is set to " << timerPeriod << " seconds");
    aBridge.publishTimer = nh.createTimer(ros::Duration(timerPeriod), &ABridge::serialActivityTimer, &aBridge);
    
    //Load TF prefix string (used for TF frame name resolution)
    ros::NodeHandle anh;
    std::string tf_prefix;
    anh.getParam("tf_prefix", aBridge.tfPrefix);
    ROS_INFO_STREAM("TF prefix is set to " << aBridge.tfPrefix);

    //Begin event-driven execution of this node
    ros::spin();
    
    return EXIT_SUCCESS;
}
