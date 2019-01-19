#ifndef MOTOR_CONTROL_TALONNODE_H
#define MOTOR_CONTROL_TALONNODE_H

#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "motor_control/TalonConfig.h"
#include <algorithm>
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <dynamic_reconfigure/server.h>

namespace motor_control{
    class TalonNode{
    private:
        ros::NodeHandle nh;
        std::string _name;
        dynamic_reconfigure::Server<motor_control::TalonConfig> server;

        std::unique_ptr<TalonSRX> talon;

        ros::Publisher tempPub;
        ros::Publisher busVoltagePub;
        ros::Publisher outputPercentPub;
        ros::Publisher outputVoltagePub;
        ros::Publisher posPub;
        ros::Publisher velPub;

        ros::Subscriber setPercentSub;
        ros::Subscriber setVelSub;

        ros::Time lastUpdate;
        ControlMode _controlMode;
        double _output;

    public:
        TalonNode(ros::NodeHandle parent, std::string name, const TalonConfig &config);

        TalonNode& operator=(const TalonNode&) = delete;

        ~TalonNode() = default;

        void reconfigure(const TalonConfig &config, uint32_t level);

        void setPercentOutput(std_msgs::Float64Ptr output);

        void setVelocity(std_msgs::Float64Ptr output);

        void stop();

        void update();
    };

}

#endif //MOTOR_CONTROL_TALONNODE_H
