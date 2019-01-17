#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include <algorithm>
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Float64.h>

#include "motor_control/TalonConfig.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

TalonSRX* talon = nullptr;

void setPercentOutput(std_msgs::Float64 output){
    talon->Set(ControlMode::PercentOutput, output.data);
}

void reconfigure(motor_control::TalonConfig &config, uint32_t level) {
    ctre::phoenix::platform::can::SetCANInterface(config.interface.c_str());
    if(talon == nullptr || talon->GetDeviceID() != config.id){
        delete talon;
        talon = new TalonSRX(config.id);
    }
    auto percentOutput = std_msgs::Float64();
    percentOutput.data = config.percent_output;
    setPercentOutput(percentOutput);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "talon_srx");
    ros::NodeHandle nh("talon_srx");

    dynamic_reconfigure::Server<motor_control::TalonConfig> server;
    dynamic_reconfigure::Server<motor_control::TalonConfig>::CallbackType f;

    f = boost::bind(&reconfigure, _1, _2);
    server.setCallback(f);

    // Verify Device ID parameter
    if(!nh.hasParam("id")){
        throw std::invalid_argument("Missing Device ID parameter!");
    }

    // Create the initial device configuration
    motor_control::TalonConfig config;
    config.interface = nh.getParam("interface", config.interface);
    config.id = nh.getParam("id", config.id);
    config.percent_output = 0;

    // Reconfigure/Create the device
    reconfigure(config, 0);

    ros::Subscriber sub = nh.subscribe("set_percent_output", 1, setPercentOutput);

    ROS_INFO("Spinning node");
    ros::spin();
    return 0;
}
