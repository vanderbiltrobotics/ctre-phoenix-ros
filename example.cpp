#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

const uint64_t MOVE = 0;
const uint64_t ROTATE = 1;

TalonSRX* fl = nullptr;
TalonSRX* fr = nullptr;
TalonSRX* bl = nullptr;
TalonSRX* br = nullptr;

void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  ctre::phoenix::unmanaged::FeedEnable(100);
  double moveValue = msg->linear.x;
  double rotateValue = msg->angular.z;
  double leftMotorOutput = 0.0;
  double rightMotorOutput = 0.0;
  if (moveValue > 0.0) {
    if (rotateValue > 0.0) {
      leftMotorOutput = moveValue - rotateValue;
      rightMotorOutput = std::max(moveValue, rotateValue);
    } else {
      leftMotorOutput = std::max(moveValue, -rotateValue);
      rightMotorOutput = moveValue + rotateValue;
    }
  } else {
    if (rotateValue > 0.0) {
      leftMotorOutput = -std::max(-moveValue, rotateValue);
      rightMotorOutput = moveValue + rotateValue;
    } else {
      leftMotorOutput = moveValue - rotateValue;
      rightMotorOutput = -std::max(-moveValue, -rotateValue);
    }
  }
  fl->Set(ControlMode::PercentOutput, leftMotorOutput);
  fr->Set(ControlMode::PercentOutput, leftMotorOutput);
  bl->Set(ControlMode::PercentOutput, rightMotorOutput);
  br->Set(ControlMode::PercentOutput, rightMotorOutput);
  ROS_INFO("Move=%f Rotate=%f", moveValue, rotateValue);
  ROS_INFO("FL=%d FR=%d BL=%d BR=%d",
          fl->GetSensorCollection().GetQuadraturePosition(),
          fr->GetSensorCollection().GetQuadraturePosition(),
          bl->GetSensorCollection().GetQuadraturePosition(),
          br->GetSensorCollection().GetQuadraturePosition());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "motor_controller");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("cmd_vel", 1000, cmdCallback);

    std::string interface = "can0";
    ctre::phoenix::platform::can::SetCANInterface(interface.c_str());

    fl = new TalonSRX(1);
    fr = new TalonSRX(2);
    bl = new TalonSRX(4);
    br = new TalonSRX(3);

    fr->SetInverted(true);
    br->SetInverted(true);

    ros::spin();
    return 0;
}
