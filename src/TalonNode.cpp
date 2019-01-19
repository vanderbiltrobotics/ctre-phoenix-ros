#include <ros/node_handle.h>

#include "ctre/phoenix/motorcontrol/VelocityMeasPeriod.h"
#include "motor_control/TalonNode.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

namespace motor_control {

    TalonNode::TalonNode(ros::NodeHandle nh, std::string name, const TalonConfig &config) :
            nh(nh), server(nh), talon(new TalonSRX(config.id)),
            tempPub(nh.advertise<std_msgs::Float64>("temperature", 1)),
            busVoltagePub(nh.advertise<std_msgs::Float64>("bus_voltage", 1)),
            outputPercentPub(nh.advertise<std_msgs::Float64>("output_percent", 1)),
            outputVoltagePub(nh.advertise<std_msgs::Float64>("output_voltage", 1)),
            posPub(nh.advertise<std_msgs::Int32>("position", 1)),
            velPub(nh.advertise<std_msgs::Int32>("velocity", 1)),
            setPercentSub(nh.subscribe("set_percent_output", 1, &TalonNode::setPercentOutput, this)),
            setVelSub(nh.subscribe("set_velocity", 1, &TalonNode::setVelocity, this)),
            lastUpdate(ros::Time::now()){
        server.setCallback(boost::bind(&TalonNode::reconfigure, this, _1, _2));
        server.updateConfig(config);
        stop();
    }

    void TalonNode::stop(){
        std_msgs::Float64Ptr output(new std_msgs::Float64());
        output->data = 0.0;
        setPercentOutput(output);
    }

    void TalonNode::setPercentOutput(std_msgs::Float64Ptr output) {
        talon->Set(ControlMode::PercentOutput, output->data);
        lastUpdate = ros::Time::now();
    }

    void TalonNode::setVelocity(std_msgs::Float64Ptr output) {
        talon->Set(ControlMode::Velocity, output->data);
        lastUpdate = ros::Time::now();
    }

    void TalonNode::reconfigure(const TalonConfig &config, uint32_t level) {
        ctre::phoenix::platform::can::SetCANInterface(config.interface.c_str());

        TalonSRXConfiguration c;
        talon->ConfigAllSettings(c);

        if (talon->GetDeviceID() != config.id) {
            talon.reset(new TalonSRX(config.id));
        }
        talon->SetInverted(config.inverted);
        talon->ConfigVoltageCompSaturation(config.peak_voltage);
        talon->EnableVoltageCompensation(true);
    }

    void TalonNode::update(){
        if(ros::Time::now() - lastUpdate > ros::Duration(1)){
            ROS_WARN("Talon SRX not receiving commands!");
            stop();
        }

        std_msgs::Float64Ptr temperature(new std_msgs::Float64());
        temperature->data = talon->GetTemperature();
        tempPub.publish(temperature);

        std_msgs::Float64Ptr busVoltage(new std_msgs::Float64());
        busVoltage->data = talon->GetBusVoltage();
        busVoltagePub.publish(busVoltage);

        std_msgs::Float64Ptr outputPercent(new std_msgs::Float64());
        outputPercent->data = talon->GetMotorOutputPercent();
        outputPercentPub.publish(outputPercent);

        std_msgs::Float64Ptr outputVoltage(new std_msgs::Float64());
        outputVoltage->data = talon->GetMotorOutputVoltage();
        outputVoltagePub.publish(outputVoltage);

        std_msgs::Int32Ptr position(new std_msgs::Int32());
        position->data = talon->GetSelectedSensorPosition();
        posPub.publish(position);

        std_msgs::Int32Ptr velocity(new std_msgs::Int32());
        velocity->data = talon->GetSelectedSensorVelocity();
        velPub.publish(velocity);
    }

}
